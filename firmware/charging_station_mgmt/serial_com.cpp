// serial_com.cpp
#include <sstream>
#include <boost/bind.hpp>
#include <boost/make_shared.hpp>

#include <iostream>
#include <assert.h>
#include "serial_com.h"
#include "serial_com_messenger.h"

using namespace serial_com;
using boost::asio::serial_port_base;

serial_com::CommErrorCategory serial_com::commErrCat;

BaseComm::BaseComm(Messenger* msgr)
  : asyncWork(new boost::asio::io_service::work(ioService)), 
    asyncThread(boost::bind(&boost::asio::io_service::run, &ioService)),
    messenger(msgr) {
	assert(messenger);
}

BaseComm::~BaseComm(){
  asyncWork.reset(); 
  ioService.stop(); 
  asyncThread.join(); 
}

void BaseComm::AsyncConnect(EndPoint peerEndpoint, CommContext connSessionId) {
	SharedControlBlock sharedCB(boost::make_shared<ControlBlock>(ioService));
    ControlBlockMap::iterator it = controlBlocks.find(connSessionId);
    if (it != controlBlocks.end()) {
        boost::system::error_code error(ErrorCode::duplicated_connection_context, commErrCat);
        assert(messenger);
        messenger->AsyncCloseConnectionHandler(connSessionId, PassiveCloseId, error);
        return;
    }

	try {
		sharedCB->serialPort.open(peerEndpoint.comPort);				// boost asio issue: it does not provide an asyn open
		sharedCB->serialPort.set_option(peerEndpoint.baudRate);
		sharedCB->serialPort.set_option(peerEndpoint.optParity);
		sharedCB->serialPort.set_option(peerEndpoint.charSize);
		sharedCB->serialPort.set_option(peerEndpoint.flowControl);
		sharedCB->serialPort.set_option(peerEndpoint.stopBit);
	}
	catch(...) {
        boost::system::error_code error(ErrorCode::device_open_error, commErrCat);
        assert(messenger);
        messenger->AsyncCloseConnectionHandler(connSessionId, PassiveCloseId, error);		
		return;
	}

    controlBlocks[connSessionId] = sharedCB;
    sharedCB->peerEndpoint = peerEndpoint;

    messenger->AsyncConnectHandler(connSessionId);

    AsyncReceiveBody(connSessionId);        // starts receiving without user instruction
}

void BaseComm::AsyncWrite(const std::vector<unsigned char> buffer, CommContext connSessionId, CommContext writeId) {
    ControlBlockMap::iterator it = controlBlocks.find(connSessionId);
    if (it == controlBlocks.end()) {
        boost::system::error_code error(ErrorCode::missing_connection_control_block, commErrCat);
       InternalCloseConnectionHandler(connSessionId, PassiveCloseId, error);
        return;
    }

    SharedControlBlock& sharedCB = it->second;
    SharedPacket sharedPacket(new Packet);
    sharedPacket->SetBuffer(buffer);
    sharedPacket->writeId = writeId;
    sharedCB->writePacketQueue.push_back(sharedPacket);

    if (sharedCB->writePacketQueue.size() == 1) {
        SharedPacket& sharedPacket = sharedCB->writePacketQueue.front();
        std::vector<boost::asio::const_buffer> asioBuffer;
		async_write(sharedCB->serialPort, sharedPacket->GetWriteBuffer(asioBuffer),
			boost::bind(&BaseComm::AsyncWriteHandler, this, connSessionId, sharedPacket->writeId,
            boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
    }
}

void BaseComm::AsyncWriteHandler(CommContext connSessionId, CommContext writeId, const boost::system::error_code& error, std::size_t bytesTransferred) {
    ControlBlockMap::iterator it = controlBlocks.find(connSessionId);
    if (it == controlBlocks.end()) {
        boost::system::error_code error(ErrorCode::missing_connection_control_block, commErrCat);
        InternalCloseConnectionHandler(connSessionId, PassiveCloseId, error);
        return;
    }

    SharedControlBlock& sharedCB = it->second;
    ControlBlock::WritePacketQueue& queue = sharedCB->writePacketQueue;
    if (queue.begin() == queue.end()) {
        assert(0);  // might be a bug
        boost::system::error_code error(ErrorCode::missing_connection_control_block, commErrCat);
        AsyncCloseConnectionHandler(sharedCB, connSessionId, PassiveCloseId, error);
        return;
    }
    if (queue.front()->writeId != writeId) {
        assert(0);  // might be a bug
        boost::system::error_code error(ErrorCode::missing_connection_control_block, commErrCat);
        AsyncCloseConnectionHandler(sharedCB, connSessionId, PassiveCloseId, error);
        return;
    }
    if (error) {
        AsyncCloseConnectionHandler(sharedCB, connSessionId, PassiveCloseId, error);
        return;
    }

    queue.pop_front();
    messenger->AsyncWriteHandler(connSessionId, writeId, bytesTransferred);
    if (queue.size() > 0) {
        SharedPacket& sharedPacket = queue.front();
        std::vector<boost::asio::const_buffer> asioBuffer;
		async_write(sharedCB->serialPort, sharedPacket->GetWriteBuffer(asioBuffer),
            boost::bind(&BaseComm::AsyncWriteHandler, this, connSessionId, sharedPacket->writeId,
            boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
    }
}

void BaseComm::AsyncReceiveBodyHandler(CommContext connSessionId, CommContext readId, const boost::system::error_code& error, std::size_t bytesTransferred) {
    ControlBlockMap::iterator it = controlBlocks.find(connSessionId);
    if (it == controlBlocks.end()) {
        boost::system::error_code error(ErrorCode::missing_connection_control_block, commErrCat);
        InternalCloseConnectionHandler(connSessionId, PassiveCloseId, error);
        return;
    }

    SharedControlBlock& sharedCB = it->second;

    if (error) {
        AsyncCloseConnectionHandler(sharedCB, connSessionId, PassiveCloseId, error);
        return;
    }

    assert(bytesTransferred <= sharedCB->bodyBuffer.size());
	// assert(bytesTransferred > 0);	It happens
	if (bytesTransferred > 0) {
		++sharedCB->readIdGenerator;
		std::vector<unsigned char> bodyBuffer(sharedCB->bodyBuffer.begin(), sharedCB->bodyBuffer.begin() + bytesTransferred);
		messenger->AsyncReceiveHandler(connSessionId, readId, bodyBuffer);       // as read buffer is one copy only, should garantee it is passes to receiver before next read
	}
	AsyncReceiveBody(connSessionId);
}

void BaseComm::AsyncReceiveBody(CommContext connSessionId) {
    ControlBlockMap::iterator it = controlBlocks.find(connSessionId);
    if (it == controlBlocks.end()) {
        boost::system::error_code error(ErrorCode::missing_connection_control_block, commErrCat);
        InternalCloseConnectionHandler(connSessionId, PassiveCloseId, error);
        return;
    }

    SharedControlBlock& sharedCB = it->second;
    sharedCB->SetBodyBuffer(sharedCB->maxLength);       ///< Verify
	sharedCB->serialPort.async_read_some(boost::asio::buffer(&sharedCB->bodyBuffer[0], sharedCB->bodyBuffer.size()),
        boost::bind(&BaseComm::AsyncReceiveBodyHandler, this, connSessionId, sharedCB->readIdGenerator,
        boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));  
}

void BaseComm::AsyncCloseConnection(CommContext connSessionId, bool graceful, CommContext closeConnId) {
    assert(graceful == false);  // todo!!!
    ControlBlockMap::iterator it = controlBlocks.find(connSessionId);
    if (it == controlBlocks.end()) {
        boost::system::error_code error(ErrorCode::missing_connection_control_block, commErrCat);
        InternalCloseConnectionHandler(connSessionId, PassiveCloseId, error);
        return;
    }

    SharedControlBlock& sharedCB = it->second;
    boost::system::error_code error; 
    AsyncCloseConnectionHandler(sharedCB, connSessionId, closeConnId, error);
}

void BaseComm::AbortConnectionCtrlBlock(const CommContext connSessionId) {
    ControlBlockMap::iterator it = controlBlocks.find(connSessionId);
    if (it == controlBlocks.end()) {
        return;
    }

    SharedControlBlock& sharedCB = it->second;
    try {
		boost::system::error_code ec;
		sharedCB->serialPort.cancel(ec);
    }
    catch(boost::system::system_error&)
    {
    }
    try {
		boost::system::error_code ec;
		sharedCB->serialPort.close(ec);
    }
    catch(boost::system::system_error&)
    {
    }
    ioService.post(boost::bind(&BaseComm::BaseCommDeferAbortConnectionCtrlBlock, this, connSessionId));   // yield the thread for asio finish buffer cleaning up
}

void BaseComm::BaseCommDeferAbortConnectionCtrlBlock(const CommContext connSessionId) {
    ControlBlockMap::iterator it = controlBlocks.find(connSessionId);
    if (it == controlBlocks.end()) {
        return;
    }
    SharedControlBlock& sharedCB = it->second;
    controlBlocks.erase(it);
}

void BaseComm::InternalCloseConnectionHandler(CommContext connSessionId, CommContext closeConnId, const boost::system::error_code& error) {        // for debugging and logging only

}

void BaseComm::AsyncCloseConnectionHandler(SharedControlBlock& sharedCB, CommContext connSessionId, CommContext closeConnId, const boost::system::error_code& error) {
    if (sharedCB->pendingClose) {
        // closing in progress
        // a disconnection can produce 2 AbortConnectionCtrlBlock calls. One from reading and the other one from writing
        return;
    }
    sharedCB->pendingClose = true;
    AbortConnectionCtrlBlock(connSessionId);
    this->messenger->AsyncCloseConnectionHandler(connSessionId, PassiveCloseId, error);
}
