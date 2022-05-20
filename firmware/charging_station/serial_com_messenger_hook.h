#ifndef SERIAL_COM_MESSENGER_HOOK_H_
#define SERIAL_COM_MESSENGER_HOOK_H_

#include <boost/shared_ptr.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>

#include "common.h"

#include "serial_com_messenger.h"
#include "serial_com.h"

namespace serial_com {

class BaseCompletionCarrier {
public:
    virtual ~BaseCompletionCarrier() {}
};

typedef void (*DoCompletePointer)(BaseCompletionCarrier* base);

template <typename Handler>
class CompletionCarrier : public BaseCompletionCarrier
{
public:
    Handler handler;
    static void DoComplete(BaseCompletionCarrier* base)
    {
        typedef CompletionCarrier<Handler> CompletionCarrierInstance;
        CompletionCarrierInstance* completionCarrier = static_cast<CompletionCarrierInstance* >(base);
        assert(completionCarrier);
        completionCarrier->handler();
        delete completionCarrier;
    }
    CompletionCarrier<Handler>(const Handler& hdl) : handler(hdl) {}
};

class BaseComm;

/// <summary>
/// MessengerHook needs the target application provide handlers via template. Messenger does not
/// </summary>

template<class MessageTarget>
class MessengerHook : public Messenger {
    MessageTarget* messageTarget;
public:
    MessengerHook(MessageTarget* messageTarget)
        : curConnectSessionId(0)
        , messageTarget(messageTarget) {
    }

    virtual void Poster(BaseCompletionCarrier* instance, DoCompletePointer doComplete) { delete instance; }   // cannot be pure virtual as it might be called still by asio thread when messenger destructing. A basic version should provide

    template <typename Handler>
    void PostMessageTarget(const Handler& hdl) {
        typedef CompletionCarrier<Handler> CompletionCarrierInstance;
        CompletionCarrierInstance* instance = new CompletionCarrierInstance(hdl);
        DoCompletePointer func = CompletionCarrierInstance::DoComplete;
        Poster(instance, func);
    }

    void AllocateConnectSessionId(SharedSerialComConnectionInfo& connectionInfo) {     // it is up to user to choose using this function or not. It is default imlementation
        // If a connection session Id was allocated, free it first. This prevent memory leak and also user does not need to call this RemoveConnection function 
        this->RemoveConnection(connectionInfo);

        ++curConnectSessionId;
        connectionInfo->connectSessionId = curConnectSessionId;
    
        connectionBlocks[curConnectSessionId] = connectionInfo;

        assert(connectionBlocks.size() < 100);       // try to detect a memory leak -- forget to remove connection block from messenger. TODO!!! maybe using weak point is better.When connectionInfo destruct, remove from here too
    }

    void RemoveConnection(SharedSerialComConnectionInfo& ConnectionInfo) {             // not the own of connectionInfo. Just move it out
        connectionBlocks.erase(ConnectionInfo->connectSessionId);
    }

	// These are on communication thread 
    void AsyncConnectHandler(CommContext connSessionId) {
        PostMessageTarget(boost::bind(&MessengerHook::OnSerialComConnect, this, connSessionId));    
    }

    void AsyncWriteHandler(CommContext connSessionId, CommContext writeId, std::size_t bytesTransferred) {
        PostMessageTarget(boost::bind(&MessengerHook::OnSerialComWrite, this, connSessionId, writeId, bytesTransferred));   
    }

    void AsyncReceiveHandler(CommContext connSessionId, CommContext readId, const std::vector<unsigned char>& buffer) {
        PostMessageTarget(boost::bind(&MessengerHook::OnSerialComReceive, this, connSessionId, readId, buffer));         // need to copy???   
    }

    void AsyncCloseConnectionHandler(CommContext connSessionId, CommContext closeConnId, const boost::system::error_code& error) {
        PostMessageTarget(boost::bind(&MessengerHook::OnSerialComCloseConnection, this, connSessionId, closeConnId, error));      
    }

	// These are on main thread
    void OnSerialComConnect(CommContext connSessionId) {
        Sessions::iterator itSession = connectionBlocks.find(connSessionId);
        if (itSession!= connectionBlocks.end()) {
            SharedSerialComConnectionInfo& connectionInfo = itSession->second;
            assert(connectionInfo);
            connectionInfo->status = SerialConnected;          // roll the state machine
            messageTarget->OnSerialComConnect(connectionInfo);
        }
        // ignore not registered connection block. user has to override OnSerialComConnect
    }

    void OnSerialComWrite(CommContext connSessionId, CommContext writeId, std::size_t bytesTransferred) {
        Sessions::iterator itSession = connectionBlocks.find(connSessionId);
        if (itSession!= connectionBlocks.end()) {
            SharedSerialComConnectionInfo& connectionInfo = itSession->second;
            assert(connectionInfo);
            connectionInfo->writeId = writeId;
            connectionInfo->msgSent++;
            messageTarget->OnSerialComWrite(connectionInfo);
        }
        // ignore not registered connection block. user has to override
    }

    void OnSerialComReceive(CommContext connSessionId, CommContext readId, std::vector<unsigned char>& buffer) {
        Sessions::iterator itSession = connectionBlocks.find(connSessionId);
        if (itSession!= connectionBlocks.end()) {
            SharedSerialComConnectionInfo& connectionInfo = itSession->second;
            assert(connectionInfo);
            connectionInfo->readId = readId;
            connectionInfo->msgReceived++;
            messageTarget->OnSerialComReceive(connectionInfo, buffer);
        }
        // ignore not registered connection block. user has to override
    }

    void OnSerialComCloseConnection(CommContext connSessionId, CommContext closeConnId, boost::system::error_code error) {
        Sessions::iterator itSession = connectionBlocks.find(connSessionId);
        if (itSession!= connectionBlocks.end()) {
            SharedSerialComConnectionInfo connectionInfo = itSession->second;
            assert(connectionInfo);
            connectionInfo->status = SerialClosed;          // roll the state machine
            connectionBlocks.erase(itSession);
            messageTarget->OnSerialComCloseConnection(connectionInfo, closeConnId, error);
        }
        // ignore not registered connection block. user has to override
    }

#if 0			// As it is simple, manage in user space
    virtual StateMachineRet StartConnectHelper(SharedSerialComConnectionInfo& connectionInfo) {
        if (connectionInfo->status != SerialComClosed) {
            std::stringstream ss;
            return std::make_pair(1, "Connection is not closed yet");
        } 

        this->AllocateConnectSessionId(connectionInfo);
        assert(connectionInfo->connectSessionId);

        connectionInfo->status = SerialComConnecting;
        this->AsyncConnect(connectionInfo->endpoint, connectionInfo->connectSessionId);
        return std::make_pair(0, "");
    }

    virtual StateMachineRet StopHelper(SharedSerialComConnectionInfo& connectionInfo) {
        if (connectionInfo->status != SerialComConnected) {
            return std::make_pair(1, "Connection is not established yet");
        } 

        connectionInfo->status = SerialComClosing;
        this->AsyncCloseConnection(connectionInfo->connectSessionId, false, 0);
        return std::make_pair(0, "");
    }
#endif
private:
    CommContext curConnectSessionId;            // for generating unique conenction session id
    Sessions connectionBlocks;                  // key is connection session id. It is not responsible for creating the block, which is done in user space
                                                // User is also reponsible for remove from this map
};

}
#endif      // SERIAL_COM_MESSENGER_HOOK_H_