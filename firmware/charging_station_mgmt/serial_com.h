#ifndef BLACKTRAX_SERIAL_COM_H_
#define BLACKTRAX_SERIAL_COM_H_

#include "common.h"

#include <map>
#include <deque>
#include <string>
#include <vector>

#include <boost/asio.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp> 
#include <boost/assign/std/vector.hpp>
using namespace boost::assign; // bring 'operator+=()' into scope

#include "endpoint.h"

namespace serial_com {
class Messenger;

class CommErrorCategory : public boost::system::error_category {
public:
    const char* name() const {return "Base Comm"; }
    std::string message( int ev ) const {return "Base Comm Error"; }
};

const CommContext PassiveCloseId = -1;    // passive close connection Id

extern CommErrorCategory commErrCat;
namespace ErrorCode {
    enum ErrorType {                                         // ErrorCode convention 0 cannot be an error
          no_error = 0
        , duplicated_connection_context = 1                  // When trying to connect on an existing connection session
        , missing_connection_control_block                   // connection_context incorrect
        , header_format_error                                //
		, device_open_error									 // When opening COM device	
    };
};

/// <summary>
/// The top-level serial port communication object
/// It handles serial port only
/// Upper layer access this class via Connection Session Id, which identifies a serial com port connection
/// Connection Session Id is unqiue in the life cycle of this object
/// Connection seesion life cycle is longer than underlying com port implementation. That is why it is introduced
/// It is supposed to run on individual thread(s)
/// </summary>
class BaseComm {
    friend Messenger;
public:
    // communication primitives
    void AsyncConnect(EndPoint peerEndpoint, CommContext connSessionId);
    void AsyncWrite(const std::vector<unsigned char> buffer, CommContext connSessionId, CommContext writeId);
    void AsyncCloseConnection(CommContext connSessionId, bool graceful, CommContext closeConnId);
    boost::asio::io_service ioService;                                              ///< os io service. can make it singleton and share with server

protected:
    BaseComm(Messenger* messenger);
    virtual ~BaseComm();
    class Packet {
        std::vector<unsigned char> body;
    public:
        CommContext writeId;
        void SetBuffer(const std::vector<unsigned char>& source) {
            body = source;
        }
        const std::vector<boost::asio::const_buffer> GetWriteBuffer(std::vector<boost::asio::const_buffer>& buf) {
            buf.push_back(boost::asio::buffer(body));
            return buf;
        }
    };

    typedef boost::shared_ptr<Packet> SharedPacket;
    struct ControlBlock {   // Control Block
        static const size_t MaxLength = 1024 * 20;       ///< default receiving packet size of UDP
        typedef std::deque<SharedPacket> WritePacketQueue;
        WritePacketQueue writePacketQueue;
        std::vector<unsigned char> bodyBuffer;
        boost::asio::serial_port serialPort;
        CommContext readIdGenerator;
        bool pendingClose;                              ///< cannot destruct this socket CB as it still holding sending and reading buffers. After socket close, should yield thread for asio to handle first 
        size_t maxLength;
        EndPoint  peerEndpoint;
        ControlBlock(boost::asio::io_service& ioService, size_t maxLength = MaxLength) 
            : serialPort(ioService), readIdGenerator(0), pendingClose(false), maxLength(maxLength)  {
        }
        void SetBodyBuffer(size_t bodySize) {
            bodyBuffer.resize(bodySize);
        }
    };

    Messenger* messenger;
    boost::scoped_ptr<boost::asio::io_service::work> asyncWork;                     ///< Make io_service run / stop
    boost::thread asyncThread;                                                      ///< A thread runs io_serivce. One thread per BaseComm

    typedef boost::shared_ptr<ControlBlock> SharedControlBlock;                     ///< handled by auto pointer
    typedef std::map<CommContext, SharedControlBlock> ControlBlockMap;              ///< container holds all connection sessions
    ControlBlockMap controlBlocks;
  
private:
    void AsyncWriteHandler(const CommContext connSessionId, const CommContext writeId, const boost::system::error_code& error, std::size_t bytesTransferred);
    void AsyncReceiveBodyHandler(const CommContext connSessionId, const CommContext readId, const boost::system::error_code& error, std::size_t bytesTransferred);
    void AsyncConnectHandler(CommContext connSessionId, const boost::system::error_code& error);
    void AsyncReceiveBody(CommContext connSessionId);

    void AbortConnectionCtrlBlock(const CommContext connSessionId);    // no exception and error return

    void InternalCloseConnectionHandler(CommContext connSessionId, CommContext closeConnId, const boost::system::error_code& error);        // for debugging and logging only

    void BaseCommDeferAbortConnectionCtrlBlock(const CommContext connSessionId);        // let asio clean up first
    void AsyncCloseConnectionHandler(SharedControlBlock& sharedCB, CommContext connSessionId, CommContext closeConnId, const boost::system::error_code& error);

    DISALLOW_COPY_AND_ASSIGN(BaseComm);
};

};

#endif // BLACKTRAX_SERIAL_COM_H_