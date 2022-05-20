#ifndef SERIAL_COM_MESSENGER_H_
#define SERIAL_COM_MESSENGER_H_

#include <string>

#include <boost/shared_ptr.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/optional/optional.hpp>

#include <common/packets.h>
#include "common.h"
#include "endpoint.h"

namespace serial_com {

class BaseComm;

enum SerialComStatus {        // small state machine
    SerialConnecting,
    SerialConnected,
    SerialClosing,
    SerialClosed
};

struct SerialComConnectionInfo {               // Serial communication control block
    CommContext blockId;                        // user defined Id. messenger does not care and it is up to user to define
    std::string name;                           // assign a name for it so that easy to trace
    serial_com::EndPoint endpoint;							// com port & settings
    int msgSent;
    int msgReceived;
    CommContext writeId;
    CommContext writtenId;
    CommContext readId;
    SerialComStatus status;
    CommContext connectSessionId;

    // add addtional info here. TODO!!! subclass them so that this file does not need to modify and can be shared
    std::string firmwareDeviceId;          // Each SerialComConnectionInfo should always assocciate with one
	std::string serialBuffer;		// for receive
	boost::optional<WhoAmI> whoAmI;
	boost::optional<RespFirmwareVersion> version;
	boost::optional<BeaconRunningStatus> deviceStatus;		// battery and wireless on / off
	boost::optional<Beacon_BatData> batteryStatus;		// battery. New format
    std::string configBodyPacket;                           // the config packet body received

	// product info received
	std::string productInfo;

    int currentFirmwareSegmentIndex;        // -1 not start any upload session yet. >= 0. Current segment (Has just been sent)

    SerialComConnectionInfo(std::string name, CommContext blockId = 0) 
        : msgSent(0)
        , msgReceived(0)
        , writeId(0)
        , writtenId(0)
        , readId(0)
        , status(SerialClosed)
        , connectSessionId(-2)
        , name(name)
        , blockId(blockId)
		, currentFirmwareSegmentIndex(-1) {}      // todo use optional from boost
};

typedef boost::shared_ptr<SerialComConnectionInfo> SharedSerialComConnectionInfo;
typedef std::map<CommContext, SharedSerialComConnectionInfo> Sessions;                // key: session id

#if 0
// These functions simplify the starting of state machine. User may use Asy* but has to manage themselves
typedef std::pair<int, std::string> StateMachineRet;
#endif

/// <summary>
/// An object that coordindates CommonInterface running on main thread (or equivalence) and BaseComm running on communication thread(s) 
/// It is a plateform and framework specific interface
/// No template is used here
/// this object is shared by 2 threads. So be aware of locking issues
/// Usually singleton is enough for an application
/// TODO: split this file into 2: one is generic and another is for Win32
/// </summary>
class Messenger {
public:
    Messenger();
    virtual ~Messenger();

    ///< communication primitives: main thread
    void AsyncConnect(serial_com::EndPoint& peerEndpoint, CommContext connSessionId); ///< interfaceAddress is for multicast only 
    void AsyncWrite(const std::vector<unsigned char>& buffer, CommContext connSessionId, CommContext writeId);
    void AsyncCloseConnection(CommContext connSessionId, bool graceful, CommContext closeConnId);

    ///< These are running in communication threads 
    virtual void AsyncConnectHandler(CommContext connSessionId) {}
    virtual void AsyncWriteHandler(CommContext connSessionId, CommContext writeId, std::size_t bytesTransferred) {}
    virtual void AsyncReceiveHandler(CommContext connSessionId, CommContext readId, const std::vector<unsigned char>& buffer) {}
    virtual void AsyncCloseConnectionHandler(CommContext connSessionId, CommContext closeConnId, const boost::system::error_code& error) {}

    void Stop();        // Stop the messenger before destructor is called. Must guaranteed. Messenger uses virtual functions in another thread, destructor cannot handle correclly.
                        // After Stop is called, no more IO operations
    BaseComm* baseComm;
};

}
#endif      // SERIAL_COM_MESSENGER_H_