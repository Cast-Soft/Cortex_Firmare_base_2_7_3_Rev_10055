#include <iostream>

#include "serial_com_messenger.h"
#include "serial_com.h"

using namespace serial_com;
boost::asio::serial_port_base;

Messenger::Messenger()
    : baseComm(NULL) {
    baseComm = new BaseComm(this);  
}

Messenger::~Messenger() {
    assert(baseComm == NULL);       // Stop() must be called first and must not be in Messengers' (and derived classes) destructors
}

void Messenger::Stop() {       // Stop the messenger before destructor is called. Must guaranteed. Messenger uses virtual functions in another thread, destructor cannot handle correclly.
    delete baseComm;           // this is blocking wait operation 
    baseComm = NULL;
}

/// <summary>
/// forward to comm object. Comm object does not expose to users
/// </summary>
void Messenger::AsyncConnect(serial_com::EndPoint& peerEndpoint, CommContext connSessionId) {
    if (baseComm == NULL) return;   // after destructed, might have short period that baseComm is NULL
    baseComm->ioService.post(boost::bind(&BaseComm::AsyncConnect, baseComm, peerEndpoint, connSessionId));
}

void Messenger::AsyncWrite(const std::vector<unsigned char>& buffer, CommContext connSessionId, CommContext writeId) {
    if (baseComm == NULL) return;   // after destructed, might have short period that baseComm is NULL
    baseComm->ioService.post(boost::bind(&BaseComm::AsyncWrite, baseComm, buffer, connSessionId, writeId));
}

void Messenger::AsyncCloseConnection(CommContext connSessionId, bool graceful, CommContext closeConnId) {
    if (baseComm == NULL) return;   // after destructed, might have short period that baseComm is NULL
    baseComm->ioService.post(boost::bind(&BaseComm::AsyncCloseConnection, baseComm, connSessionId, graceful, closeConnId));
}