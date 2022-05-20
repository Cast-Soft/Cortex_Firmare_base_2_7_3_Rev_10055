#ifndef _SERIAL_COM_ENDPOINT_H_
#define _SERIAL_COM_ENDPOINT_H_

#include <map>
#include <deque>
#include <string>
#include <vector>

#include <boost/asio.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp> 
#include <boost/assign/std/vector.hpp>

namespace serial_com {
// Com port communication end point
struct EndPoint {
	std::string comPort;	// like COM1, COM2 ... 	
	std::string deviceID;

	boost::asio::serial_port_base::baud_rate baudRate;
    boost::asio::serial_port_base::parity optParity;
    boost::asio::serial_port_base::character_size charSize;
    boost::asio::serial_port_base::flow_control flowControl;
    boost::asio::serial_port_base::stop_bits stopBit;

	EndPoint()
	: baudRate(115200)
	, optParity(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none))
    , charSize (boost::asio::serial_port_base::character_size(8))
	, flowControl(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none))
	, stopBit(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one)) {
		
	}
};

};
#endif