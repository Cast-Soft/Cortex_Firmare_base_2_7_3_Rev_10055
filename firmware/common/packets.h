// packets.h
// Header file with all packet types, enums, structs etc...

#ifndef BEACON_COMMON_PACKETS_H
#define BEACON_COMMON_PACKETS_H

// Notet that this defined in radio_def.h in beacon project
#define NUM_OF_IMU_PKTS_IN_RF_PKT           5

#include <stdint.h>	

#pragma pack(push, 1)

#define TK_FRAME_SIGNATURE 0x86F3A9CE

#define FIRMWARE_BLOCK_LENGTH 58
//const uint16_t FIRMWARE_BLOCK_LENGTH = 58;

enum TK_PACKET_TYPES {
	TK_PT_TKPacketFrame						= 0x00000000,
	TK_PT_TKPacketDiscovery					= 0x00000001,
	TK_PT_TKPacketDiscoveryACK				= 0x00000002,
        TK_PT_TKPacketStatusSummaryRequest		= 0x00000003,
	TK_PT_TKPacketStatusSummary			= 0x00000004,
        TK_PT_TKPacketStatusDetailRequest		= 0x00000005,
	TK_PT_TKPacketStatusDetail				= 0x00000006,
//#endif
        TK_PT_TKPacketRadioControl				= 0x00000007,
	TK_PT_TKPacketRadioControlACK			= 0x00000008,
	TK_PT_TKPacketSyncControl				= 0x00000009,
	TK_PT_TKPacketSyncControlACK			= 0x00000023,
	TK_PT_TKPacketSetEEPROM					= 0x00000010,
	TK_PT_TKPacketReadEEPROM				= 0x00000011,
	TK_PT_TKPacketReadEEPROMResponse		= 0x00000012,
	TK_PT_TKBCPacketIMU						= 0x00000013,
	TK_PT_TKBCPacketButton					= 0x00000014,
	TK_PT_TKBCPacketRadioFrequency			= 0x00000015,
	TK_PT_TKBCPacketStatus					= 0x00000016,
	TK_PT_TKBCPacketSetBeaconNumber			= 0x00000017,
	TK_PT_TKBCPacketSetBeaconNumberACK		= 0x00000018,
	TK_PT_TKBCPacketFlashLED				= 0x00000019,
	TK_PT_TKBCPacketFlashLEDACK				= 0x00000020,
	TK_PT_TKBCPacketPowerMode				= 0x00000021,
	TK_PT_TKBCPacketPowerModeACK			= 0x00000022,

	BC_PT_USBBCDetailRequest				= 0x00000024,
	BC_PT_USBBCDetail						= 0x00000025,
	BC_PT_USBBCRadioDetailRequest			= 0x00000026,
	BC_PT_USBBCRadioDetail					= 0x00000027,
	BC_PT_USBBCSetAuthKey					= 0x00000028,
	BC_PT_USBBCSetAuthKeyACK				= 0x00000029,

	BC_PT_USBBCFirmwareIOSetup				= 0x00000030,
	BC_PT_USBBCFirmwareIOSetupACK			= 0x00000031,
	BC_PT_USBBCSetFirmwareMemoryBlock		= 0x00000032,
	BC_PT_USBBCSetFirmwareMemoryBlockACK	= 0x00000033,
	BC_PT_USBBCSetFirmwareMemoryBlockRequest= 0x00000034,
        BC_PT_TKBCForwardAny                    = 0x00000035,
        TK_PT_TKPacketPassThroughRequest		= 0x00000036,
	TK_PT_TKPacketPassThroughRequestStatus			= 0x00000037,

	TK_PT_BASE = 0x00000000,
	TK_PT_NEXT_ID = 0x00000038
};

enum TK_RADIO_STATUS
{
	TK_RS_OFF =0x00,
	TK_RS_ON_RECIEVER= 0x01,
	TK_RS_OFF_TANSCIEVER= 0x02,
	TK_RS_ON_TANSCIEVER= 0x03
};

enum TK_SYNC_STATUS
{
	TK_SS_OFF= 0x00,
	TK_SS_ON_AND_RESUME= 0x01,
	TK_SS_ON_AND_SET= 0x02
};

enum TK_BC_POWER_MODE
{
	TK_BC_PM_OFF =0x00, //
	TK_BC_PM_ON = 0x10,// Turns on receiver
	TK_BC_PM_TRANCIEVER_ON= 0x01, //
	TK_BC_PM_IR_LED_ON =0x02, //
	TK_BC_PM_IMU_ON =0x04, // report/no report IMU data
	TK_BC_PM_DISPLAY_LED_ON= 0x08, //
	TK_BC_PM_FULL_POWER= 0x1F // Enable radio, IMU,and pulsing
};

enum BC_SYNC_STATUS
{
	BC_SS_NORMAL= 0x00,
	BC_SS_BUTTON1 =0x01,
	BC_SS_BUTTON2 =0x02,
	BC_SS_LOW_POWER= 0x04,
	BC_SS_MED_POWER =0x08
};

enum TK_BC_COMMAND
{
	TK_BC_C_NONE =0x00,
	TK_BC_C_FLASH_LEDS= 0x01
};
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/


typedef struct {
	uint32_t packetID; /// Incrementing number
	uint32_t contentType; // IMU = 0x00001001
	uint32_t contentLength; // Entire packet length including contentType and contentLength
	uint16_t timekeeperID; // Last 16-bits of MAC Address Unique ID
}udp_hddr_t;

//typedef union {
//	uint8_t Ether_pkt[1500];
//	uint16_t Ether_pkt16[750];
//	uint32_t Ether_pkt_32[375];
//} hddr_union_t;

struct Ether_status{       // local storage of various ethernet link parameters
	uint8_t Link_Up;      // shows the Ether link is up
	uint8_t Probe_cnt;    // number of probe packets sent
	uint8_t BT_Link;      // found BT server
};

struct GenericSendHddr{
	uint8_t  packetID; // Incrementing number
	uint32_t contentType; // 0x00001001
	uint32_t contentLength; // Entire packet length including contentType and contentLength
	uint16_t timekeeperID; // Last 16-bits of MAC Address Unique ID
};


// Used for BC and TK serial numbers
// Unique to each ARM processor
struct DeviceSerialNumber {
	uint32_t a;
	uint32_t b;
	uint32_t c;
};

enum AuthenticationKeyFlags {
	AKF_NOT_SET =		0x0000,
	AKF_SET =		0x0001,

	AKF_WRITE_FAIL =	0x0100,
	ACK_PREVIOUSLY_SET = 0x0200
};

// Used for BC and TK serial numbers
// Unique to each ARM processor
struct AuthenticationKey {
	uint32_t a;
	uint32_t b;
	uint32_t c;
	uint32_t d;
};

typedef uint16_t TimekeeperID;

// Contains all other packets
struct TKFrame
{
    uint32_t signature;     // CAST TimeKeeper Packet Signature = TK_FRAME_SIGNATURE
    uint8_t version_major;  // Version information
    uint8_t version_minor;
    uint8_t version_revision;
    uint8_t version_build;
	TimekeeperID timekeeperID;  // Last 16-bits of MAC Address Unique ID
	uint32_t contentLength;
	uint32_t magicNumber;       // random number / session id
    uint32_t lostRequests;   // number of lost packet from Bridge
};

// for Bridge only
#ifdef _MSC_BUILD
// helpers
TKFrame& InitTKFrame(TKFrame& frame);
#endif

// TODO: Rename to GenericPacketHeader
struct GenericPHeader
{
	uint32_t packetID; /// Incrementing number
	uint8_t contentType;  // TK_PACKET_TYPES
	uint16_t contentLength; // Entire packet length including contentType and contentLength

};

struct GenericUSBPacketHeader
{
	uint16_t packetID; /// Incrementing number
	uint8_t contentType;  // TK_PACKET_TYPES
	uint8_t contentLength; // Entire packet length including contentType and contentLength
};

struct PassThroughHddr {
  struct GenericPHeader header;
  uint8_t       channel; // 9-21
  uint16_t      destAddr;
  uint8_t       payloadSize;
  uint8_t       payload[1];
};

struct PassThroughHddrStatus
{
	struct GenericPHeader header;
	uint32_t sourcePacketID; // packetID of the packet that is being ACK
        uint8_t   result;
};

// ----------------------------------------
// 2.2 Discovery probe
// ----------------------------------------
// TKPacketDiscovery is sent from TK to PC
struct TKPacketDiscovery
{
	struct GenericPHeader header;
};

// TKPacketDiscoveryACK is sent from PC to TK upon receiving of TKPacketDiscovery
struct TKPacketDiscoveryACK
{
	struct GenericPHeader header;
	uint32_t sourcePacketID; // packetID of the packet that is being ACK
};

// ----------------------------------------
// 2.3 Status
// ----------------------------------------
// TKPacketStatusSummaryRequest is sent from PC to TK
struct TKPacketStatusSummaryRequest
{
	struct GenericPHeader header;
};

// TKPacketStatusSummary is sent from TK to PC upon receiving TKPacketStatusSummaryRequest
struct TKPacketStatusSummary
{
	struct GenericPHeader header;
	uint32_t sourcePacketID; // packetID of the packet that is being ACK
	uint32_t numberActiveBeacons; //
	uint8_t ipaddress[4]; //
};

// TKPacketStatusDetailRequest is sent from PC to TK
struct TKPacketStatusDetailRequest
{
	struct GenericPHeader header;
};

// TKBeaconStatusDetail is sent from TK to PC upon receiving TKPacketStatusDetailRequest
// TKBeaconStatusDetail is followed by (numberBeacons x TKBeaconStatusDetail)
// The pointer 'beacons' is set to program memory when allocated, and set to an offset
// from the address of the 'beacons' pointer - see onRecv and onSend
struct TKPacketStatusDetail
{
	struct GenericPHeader header;
	uint32_t sourcePacketID; // packetID of the packet that is being ACK
	uint32_t numberBeacons;		// [read-only after memory allocation]
	uint8_t ipaddress[4];			//
	struct TKBeaconStatusDetail * beacons; // Pointer to arrays of structures
};
struct TKBeaconStatusDetail
{
	uint16_t beaconID; // is now identical to mySrcAddr
	uint16_t mySrcAddr; // BC specified Address
	uint16_t beaconTimeLastActivity; // Last sync tick of last received packet
	uint8_t beaconStatus; // size of dynamic area is equal to numberConnectedBeacons
	uint8_t beaconBattery; //  See below
	uint8_t beaconSignalTK; // RSSI of last packet Rx'd by TK
	uint8_t beaconSignalBC; // RSSI of last packet Rx'd by BC
	uint8_t radioStatus; // TK_RADIO_STATUS
	uint8_t syncStatus; // TK_SY N
	uint8_t RxErrors;   // rx errors since last status poll....
    uint8_t irLed0;
    uint8_t irLed1;
    uint8_t irLed2;
};

#ifdef _MSC_BUILD       // Birdge using only
extern void convertForUsbError(struct TKBeaconStatusDetail* detail);
extern bool USING_USB_CAMERA_SYSTEM;
#endif

/* Battery Levels
*Data for V#3 hardware.. 4.2v= E1, 4.0v=D7, 3.9v=D0, 3.8v=CA, 3.7v=C6, 3.6v=C1
*                         3.5v=BB, 3.4v=B6, 3.3v=B0,3.2v=AB, 3.1v= A5, 3.0v= A0
*                         2.9v= 9A, 2.8v= 96, 2.7v= 90, 2.6v= 8B
*/


// ----------------------------------------
//2.4 Commands
// ---------------------------------------

// TKPacketRadioControl sent from PC to TK
// Packet is sent upon receiving TKPacketDiscovery
// Packet can also be sent at any other time during the show
struct TKPacketRadioControl
{
	struct GenericPHeader header;
	uint8_t controlCommand; // TK_RADIO_STATUS
};

// TKPacketRadioControlACK sent from TK to PC upon receiving TKPacketRadioControl
struct TKPacketRadioControlACK
{
	struct GenericPHeader header;
	uint32_t sourcePacketID; // packetID of the packet that is being ACK
};

// TKPacketSyncControl sent from PC to TK
// Packet is sent upon receiving TKPacketDiscovery
// Packet can also be sent at any other time during the show
struct TKPacketSyncControl
{
	struct GenericPHeader header;
	uint8_t controlCommand; // TK_SY NC_STATUS
	uint32_t controlParam; // For TK_SS_ON_AND_SET, set sync packet number for the current frame
};

// TKPacketSyncControlACK sent from TK to PC upon receiving TKPacketSyncControl
struct TKPacketSyncControlACK
{
	struct GenericPHeader header;
	uint32_t sourcePacketID; // packetID of the packet that is being ACK
};

// ----------------------------------------
// 2.5 Parameters
// ----------------------------------------

struct TKEEPROMData
{
	uint32_t version; //
	uint8_t macAddress[6]; //
	uint8_t usbAddress[6]; // VendorID, PID, SerialNumber
	uint8_t defaultIPAddress[4];// Format should be something like 10.133.0.90,[0] = 10, [1] = 133, ...
	uint8_t defaultMulticastServerIPAddress[4]; // Format should be something like 10.133.0.90, [0] = 10, [1] = 133, ...
	uint8_t defaultMulticastServerPort[2]; // Default - 2300
	uint8_t defaultMulticastTKIPAddress[4]; // Format should be something like 10.133.0.90, [0] = 10, [1] = 133, ...
	uint8_t defaultMulticastTKPort[2]; // Default - 2301
};
struct TKPacketSetEEPROM
{
	struct GenericPHeader header;
	uint32_t epromDatalength; // sizeof TKEEPROMData
	//	uint8_t * epromData; // TKEEPROMData
};
struct TKPacketReadEEPROM
{
	struct GenericPHeader header;
};


struct TKPacketReadEEPROMResponse
{
	struct GenericPHeader header;
	uint32_t epromDatalength; // sizeof TKEEPROMData
	//	uint8_t epromData; // TKEEPROMData
};

// ----------------------------------------
// 2.6 Beacon
// ----------------------------------------

struct BCIMUTimeStamp
{
        uint8_t   SyncFrameIMU; // LSB of 32 bit sync frame number
        uint8_t   MsTimerIMU; // LSB of 500us counter tied to sync frame # above
        uint8_t   IMUPktNum;  // Packet number of IMU sample at sync frame above
};

struct BCIMUData
{
        uint8_t  Timestamp;   // imu packet number 0-255
        uint16_t gyroscopeX;
	uint16_t gyroscopeY;
	uint16_t gyroscopeZ;
	uint16_t accelerationX;
	uint16_t accelerationY;
	uint16_t accelerationZ;

};

// structure of array of IMU data from Beacon
struct BCIMUDSTRCT
{
	struct BCIMUData ImuData[NUM_OF_IMU_PKTS_IN_RF_PKT]; //
};

struct TKEtherIMUSend
{
        struct BCIMUTimeStamp IMU_TimeStamp;
	struct BCIMUData imuData[NUM_OF_IMU_PKTS_IN_RF_PKT];
};

// structure of header  of ether packet of IMU data to BT server
struct TKBCPacketIMU
{
	struct GenericPHeader header;         // len 9
	uint16_t beaconID;
	uint8_t sequenceNumber; //
    uint32_t ArrivalTime; // time of this packet arrival from Beacon

};

// Total structure of data Ether packet from TK to BT server

struct TKBCIMUEthPacket
{
	struct TKBCPacketIMU TKBCIMUHeader;
         struct BCIMUTimeStamp TimeStamp;
	struct BCIMUDSTRCT BC_IMUData;
};

// TKBCPacketButton is sent from TK to PC whenever user presses button on beacon
struct TKBCPacketButton
{
	struct GenericPHeader header;
	uint16_t beaconID;
	uint8_t buttonState; //
};

struct TKBCPacketRadioFrequency
{
	struct GenericPHeader header;
	uint16_t beaconID; // TK specified beacon ID, starting at 0
	uint8_t signalStrength; //
};

struct TKBCPacketStatus
{
	struct GenericPHeader header;
	uint16_t beaconID;
	uint16_t beaconNumber; // Indexed as 0, 1, 2, etc based on LED pulsing signature
	uint8_t currentStatus; //
};

// ----------------------------------------
// 2.6.1 Beacon commands
// ----------------------------------------
struct TKBCPacketSetBeaconNumber
{
	struct GenericPHeader header;
	uint16_t beaconID;
	uint16_t beaconNumber; // Indexed as 0, 1, 2, etc based on LED pulsing signature
};
struct TKBCPacketSetBeaconNumberACK
{
	struct GenericPHeader header;
	uint32_t sourcePacketID; // packetID of the packet that is being ACK
};

// TKBCPacketFlashLED is sent from PC to TK to instruct the beacon to flash its LEDs
struct TKBCPacketFlashLED
{
	struct GenericPHeader header;
	uint8_t beaconID; // TK specified beacon ID, starting at 0
};

// TKBCPacketFlashLEDACK is sent from TK to PC upon receiving TKBCPacketFlashLED
struct TKBCPacketFlashLEDACK
{
	struct GenericPHeader header;
	uint32_t sourcePacketID; // packetID of the packet that is being ACK
};

// TKBCPacketPowerMode is sent from PC to TK to change the power mode of the beacon
struct TKBCPacketPowerMode
{
	struct GenericPHeader header;
	uint8_t beaconID; // TK specified beacon ID, starting at 0
	uint8_t powerMode; // TK_BC_POWER_MODE
};

// TKBCPacketPowerModeACK is sent from TK to PC upon receiving TKBCPacketPowerMode
struct TKBCPacketPowerModeACK
{
	struct GenericPHeader header;
	uint32_t sourcePacketID; // packetID of the packet that is being ACK
};

// New TK packet type - a "forward any message from beacon"
struct TKBCPacketForwardAny
{
    struct GenericPHeader header;
    uint16_t    msgType;        //"magic" 'FABC'
    uint16_t    routerId;
    uint16_t    destAddr;
    uint16_t    srcAddr;
    uint8_t     seqNumber;
    uint8_t     payloadSize;
    uint8_t     payload[128];

};

// ----------------------------------------
// Chapter 3 - BC Radio
// ----------------------------------------
// 118 usable bytes per radio packet and per sync.
// Each packet has 10 bytes + payload

// ----------------------------------------
// 3.1 current implementation
// ----------------------------------------


// TODO: Sandy - add battery state to this packet
// TODO: More meaninful name for this packet
struct BCPacketIMU
{
        uint8_t seqNum; // sequence number from Beacon
	uint8_t buttonState; // button state
	uint8_t signalStrength; // Signal strength as recieved from TK at Beacon
	uint8_t led1State; // IRLed 0 code
	uint8_t led2State; // IRLed 1 code
	uint8_t led3State; // IRLed 2 code
        uint8_t   Battery_lev; //Battery level
        uint8_t   SyncFrameIMU; // LSB of 32 bit sync frame number
        uint8_t   MsTimerIMU; // LSB of 500us counter tied to sync frame # above
         uint8_t   IMUPktNum;  // Packet number of IMU sample at sync frame above

	struct BCIMUData imuData[NUM_OF_IMU_PKTS_IN_RF_PKT]; //
};

//struct TKETHIMUTstamp{
//        uint8_t   SyncFrameIMU;
//        uint8_t   MsTimerIMU;
//};




//3.2 Phase II
//There are an extra 2 bits of flags in each uint16_t.

struct BCPacket
{
	uint8_t status; // BC_STATUS
	uint8_t signalStrength; //
        uint16_t gyroscopeX; // Average value since last update
	uint16_t gyroscopeY; // Average value since last update
	uint16_t gyroscopeZ; // Average value since last update
	uint16_t accelerationX; // Average value since last update
	uint16_t accelerationY; // Average value since last update
	uint16_t accelerationZ; // Average value since last update

};
struct BCPacketSetting
{
	uint8_t numberBeacons; //
	uint16_t buttonID; // dynamic array of size numberBeacons
	uint8_t powerMode; // TK_BC_POWER_MODE dynamic array of size numberBeacons
	uint8_t command; // TK_BC_COMMAND dynamic array of size number Beacons
};

// Data returned from beacon
// as stored in structure in TK memory for each active beacon
//{seq#, button_state, beaconRSSI, led0Id, led1Id, led2Id}
struct BeaconData{
	uint8_t   NewBKData ;
	uint8_t   BC_PowerMode;
	uint8_t   LastButtonState;
	uint32_t  arrivalTime;  // TK Master sync counter
	uint8_t   TK_RxRSSI;  // RSSI of this recieved packet
	uint8_t   seqNum; // sequence number from Beacon
	uint8_t   button_state;
	uint8_t   beaconRSSI;  // RSSI signal strength of TK as recieved at beacon ..
	uint8_t   led0Id;
	uint8_t   led1Id;
	uint8_t   led2Id;
    uint8_t   beaconBattery;
    uint8_t   SyncFrameIMU;   // sync frame of following IMU samples
    uint8_t   MsTimerIMU;   // time ticks since start of IMU sample below
    uint8_t   IMUPktNum;  // Packet number of IMU sample at sync frame above
	struct BCIMUData ImuData[NUM_OF_IMU_PKTS_IN_RF_PKT]; //
};

// ------------------------------------------
// Beacon USB Packets
// ------------------------------------------

// USBBCDetailRequest is sent from PC to BC
struct USBBCDetailRequest {
	struct GenericUSBPacketHeader header;
};

// USBBCDetail is sent from BC to PC upon receiving USBBCDetailRequest
struct USBBCDetail {
	struct GenericUSBPacketHeader header;
	uint16_t sourcePacketID;	// packetID of the packet that is being ACK

	// Unique serial number of Beacon - processor ID
	struct DeviceSerialNumber serialNumber;

	uint16_t authKeyFlags;	// AuthenticationKeyFlags
	uint16_t panId;			// Radio Frequency Pan Group - 0XFFFF is broadcast
	uint16_t mySrcAddr;		// Radio Frequency ID of Beacon - unique for the active system
	uint16_t tkDstAddr;		// Radio Frequency ID of TimeKeeper - unique for the active system
	uint16_t ledOnOffs;		// Sync time marker - start flashing
	uint16_t ledOffOffs;		// Sync time marker - stop flashing
	uint8_t rfChan;			// RF Channel for TK communication
	uint8_t led0Id;			// IR Flash code for LED
	uint8_t led1Id;			// IR Flash code for LED
	uint8_t led2Id;			// IR Flash code for LED
	uint8_t beaconBattery;	//  empty 0, full at 255
	uint8_t beaconSignalTK;	// RSSI of last packet Rx'd by TK
	uint8_t beaconSignalBC;	// RSSI of last packet Rx'd by BC
	uint8_t radioStatus;		// TK_RADIO_STATUS
	uint8_t syncStatus;		// TK_SY N
	uint8_t RxErrors;   // rx errors since last status poll....
};

// USBBCDetailRequest is sent from PC to BC
struct USBBCRadioDetailRequest {
	struct GenericUSBPacketHeader header;
};

// USBBCDetail is sent from BC to PC upon receiving USBBCDetailRequest
struct USBBCRadioDetail {
	struct GenericUSBPacketHeader header;
	uint16_t sourcePacketID;	// packetID of the packet that is being ACK

	uint8_t beaconSignalTK;	// RSSI of last packet Rx'd by TK
	uint8_t beaconSignalBC;	// RSSI of last packet Rx'd by BC
	uint8_t radioStatus;		// TK_RADIO_STATUS
	uint8_t syncStatus;		// TK_SY N
	uint8_t RxErrors;   // rx errors since last status poll....
};

// USBBCSetAuthKey sent from PC to BC
// Should only be sent once
// authKeyFlags should be set once set
struct USBBCSetAuthKey {
	struct GenericUSBPacketHeader header;
	struct AuthenticationKey authKey;
};

// USBBCSetAuthKeyACK sent from BC to PC upon receiving USBBCSetAuthKey
struct USBBCSetAuthKeyACK {
	struct GenericUSBPacketHeader header;
	uint16_t sourcePacketID;	// packetID of the packet that is being ACK
	uint16_t authKeyFlags;	// AuthenticationKeyFlags
};


// USBBCFirmwareIOSetup sent from PC to BC
struct USBBCFirmwareIOSetup {
	struct GenericUSBPacketHeader header;
	uint32_t firmwareSize;	// Total size in bytes of the firmware
	uint16_t CRC_val;				// Checksum for entire firmware
};

// USBBCFirmwareIOSetupACK sent from BC to PC upon receiving USBBCFirmwareIOSetup
struct USBBCFirmwareIOSetupACK {
	struct GenericUSBPacketHeader header;
	uint16_t sourcePacketID;	// packetID of the packet that is being ACK
};

// USBBCSetFirmwareMemoryBlock sent from PC to BC
// USBBCSetFirmwareMemoryBlock should be called after receiving USBBCFirmwareIOSetupACK

struct USBBCSetFirmwareMemoryBlock {
	struct GenericUSBPacketHeader header;
	uint16_t blockNumber;	// memory block ID
      uint8_t data[FIRMWARE_BLOCK_LENGTH];		// Data contained in the memory block
};

// USBBCSetFirmwareMemoryBlockACK sent from BC to PC upon receiving USBBCSetFirmwareMemoryBlock
struct USBBCSetFirmwareMemoryBlockACK {
	struct GenericUSBPacketHeader header;
	uint16_t sourcePacketID;	// packetID of the packet that is being ACK
};

// USBBCSetFirmwareMemoryBlockResend sent from BC to PC if block is needed due to invalid CRC
// Block CRCs are only evaluated if the global CRC fails
struct USBBCSetFirmwareMemoryBlockRequest {
	struct GenericUSBPacketHeader header;
	uint16_t blockNumber;
};



//*************************************************************************************************************************
//*************************************************************************************************************************
//*************************************************************************************************************************

//=================================================================================================
// Packet from Controlling application to device
//=================================================================================================

// 1. Firmware Update command lines string. Sending from Controlling application to firmware
// For firmware unpdating, it uses 'U' command
// U <Base64 encoded packet>

// 2. packet layer
struct PacketHeader {
    uint8_t type;
    uint16_t size;        // including header and buffer
};

// Packet content (buffer) follows PacketHeader

// TODO!!! turn to static const
#define DEV_CMD_UPD_REQ				1		// to device
#define DEVCMD_UPD_PACKET			2		// firmware update packet (To device)
#define DEV_CMD_FIRMWARE_VERIFY		3		// verify firmware request
#define DEV_CMD_REBOOT				4		// Reboot
#define DEV_CMD_CONFIG_REQ			5		// Get configuration request
#define DEV_CMD_RUNNING_STATUS_REQ	6		// Get Running status request
#define DEV_CMD_SET_CONFIG			7		// Change entire configuration. body is packed config_t structure
#define DEV_CMD_GET_VERSION			8		// Get firmware version
#define DEV_CMD_BAT_STATUS                      9

#define DEV_CMD_GET_PROD_AREA           0x10
#define DEV_CMD_SET_PROD_AREA           0x11

#define DEV_CMD_GET_EEPROM_DATA        0x12
#define DEV_CMD_SET_EEPROM_DATA        0x13

#define DEV_CMD_SEND_BEACON_DATA        0x14


// for DEVCMD_UPD_PACKET only
struct FirmwarePacketHeader{
    uint16_t count;                // number of fragments
    uint16_t index;                // current fragment index
    // follow by buffer;          // firmware binary. Size is determined by packet size - FirmwarePacket header == packet size - 4
};

// for DEV_CMD_FIRMWARE_VERIFY only
struct FirmwareVerifyPacketHeader{
    uint32_t address;              // Firmware start address
    uint32_t count;                // Number of bytes to verify
};

// for DEV_CMD_GET_PROD_AREA only
struct GetProdAreaPacketHeader{
    uint16_t index;                // index of area, starts from 0, each one is 128 bytes
};

typedef struct sDataSend{
  uint8_t channel;
  uint8_t payloadSize;
  uint16_t destAddr;
  uint8_t payload [128];

} tDataSend;

// =================================================================================================
// Packet from device to Controlling appication, using binary protocol
//
// Simple packet for USB Debugging
//=================================================================================================
struct USBDebugInfoHeader {
    uint32_t signature;         // CAST TimeKeeper Packet Signature = TK_FRAME_SIGNATURE
    uint8_t version;            // Version information
    uint32_t checkSum;
	uint32_t magicNumber;       // random number / session id
	uint32_t packetID;          // Incrementing number
	uint8_t contentType;        // TK_PACKET_TYPES
	uint16_t contentLength;     // Entire packet length including Header
};

// content type
#define DEV_RESP_INFO			0				// Display info. Compatible with previous version. Show on console directly
#define DEV_RESP_IMU_DATA		1				// IMU data, binary format.
#define DEV_RESP_UPD			2				// Response from firmware update
#define DEV_RESP_CONFIG			3				// Firmware configuration. Binary content of config_t. Beacon, T.K. & Router use different configuration structures
#define DEV_RESP_RUNNING_STATUS	4				// Device status. Beacon, T.K. & Router can define their own structures
#define DEV_RESP_FIRMWARE_SUM   5				// reply firmware signature / checkesum
#define DEV_RESP_WHOAMI			6				// Inquire device type and ID
#define DEV_RESP_VERSION		7				// version information

#define DEV_RESP_BAT_STATUS             9

#define DEV_RESP_GET_PROD_AREA          0x10
#define DEV_RESP_SET_PROD_AREA          0x11

#define DEV_RESP_GET_EEPROM_DATA        0x12
#define DEV_RESP_SET_EEPROM_DATA        0x13

#define DEV_RESP_SEND_BEACON_DATA        0x14

#define DEV_RESP_HARDWARE_VERSION        0x20

//DEV_RESP_GET_EEPROM_DATA
struct RespEepromData {
  uint16_t  errorCode;
  uint8_t   data[32];
};

//DEV_RESP_GET_PROD_AREA
struct RespProdArea {
  uint16_t   errorCode;
  uint16_t   index;
  uint8_t    data[128];
};

// DEV_RESP_UPD
// Response from firmware update
// Follows USBDebugInfoHeader
struct RespUpdate {
    uint16_t errorCode;             // error code.  0 successful
    uint16_t index;                 // The segment update just finished
};

// Response of DEV_CMD_RUNNING_STATUS_REQ
// Follows USBDebugInfoHeader
struct BeaconRunningStatus {
    uint16_t errorCode;             // error code.  0 successful
    uint16_t index;                 // Battery, percentage
	uint8_t radioOnOff;				// 0 off, 1 on
};

struct TimekeeperRunningStatus {
    uint16_t errorCode;             // error code.  0 successful
	uint8_t radioOnOff;				// 0 off, 1 on
									// Maybe extend number of send / receive ... in the near future
};

struct RouterRunningStatus {
    uint16_t errorCode;             // error code.  0 successful
	uint8_t radioOnOff;				// 0 off, 1 on
									// Maybe extend number of send / receive ... in the near future
};

// DEV_RESP_FIRMWARE_SUM
// Response of DEV_CMD_FIRMWARE_VERIFY
// Follows USBDebugInfoHeader
struct RespFirmwareSum {
    uint16_t errorCode;             // Error code.  0 successful
    uint8_t sum[16];                 // For storing MD5 checksum
};

// body of DEV_RESP_WHOAMI
struct WhoAmI {
	uint8_t type;            // 1: T.K., 2: Router, 3: beacon
	uint8_t module;          // 1: boot loader; 2: main firmware
	uint8_t id[12];         // unique Id of device
};

//#define DEV_RESP_VERSION
struct RespFirmwareVersion {
	uint8_t major;
	uint8_t minor;
	uint8_t patch;
	uint8_t reserved;
        uint8_t dateString[32];		// building date C-string /0 terminated
        uint32_t revision;	        // SVN revision number
        uint8_t padding[28];
	uint8_t timeString[32];		// building time C-string /0 terminated
        uint8_t reserved1[32];		        
};

//DEV_RESP_SEND_BEACON_DATA

#define SEND_BEACON_DATA_OK             0
#define SEND_BEACON_DATA_BUSY           1
//#define SEND_BEACON_DATA_CORRUPT        2

struct RespSendBeaconData {
        uint8_t status;
};

#define DEV_RESP_HARDWARE_VERSION        0x20
struct RespHardwareVersion {
        uint16_t version;
};

//radioPacketFlags in the following structure has 3 flags to send
// 0x01 - regilar IMU data
// 0x02 - button press data (sent on button press)
// 0x04 - periodic, ~ 1minute battery status data
#define RADIOPACKET_IMU         0x01
#define RADIOPACKET_BUTTONPRESS 0x02
#define RADIOPACKET_BATTERY     0x04

#define BC_PRODUCT_ID   0xBC10
#define TK_PRODUCT_ID   0xCD10
#define RT_PRODUCT_ID   0xDE10

#ifdef _MSC_BUILD
namespace beacon {


struct Config {
  uint16_t      size;
  uint16_t      checksum;
//  from config_t without checksum and u32IwdgResetEvents;
  uint16_t    productID;
  uint16_t    serialNum;
  uint16_t    panId;
  uint16_t    mySrcAddr;
  uint16_t    tkDstAddr;
  uint16_t    ledOnOffs;
  uint16_t    ledOffOffs;
  uint16_t    ledDAC;
  uint8_t     rfChan;
  uint8_t     rfTimeSlot;
  uint8_t     led0Id;
  uint8_t     led1Id;
  uint8_t     led2Id;
  uint8_t     TestMode;
  uint8_t     TxLevel;
  uint8_t     radioPacketFlags;

  uint32_t      led0IdPattern;
  uint32_t      led1IdPattern;
  uint32_t      led2IdPattern;
  uint32_t      led0Index;
  uint32_t      led1Index;
  uint32_t      led2Index;
  uint8_t       frameBits;

	Config() {
		productID      = BC_PRODUCT_ID; //0xBC10;
		serialNum      = 0x444;
		panId          = 0x0001;
		mySrcAddr      = 0x5678;
		tkDstAddr      = 0xABCD;
		ledOnOffs      = 52000;
		ledOffOffs     = 4000;
		ledDAC         = 3840;
		rfChan         = 15;
		rfTimeSlot     = 2;
		led0Id         = 0xFE;
		led1Id         = 0xFD;
		led2Id         = 0xFB;
		TestMode       = 0x00;
		TxLevel        = 8;
		radioPacketFlags          = 0xFF;
                led0IdPattern   = 0xFFFFFFFF;
                led1IdPattern   = 0xFFFFFFFE;
                led2IdPattern   = 0xFFFFFFFD;
                led0Index       = 1;
                led1Index       = 2;
                led2Index       = 3;
                frameBits       = 8;
	}

};


typedef Config config_t;
};	// of beacon namespace


namespace timekeeper {
struct Config {
    uint16_t    productID;
    uint16_t    serialNum;
    uint16_t    panId;
    uint16_t    mySrcAddr;
    uint8_t     rfChan;
    uint8_t     TxPower;
    uint8_t     TestMode;
    uint8_t     SyncOutEn;
    uint32_t    u32IwdgResetEvents;
    //uint16_t     Dummy1;

    /* !! ABOVE MUST HAVE EVEN # BYTES!! */
    uint16_t    checksum;   // MUST BE LAST!!

	Config() {
		productID      = TK_PRODUCT_ID; //0xCD10;
		serialNum      = 0x555;
		panId          = 0x0001;
		mySrcAddr      = 0xABCD;
		rfChan         = 15;
		TxPower        = 8;   // index of tx power
		TestMode       = 0x00;
		SyncOutEn      = 0x01;
		//.Dummy          = 0;
		u32IwdgResetEvents = 0;
	}
};
typedef Config config_t;
};      // of Timekeeper namespace


#endif

#define PACKET_VERSION          3
#define NUM_IMU_PACKETS         5

//#if defined (__ICCARM__)

/** Packet types **/
#define SET_LED_ON                      0x01
#define SET_LED_OFF                     0x0F
#define GET_LED_STATUS                  0x04
#define SET_LED_PULSE_POSITION          0x02
#define SET_LED_PULSE_WIDTH             0x03
#define WRITE_CONFIG                    0x1C
#define GET_CONFIG                      0x2C

#define PKT_TYPE_BATTERY                0xBA
#define PKT_TYPE_LED_STATUS             0x1E

/** ---- **/
struct BeaconDataHeader {
    uint8_t     crc8;
    uint8_t     type;         /** Packet types **/  //0xBA - battery, 0xAD orientation
    uint8_t     version;
};

struct BeaconDataHeaderAsync {
  struct        BeaconDataHeader     hdr;
  uint8_t       tick;
  uint8_t     led0Id;
  uint8_t     led1Id;
  uint8_t     led2Id;
};


struct BeaconDataLedOnOff {
    struct BeaconDataHeader     hdr; //type 0x1E
    uint16_t                    time;
};

struct BeaconDataLedStatus {
    struct BeaconDataHeaderAsync    ahdr;
    uint8_t     status;         //0 - off, 1 - on
};

struct Beacon_BatData
{
    uint8_t     crc8;
    uint8_t     type;           //0xBA - battery
    uint8_t     version;
    uint8_t     tick;
    uint8_t     flags;          //0x01 - charging
    uint16_t    minToRun;       //estimated minutes left to run at current level
                                //0xFFFF means unknown;
    uint16_t    voltiCents;     //current voltage in 0.01 V
   // uint16_t    lowestVoltiCents; //(0.00V is unknown) - minimum Voltage riched
                                 // after last charge - reported only when not charging
    uint8_t     percents;       //battery level
#if PACKET_VERSION > 3
    uint8_t     led0Id;
    uint8_t     led1Id;
    uint8_t     led2Id;
#endif
#ifdef CALIBRATION_TEST
    uint16_t    tim5_phase;
    uint16_t    tim1AtFrameId;
#endif

};

#define BAT_FLAG_CHARGING       0x01
#define BAT_FLAG_USB_POWER      0x02

struct quant
{
    float       q0;
    float       q1;
    float       q2;
    float       q3;

};

struct Madgwick_pkt
{
    uint8_t     crc8;
    uint8_t     type;           //0xAD
    uint8_t     version;
    uint8_t     bat_percents;
    uint8_t     button_state;
    uint8_t     IRLed0;
    uint8_t     IRLed1;
    uint8_t     IRLed2;
    uint32_t    Seq_Num;
    struct      quant quant[NUM_IMU_PACKETS];
};

//#endif

#define FRAME_CLOCK_100 100
#define FRAME_CLOCK_120 120
#define FRAME_CLOCK_180 180
#define FRAME_CLOCK_240 240

#if defined (__ICCARM__)
struct BeaconOldStruct
{
    uint32_t    frameId;
    uint16_t    magic;  //'BT'
    uint8_t     tick;
    uint8_t     frameClock;
    uint8_t     crc8;            // To make it comaptible with legacy format
};

struct BeaconNewStruct
{
    uint32_t    frameId;
    uint16_t    magic;  //'BT'
    uint8_t     tick;
    uint8_t     frameClock;
    uint8_t     crc8;            // To make it comaptible with legacy format
    uint32_t    session;
    uint16_t    offsetDelta;    // To make classic beacon compatible with a new type of T.K. it is supposed to be a contant for a specific type of T.K.
};
#endif

#define BUTTON_PRESS            0x01
#define BUTTON_RELEASE          0x08
#define BUTTON_CLICK            0x02
#define BUTTON_DBLCLICK         0x04
#define BUTTON_A                0x0F
#define BUTTON_B                0xF0

struct ButtonClick
{
    uint8_t     crc8;
    uint8_t     type;           //0xBC
    uint8_t     version;
    uint8_t     tick;           //To improve reliability, same message sent twice
                                //with the same tick. Next message increments tick
    uint8_t     button_events;  //click, double click, press, release
#if PACKET_VERSION > 3
    uint8_t     led0Id;
    uint8_t     led1Id;
    uint8_t     led2Id;
#endif
};

/* ----- */ //The following are REVJ packet structures
enum BC_MSG_TYPES {
	BC_IMU_PacketFrame 				= 0x11,
	BC_BTN_PacketFrame 				= 0x12,
	BC_BTN_IMU_PacketFrame 			= 0x23,
	BC_BAT_PacketFrame 				= 0x14,
	BC_BAT_IMU_PacketFrame 			= 0x25,
	BC_BAT_BTN_PacketFrame 			= 0x26,
	BC_BAT_BTN_IMU_PacketFrame 		= 0x37,
	BC_LED_PacketFrame 				= 0x18,
	BC_LED_IMU_PacketFrame 			= 0x29,
	BC_LED_BTN_PacketFrame 			= 0x2A,
	BC_LED_BTN_IMU_PacketFrame 		= 0x3B,
	BC_LED_BAT_PacketFrame 			= 0x2C,
	BC_LED_BAT_IMU_PacketFrame 		= 0x3D,
	BC_LED_BAT_BTN_PacketFrame 		= 0x3E,
	BC_LED_BAT_BTN_IMU_PacketFrame 	= 0x4F
};
#if 0
struct BeaconData{
	uint8_t   NewBKData ;
	uint8_t   BC_PowerMode;
	uint8_t   LastButtonState;
	uint32_t  arrivalTime;  // TK Master sync counter
	uint8_t   TK_RxRSSI;  // RSSI of this recieved packet
	uint8_t   seqNum; // sequence number from Beacon
	uint8_t   button_state;
	uint8_t   beaconRSSI;  // RSSI signal strength of TK as recieved at beacon ..
	uint8_t   led0Id;
	uint8_t   led1Id;
	uint8_t   led2Id;
    uint8_t   beaconBattery;
    uint8_t   SyncFrameIMU;   // sync frame of following IMU samples
    uint8_t   MsTimerIMU;   // time ticks since start of IMU sample below
    uint8_t   IMUPktNum;  // Packet number of IMU sample at sync frame above
	struct BCIMUData ImuData[NUM_OF_IMU_PKTS_IN_RF_PKT]; //
};
#endif
#pragma pack(pop)


#endif  // BEACON_COMMON_PACKETS_H
