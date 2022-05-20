#ifndef _BEACON_FRAME_H 
#define _BEACON_FRAME_H

#ifdef _MSC_BUILD

// Typedefs for specific-width integers.
typedef unsigned __int32 uint32_t;
typedef unsigned __int16 uint16_t;
typedef unsigned __int8  uint8_t;

typedef __int32  int32_t;
typedef __int16  int16_t;
#if (_MSC_VER < 1600)
typedef __int8   int8_t;
#endif

#endif

#pragma pack(push, 1)
    #define BC_NUM_OF_IMU_PKTS_IN_RF_PKT 5
#ifndef BC_HW_REVJ
    //Following used in the type variable within Beacon_Preamble struct to indicate type of msg.
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

    //Following to be used as masks for ledstat bit within BK_LEDData
    enum ledstat_bits{
        BC_LED0_STAT 				= 0x01,
	    BC_LED1_STAT 				= 0x02,
	    BC_LED2_STAT                = 0x04
    };
    
    struct Beacon_Preamble{
        uint8_t   Seq_Num;
        uint8_t   type;
        uint8_t   BeaconRSSI;
        uint8_t   SyncFrame;
        uint8_t   MsTimerIMU;
        uint8_t   IMUPktNum;
    };
    
    struct BK_IMUData{
        uint8_t  Timestamp;
  	    uint16_t gyroscopeX;
	    uint16_t gyroscopeY;
	    uint16_t gyroscopeZ;
	    uint16_t accelerationX;
	    uint16_t accelerationY;
	    uint16_t accelerationZ;
    };

    struct BK_BTNData{
    	uint8_t buttonA_events;
    	uint8_t buttonA_tick;
        uint8_t	buttonB_events;
    	uint8_t	buttonB_tick;    
    };

    struct BK_BATData{
    	uint8_t Battery_lev;
    };
    
    struct BK_LEDData{
	    uint8_t ledstat;
        uint8_t led_ticks;
    };

//unique 15 types of messages pertaining the above 4 main messages. 
    struct IMU_Data_pkt{
        struct Beacon_Preamble BK_Preamble;
        struct BK_IMUData BeaconIMUData[BC_NUM_OF_IMU_PKTS_IN_RF_PKT];
    };

    struct BTN_Data_pkt{
	    struct Beacon_Preamble BK_Preamble;
        struct BK_BTNData BeaconBTNData;	
    };
    struct BTN_IMU_Data_pkt{
	    struct Beacon_Preamble BK_Preamble;
        struct BK_BTNData BeaconBTNData;
        struct BK_IMUData BeaconIMUData[BC_NUM_OF_IMU_PKTS_IN_RF_PKT];
    };
    struct BAT_Data_pkt{
        struct Beacon_Preamble BK_Preamble;
        struct BK_BATData BeaconBATData;
    };
    struct BAT_IMU_Data_pkt{
	    struct Beacon_Preamble BK_Preamble;
        struct BK_BATData BeaconBATData;
        struct BK_IMUData BeaconIMUData[BC_NUM_OF_IMU_PKTS_IN_RF_PKT];
    };
    struct BAT_BTN_Data_pkt{
        struct Beacon_Preamble BK_Preamble;
        struct BK_BATData BeaconBATData;
        struct BK_BTNData BeaconBTNData;
    };
    struct BAT_BTN_IMU_Data_pkt{
        struct Beacon_Preamble BK_Preamble;
        struct BK_BATData BeaconBATData;
        struct BK_BTNData BeaconBTNData;
        struct BK_IMUData BeaconIMUData[BC_NUM_OF_IMU_PKTS_IN_RF_PKT];
    };
    struct LED_Data_pkt{
        struct Beacon_Preamble BK_Preamble;
        struct BK_LEDData BeaconLEDData;
    };
    struct LED_IMU_Data_pkt{
        struct Beacon_Preamble BK_Preamble;
        struct BK_LEDData BeaconLEDData;
        struct BK_IMUData BeaconIMUData[BC_NUM_OF_IMU_PKTS_IN_RF_PKT];
    };
    struct LED_BTN_Data_pkt{
        struct Beacon_Preamble BK_Preamble;
        struct BK_LEDData BeaconLEDData;
        struct BK_BTNData BeaconBTNData;
    };
    struct LED_BTN_IMU_Data_pkt{
        struct Beacon_Preamble BK_Preamble;
        struct BK_LEDData BeaconLEDData;
        struct BK_BTNData BeaconBTNData;
        struct BK_IMUData BeaconIMUData[BC_NUM_OF_IMU_PKTS_IN_RF_PKT];
    };
    struct LED_BAT_Data_pkt{
        struct Beacon_Preamble BK_Preamble;
        struct BK_LEDData BeaconLEDData;
        struct BK_BATData BeaconBATData;
    };
    struct LED_BAT_IMU_Data_pkt{
        struct Beacon_Preamble BK_Preamble;
        struct BK_LEDData BeaconLEDData;
        struct BK_BATData BeaconBATData;
        struct BK_IMUData BeaconIMUData[BC_NUM_OF_IMU_PKTS_IN_RF_PKT];
    };
    struct LED_BAT_BTN_Data_pkt{
        struct Beacon_Preamble BK_Preamble;
        struct BK_LEDData BeaconLEDData;
        struct BK_BATData BeaconBATData;
        struct BK_BTNData BeaconBTNData;
    };
    struct LED_BAT_BTN_IMU_Data_pkt{
        struct Beacon_Preamble BK_Preamble;
        struct BK_LEDData BeaconLEDData;
        struct BK_BATData BeaconBATData;
        struct BK_BTNData BeaconBTNData;
        struct BK_IMUData BeaconIMUData[BC_NUM_OF_IMU_PKTS_IN_RF_PKT];
    };

#else
    struct BK_IMUData {
        uint8_t  Timestamp;
  	    uint16_t gyroscopeX;
	    uint16_t gyroscopeY;
	    uint16_t gyroscopeZ;
	    uint16_t accelerationX;
	    uint16_t accelerationY;
	    uint16_t accelerationZ;
    };

    struct Beacon_Preamble {
         uint8_t   Seq_Num;
         uint8_t   button_pr;
         uint8_t   BeaconRSSI;
        uint8_t   IRLed0;
        uint8_t   IRLed1;
        uint8_t   IRLed2;
        // uint16_t  mySrcAddr;        // extended for old USB communication only
        uint8_t   Battery_lev;
        uint8_t   SyncFrameIMU;
        uint8_t   MsTimerIMU;
        uint8_t   IMUPktNum;
    };
    
    struct Beacon_Data_pkt {
        struct Beacon_Preamble BK_Preamble;
        struct BK_IMUData BeaconIMUData[BC_NUM_OF_IMU_PKTS_IN_RF_PKT];
    };
#endif

#pragma pack(pop)

#endif