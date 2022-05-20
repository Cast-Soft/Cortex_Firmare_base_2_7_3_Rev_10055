// For Bridge only. T.K. does not use this file

#include "packets.h"

TKFrame& InitTKFrame(TKFrame& frame) {
    frame.signature = TK_FRAME_SIGNATURE;      
    frame.version_major = TK_VERSION_MAJOR;
    frame.version_revision = TK_VERSION_REVISION;
    frame.version_minor = TK_VERSION_MINOR;
    frame.version_build = TK_VERSION_BUILD;

    return frame;
}

bool USING_USB_CAMERA_SYSTEM = false;

static uint8_t circularShiftLeft(uint8_t val)
{
	uint8_t temp = val;
	val <<= 1;
	if(temp < val) return val;
	return val |= 0x01;
}

void convertForUsbError(struct TKBeaconStatusDetail* detail)
{
	if(USING_USB_CAMERA_SYSTEM) {
		detail->irLed0 = circularShiftLeft(detail->irLed0);
		detail->irLed1 = circularShiftLeft(detail->irLed1);
		detail->irLed2 = circularShiftLeft(detail->irLed2);
	}
}
