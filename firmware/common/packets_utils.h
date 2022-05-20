// packets_utils.h
// Contains methods for converting values found in the packets into usable,
// and convienent values.
 
#ifndef BEACON_COMMON_PACKETSUTILS_H_
#define BEACON_COMMON_PACKETSUTILS_H_

// Beacon Common Includes
#include <common/packets.h>

// BlackBox Math Includes
#include <btcommon/algebra.h>

// Constants
/// <summary> The earth's gravitational constant in metres per second squared. </summary>
const double EARTH_GRAVITY = 9.80665;

namespace beacon {
namespace common {

/// <summary> The coefficient to multiply the IMU's acceleration data by to convert to Gs. </summary>
const double IMU_ACCEL_G_CONVERSION_COEFF = 0.001;

/// <summary> The coefficient to multiply the IMU's gyro data by to convert to degrees / s. </summary>
const double IMU_GYRO_DEGS_CONVERSION_COEFF = 0.05;

/// <summary> Converts the given num to a 14-bit 2's compliment integer. </summary>
/// <param name="num"> Number to convert. </param>
/// <returns> The value of num as a 14-bit 2's compliment integer. </returns>
inline int16_t To14Bit2sCompliment(uint16_t num) {
    if (num & 0x8000) {
        return -((~num) + 1);
    }
    else {
        return num;
    }
}

/// <summary> Converts the given raw acceleration value from the beacon IMU
/// into an appropriate metric value for the acceleration in metres per second squared.
/// </summary>
/// <param name="accelVal"> Original, raw IMU acceleration value to convert. </param>
/// <returns> The resulting acceleration value in m/s^2. </returns>
inline double IMUToMetricAccel(uint16_t accelVal) {
    return static_cast<double>(To14Bit2sCompliment(accelVal)) * 
        IMU_ACCEL_G_CONVERSION_COEFF * EARTH_GRAVITY;
}

/// <summary> Converts the given raw gyroscope value from the beacon IMU
/// into an appropriate metric value for the angular velocity in radians per second.
/// </summary>
/// <param name="gyroVal"> Original, raw IMU angular velocity (gyro) value to convert. </param>
/// <returns> The resulting angular velocity value in radians/s. </returns>
inline double IMUToMetricGyro(uint16_t gyroVal) {
    return blackbox::bbmath::DegreesToRadians<double>(
        static_cast<double>(To14Bit2sCompliment(gyroVal)) * IMU_GYRO_DEGS_CONVERSION_COEFF);
}

} // namespace common
} // namespace beacon


#endif // BEACON_COMMON_PACKETSUTILS_H_