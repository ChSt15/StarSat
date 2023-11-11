#ifndef FLOATSAT_HARDWARE_IMU_HPP_
#define FLOATSAT_HARDWARE_IMU_HPP_

#include "rodos.h"
#include "matlib.h"

#include "../timestamp.hpp"


struct IMUData 
{
    Vector3D angularVelocity;   // [rad/s]
    Vector3D magneticField;     // [gauss]
    Vector3D acceleration;      // [g]
    float temperature;          // [�C]
};

struct IMUCalib
{
    Vector3D gyroOffset;        // [rad/s]
    Vector3D accelOffset;       // [g]
    Vector3D magOffset;         // [gauss] Is for hard iron calibration. Calibration: B = magScale*(B_raw - magOffset)
    Matrix3D magScale;          // [gauss] Is for soft iron calibration, could theoretically deal with axis misalignment aswell. Calibration: B = magScale*(B_raw - magOffset)
};


class IMU
{
private:

    TimestampedData<IMUData> data;
    IMUCalib calib;

    HAL_I2C i2c;

public:

    /**
     * @brief Construct a new IMU object
     * @param i2c I2C bus to which the IMU is connected
     * @note Might need to add params for the I2C pins
    */
    IMU(RODOS::I2C_IDX i2c);

    /// @brief Get IMU data
    /// @return Timestamped IMU data struct defined in imu.hpp. Timestamp is of time of reading.
    TimestampedData<IMUData> getData();

    /// @brief Get IMU calibration data
    /// @return IMU calibration struct defined in imu.hpp (angularVelocityOffset [rad/s], magneticFieldMin/Max [gauss], accelerationOffset [g])
    IMUCalib getCalib();

    /// @brif Sets IMU Calibration Parameters
    /// @param calib -> IMU calibration struct defined in imu.hpp (angularVelocityOffset [rad/s], magneticFieldMin/Max [gauss], accelerationOffset [g])
    void setCalib(IMUCalib calib);

private:

    /// @brief Checks I2C Enable Pins (if connected), just for initial Testing/Debugging
    void Check_I2C_Enable();
    /// @brief Checks the WHO_AM_I registers to confirm sensor adress is correct, just for initial Testing/Debugging
    void Check_WHOAMI();

    /// @brief Initializes gyro
    void gyroInit();
    /// @brief Reads gyro and saves to class varriable "data" (we can make it return the date aswell)
    void gyroRead();

    /// @brief Initializes accelerometer
    void accelInit();
    /// @brief Reads accelerometer and saves to class varriable "data" (we can make it return the date aswell)
    void accelRead();

    /// @brief Initializes magnetometer
    void magInit();
    /// @brief Reads magnetometer and saves to class varriable "data" (we can make it return the date aswell)
    void magRead();

    /// @brief Reads Gyro and saves to class varriable "data" (we can make it return the date aswell)
    void TempRead();

};


extern IMU imu;

#endif
