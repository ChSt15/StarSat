#ifndef FLOATSAT_HARDWARE_IMU_HPP_
#define FLOATSAT_HARDWARE_IMU_HPP_

#include "rodos.h"
#include "matlib.h"

struct IMU_Data 
{
    Vector3D angularVelocity;   // [rad/s]
    Vector3D magneticField;     // [gauss]
    Vector3D acceleration;      // [g]
    float temperature;          // [°C]
};

struct IMU_Calib
{
    Vector3D gyroOffset;        // [rad/s]
    Vector3D accelOffset;       // [g]
    Vector3D magMin;            // [gauss]
    Vector3D magMax;            // [gauss]
};


class IMU
{

public:

    IMU();

    // @brief Get IMU data
    // @return IMU data struct defined in imu.hpp (angularVelocity [rad/s], magneticField [gauss], acceleration [g] and temperature [°C])
    IMU_Data getData();

    // @brief Get IMU calibration data
    // @return IMU calibration struct defined in imu.hpp (angularVelocityOffset [rad/s], magneticFieldMin/Max [gauss], accelerationOffset [g])
    IMU_Calib getCalib();

    // @brif Sets IMU Calibration Parameters
    // @param calib -> IMU calibration struct defined in imu.hpp (angularVelocityOffset [rad/s], magneticFieldMin/Max [gauss], accelerationOffset [g])
    void setCalib(IMU_Calib calib);

private:

    // @brief Checks I2C Enable Pins (if connected), just for initial Testing/Debugging
    void Check_I2C_Enable();
    // @brief Checks the WHO_AM_I registers to confirm sensor adress is correct, just for initial Testing/Debugging
    void Check_WHOAMI();

    // @brief Initializes gyro
    void gyroInit();
    // @brief Reads gyro and saves to class varriable "data" (we can make it return the date aswell)
    void gyroRead();

    // @brief Initializes accelerometer
    void accelInit();
    // @brief Reads accelerometer and saves to class varriable "data" (we can make it return the date aswell)
    void accelRead();

    // @brief Initializes magnetometer
    void magInit();
    // @brief Reads magnetometer and saves to class varriable "data" (we can make it return the date aswell)
    void magRead();

    // @brief Reads Gyro and saves to class varriable "data" (we can make it return the date aswell)
    void TempRead();

    IMU_Data data;
    IMU_Calib calib;

    //// compiler complains if defined without decleration
    //HAL_I2C i2c;
};


extern IMU imu;

#endif
