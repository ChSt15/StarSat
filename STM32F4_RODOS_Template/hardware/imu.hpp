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

/**
 * @brief Calibration data for an imu sensor (e.g. magnetometer, accel, gyro)
 * @note This is a generic struct. Each sensor is calibrated using: y = scale * (x - bias). For gyro and accel, scale is probably identity matrix.
*/
struct IMUCalib
{
    Vector3D bias;       // Bias of sensor. E.g. for magnetometer this is the soft iron offset.
    Matrix3D scale;      // Scale of sensor. E.g. for magnetometer this is the hard iron offset. Also deals with non-orthogonality.
};


extern Topic<TimestampedData<IMUData>> IMUDataTopic;

class IMU
{
private:

    /// @brief Data from IMU with calibration
    TimestampedData<IMUData> dataCalibrated;
    /// @brief Data from IMU without calibration
    TimestampedData<IMUData> dataRaw;

    IMUCalib gyroCalib;
    IMUCalib accelCalib;
    IMUCalib magCalib;

    HAL_I2C i2c;

    /// @brief Scale factor to convert data of sensors in corresponding units; depends on the initialized range
    const float gyroScale = 70 / 1000.0;            // +-2000dps -> 70, +-500dps -> 17.50, +-245dps -> 8.75
    const float accelScale = 0.061 / 1000.0;        // +-2g -> 0.061, +-4g -> 0.122, +-8g -> 0.244, +-16g -> 0.732
    const float magScale = 0.14 / 1000.0;           // +-4gauss -> 0.14, +-8gauss -> 0.29, +-12gauss -> 0.43, +-16gauss -> 0.58

    /// @brief define adresses of registers for reading data
    /// Gyroscope
    const uint8_t LSM9DS1_G_OUT_X[2] = { 0x18 };
    const uint8_t LSM9DS1_G_OUT_Y[2] = { 0x1A };
    const uint8_t LSM9DS1_G_OUT_Z[2] = { 0x1C };
    /// Accelerometer
    const uint8_t LSM9DS1_XL_OUT_X[2] = { 0x28 };
    const uint8_t LSM9DS1_XL_OUT_Y[2] = { 0x2A };
    const uint8_t LSM9DS1_XL_OUT_Z[2] = { 0x2C };
    /// Magnetometer
    const uint8_t LSM9DS1_M_OUT_X[2] = { 0x28 };
    const uint8_t LSM9DS1_M_OUT_Y[2] = { 0x2A };
    const uint8_t LSM9DS1_M_OUT_Z[2] = { 0x2C };
    // Temperature
    const uint8_t LSM9DS1_OUT_TEMP[2] = { 0x15 };

    /**
     * ------- ONLY FOR TESTING -------
    */
   bool calibRunning = false;       // for pausing the IMU Thread
   bool calibDone = false;          // for indicating whether calibration values are determined and can be applied to raw data


public:

    /**
     * @brief Construct a new IMU object
     * @param i2c I2C bus to which the IMU is connected
     * @note Might need to add params for the I2C pins
    */
    IMU(RODOS::I2C_IDX i2c = RODOS::I2C_IDX1);

    /// @brief Get IMU data. Used by the attitude estimation.
    /// @return Timestamped IMU data struct defined in imu.hpp. Timestamp is of time of reading.
    TimestampedData<IMUData> getData();

    /// @brief Gets the uncalibrated IMU data. Used by the calibration routine.
    /// @return Timestamped IMU data struct defined in imu.hpp. Timestamp is of time of reading.
    TimestampedData<IMUData> getDataRaw();

    /// @brief Set the Gyro calibration values
    /// @param calib 
    void setGyroCalib(const IMUCalib& calib);

    /// @brief Get the Gyro calibration values
    const IMUCalib& getGyroCalib();

    /// @brief Set the Accel calibration values
    /// @param calib
    void setAccelCalib(const IMUCalib& calib);

    /// @brief Get the Accel calibration values
    const IMUCalib& getAccelCalib();

    /// @brief Set the Mag calibration values
    /// @param calib
    void setMagCalib(const IMUCalib& calib);

    /// @brief Get the Mag calibration values
    const IMUCalib& getMagCalib();

    /// @brief Checks I2C Enable Pins (if connected), just for initial Testing/Debugging
    void Check_I2C_Enable();
    /// @brief Checks the WHO_AM_I registers to confirm sensor adress is correct, just for initial Testing/Debugging
    void Check_WHOAMI();

    /// @brief Calls subroutines and initialize the IMU
    void initialization();

    /// @brief Calls subroutines and reads raw data of IMU and saves it into this->dataRaw
    /// @return saved this->dataRaw
    TimestampedData<IMUData>& readRawData();

    /// @brief Calibrate current raw data (this->dataRaw) by applying current calibration values and saves to this->dataCalibrated
    void calibrateData();

    /**
     * ------- ONLY FOR TESTING -------
    */
    bool isCalibRunning();
    void setCalibRunning(bool status);

    bool isCalibDone();
    void setCalibDone(bool status);

private:

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
    void tempRead();

    /// @brief Converts read data to float
    static float bytes_to_float(uint8_t* data);

};


extern IMU imu;

#endif
