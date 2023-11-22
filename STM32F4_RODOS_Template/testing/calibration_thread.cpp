#include "calibration_thread.hpp"
#include "rodos.h"


CalibrationIMU::CalibrationIMU(const char* name, uint16_t numberOfSamples, uint16_t modePeriod, uint16_t calibPeriod) : Thread(name) 
{
    this->calibMode = DoNothing;
    this->numberOfSamples = numberOfSamples;
    this->modePeriod = modePeriod;
    this->calibPeriod = calibPeriod;
}


void CalibrationIMU::init(){
    blue_led.init(true, 1, 0);
}


void CalibrationIMU::run(){

    /// @brief Performing the three calibration routines after each other; then resume reading data in imu_thread.cpp
    // Wait initially 10 Seconds
    suspendCallerUntil(NOW() + 10*SECONDS);
    imu.setCalibRunning(true);

    blue_led.setPins(1);

    IMUCalib gyroCalib = calibrateGyro();
    IMUCalib accelCalib = calibrateAccel();
    IMUCalib magCalib = calibrateMag();

    imu.setAccelCalib(accelCalib);
    imu.setGyroCalib(gyroCalib);
    imu.setMagCalib(magCalib);
    
    blue_led.setPins(0);
    imu.setCalibRunning(false);
    imu.setCalibDone(true);
}



IMUCalib CalibrationIMU::calibrateAccel(){
    uint16_t counter = 0;
    float sum_x = 0.0;
    float sum_y = 0.0;
    float sum_z = 0.0;

    RODOS::PRINTF("----- Accelerometer Calibration -----\n\n");
    RODOS::PRINTF("Orientate IMU along the Z axis. Don't move the IMU sensor for %d seconds!\n\n", (((numberOfSamples*calibPeriod) / 1000) + 2));
    
    suspendCallerUntil(NOW() + 2 * SECONDS);
    while (counter < numberOfSamples)
    {   
        // Read data
        TimestampedData<IMUData> imuData = imu.readRawData();
        sum_x += imuData.data.angularVelocity.x;
        sum_y += imuData.data.angularVelocity.y;
        counter++;
        suspendCallerUntil(NOW() + calibPeriod * MILLISECONDS);
    }

    RODOS::PRINTF("Orientate IMU along the Y axis. Don't move the IMU sensor for %d seconds!\n\n", (((numberOfSamples*calibPeriod) / 1000) + 2));
    suspendCallerUntil(NOW() + 2 * SECONDS);
    counter = 0;
    while (counter < numberOfSamples)
    {   
        // Read data
        TimestampedData<IMUData> imuData = imu.readRawData();
        sum_x += imuData.data.angularVelocity.x;
        sum_z += imuData.data.angularVelocity.z;
        counter++;
        suspendCallerUntil(NOW() + calibPeriod * MILLISECONDS);
    }

    RODOS::PRINTF("Orientate IMU along the X axis. Don't move the IMU sensor for %d seconds!\n\n", (((numberOfSamples*calibPeriod) / 1000) + 2));
    suspendCallerUntil(NOW() + 2 * SECONDS);

    counter = 0;
    while (counter < numberOfSamples)
    {   
        // Read data
        TimestampedData<IMUData> imuData = imu.readRawData();
        sum_y += imuData.data.angularVelocity.y;
        sum_z += imuData.data.angularVelocity.z;
        counter++;
        suspendCallerUntil(NOW() + calibPeriod * MILLISECONDS);
    }

    // Average data and summarize in IMUcalib struct
    IMUCalib calib;
    calib.bias.x = sum_x / (2.0 * numberOfSamples);
    calib.bias.y = sum_y / (2.0 * numberOfSamples);
    calib.bias.z = sum_z / (2.0 * numberOfSamples);
    calib.scale = Matrix3D();
    
    RODOS::PRINTF("----- Accelerometer calibration finished! -----\n");
    RODOS::PRINTF("Calibration values for accelerometer [g]: bias_x: %f, bias_y: %f, bias_z: %f\n\n", calib.bias.x, calib.bias.y, calib.bias.z);
    return calib;
}




IMUCalib CalibrationIMU::calibrateMag(){
    float x_max = __FLT_MIN__;
    float y_max = __FLT_MIN__;
    float z_max = __FLT_MIN__;

    float x_min = __FLT_MAX__;
    float y_min = __FLT_MAX__;
    float z_min = __FLT_MAX__;

    float mx_current = 0.0;
    float my_current = 0.0;
    float mz_current = 0.0;
    
    RODOS::PRINTF("----- Magnetometer Calibration -----\n\nPlease move the IMU sensor in all directions for %d seconds!\n\n", (((numberOfSamples*calibPeriod) / 1000)));
    uint16_t counter = 0;
    while (counter < numberOfSamples)
    {   
        // Read data
        TimestampedData<IMUData> imuData = imu.readRawData();
        mx_current = imuData.data.magneticField.x;
        my_current = imuData.data.magneticField.y;
        mz_current = imuData.data.magneticField.z;
        // Check x limits
        if(mx_current > x_max) {
        	x_max = mx_current;
        }
        if(mx_current < x_min) {
        	x_min = mx_current;
        }
        // Check y limits
        if(my_current > y_max) {
        	y_max = my_current;
        }
        if(my_current < y_min) {
        	y_min = my_current;
        }
        // Check z limits
        if(mz_current > z_max) {
        	z_max = mz_current;
        }
        if(mz_current < z_min) {
        	z_min = mz_current;
        }

        suspendCallerUntil(NOW() + calibPeriod * MILLISECONDS);
        counter++;
    }

    // Average data and summarize in IMUcalib struct
    IMUCalib calib;
    /**
     * @todo convert min/max values in bias vector and scale matrix
    */
    calib.bias.x = 0.0;
    calib.bias.y = 0.0;
    calib.bias.z = 0.0;
    calib.scale = Matrix3D();

    RODOS::PRINTF("----- Magnetometer calibration finished! -----\n");
    RODOS::PRINTF("Calibration values for magnetometer [gauss]: x_min: %f, x_max: %f, y_min: %f, y_max: %f, z_min: %f, z_max: %f\n\n", x_min, x_max, y_min, y_max, z_min, z_max);

    return calib;
}




IMUCalib CalibrationIMU::calibrateGyro(){
    RODOS::PRINTF("----- Gyroscope Calibration -----\n\nPlease don't move the IMU sensor for %d seconds!\n\n", (((numberOfSamples*calibPeriod) / 1000) + 1));
    suspendCallerUntil(NOW() + 1 * SECONDS);

    uint16_t counter = 0;
    float sum_x = 0.0;
    float sum_y = 0.0;
    float sum_z = 0.0;

    while (counter < numberOfSamples)
    {   
        // Read data
        TimestampedData<IMUData> imuData = imu.readRawData();
        sum_x += imuData.data.angularVelocity.x;
        sum_y += imuData.data.angularVelocity.y;
        sum_z += imuData.data.angularVelocity.z;
        counter++;
        suspendCallerUntil(NOW() + calibPeriod * MILLISECONDS);
    }

    // Average data and summarize in IMUcalib struct
    IMUCalib calib;
    calib.bias.x = sum_x / counter;
    calib.bias.y = sum_y / counter;
    calib.bias.z = sum_z / counter;
    calib.scale = Matrix3D();

    RODOS::PRINTF("----- Gyroscope calibration finished! -----\n");
    RODOS::PRINTF("Calibration values for gyroscope [deg/s]: bias_x: %f, bias_y: %f, bias_z: %f\n\n", calib.bias.x, calib.bias.y, calib.bias.z);

    return calib;
}


CalibrationIMU calibration("Calibration", 1000, 100, 10);
