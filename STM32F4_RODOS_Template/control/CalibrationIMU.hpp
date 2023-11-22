#ifndef FLOATSAT_CONTROL_CALIBRATIONIMU_HPP_
#define FLOATSAT_CONTROL_CALIBRATIONIMU_HPP_

#include "rodos.h"
#include "matlib.h"

#include "../timestamp.hpp"
#include "../hardware/imu.hpp"



extern IMU imu;


/// @brief Possible calibration modes.
enum CalibrationMode {
    DoNothing,
    CalibAccel,
    CalibGyro,
    CalibMag
};


class CalibrationIMU : public Thread
{
private:

    /// @brief Indicates the mode of the calibration thread.
    CalibrationMode calibMode;

	/// @brief Number of samples that should be used for determining the calibration data.
	int numberOfSamples;

    /// @brief Period of checking the calibMode in milliseconds.
    int modePeriod;

    /// @brief Period of getting IMU Data during calibration routine in milliseconds
    int calibPeriod;


public:

    void init() override;

    /// @brief Checks cyclically the current calibration mode and calls respective function
    /// First idea of structure in CalibrationIMU.cpp
    void run() override;

    /// @brief Get gyroscope data of extern defined IMU using getDataRaw() with this->calibPeriod and determines calibration values for gyroscope
    /// @return IMUCalib: Calibration values for gyroscope
    IMUCalib calibrateGyro();

    /// @brief Get accelerometer data of extern defined IMU using getDataRaw() with this->calibPeriod and determines calibration values for accelerometer
    /// @return IMUCalib: Calibration values for accelerometer
    IMUCalib calibrateAccel();

    /// @brief Get magnetometer data of extern defined IMU using getDataRaw() with this->calibPeriod and determines calibration values for magnetometer
    /// @return IMUCalib: Calibration values for magnetometer
    IMUCalib calibrateMag();

};


#endif
