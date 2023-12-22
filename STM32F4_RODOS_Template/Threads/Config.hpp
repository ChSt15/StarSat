#ifndef FLOATSAT_THREADS_CONFIG_HPP_
#define FLOATSAT_THREADS_CONFIG_HPP_

#include "../hardware/imu.hpp"
#include "../control/PIDController.hpp"

/* ----------------------------------------- Threads -----------------------------------------  */
// Control
static const bool control_thread_enable = true;
static const int  control_thread_period = 100;     // [ms]

// Sensor
static const bool sensor_thread_enable = true;
static const int  sensor_thread_period = 50;       // [ms]

// Coms
static const bool com_thread_enable = true;
static const int  com_thread_period = 200;         // [ms]

// Debug
static const bool debug_thread_enable = false;
static const int  debug_thread_period = 500;       // [ms]

/* ---------------------------------------- Hardware ---------------------------------------  */
// IMU
static const IMUCalib gyroCalib{ 
    Vector3D_F(0,0,0),                  // Offset [rad/s]
    Matrix3D_F(YPR_F(0, 0, 0))};        // Yaw, Pitch, Roll [rad]
static const IMUCalib accelCalib{ 
    Vector3D_F(0,0,0),                  // Offset [g]
    Matrix3D_F(YPR_F(0, 0, 0))};        // Yaw, Pitch, Roll [rad]
static const IMUCalib magCalib{ 
    Vector3D_F(0,0,0),                  // Offset [gauss]
    Matrix3D_F(YPR_F(0, 0, 0))};        // Yaw, Pitch, Roll [rad]

// HBridge
static const int pwmFrequency = 2000;       // [Hz]
static const int pwmIncrements = 500;

/* ---------------------------------------- Control ----------------------------------------  */
// Speed Controller
static const float limitSpeedController = 1.f;                      // 
static const PIDParams paramsSpeedControl{ 1.0f, 1.0f, 1.0f };      // P, I, D
// Position Controller
static const float limitPosController = 1.f;                        //
static const PIDParams paramsPosController{ 1.0f, 1.0f, 1.0f };     // P, I, D
// Velocity Controller  
static const float limitVelController = 1.f;                        // 
static const PIDParams paramsVelController{ 1.0f, 1.0f, 1.0f };     // P, I, D

// Arm Controller
static const int max_vel = 50;			// [step/s]
static const int min_vel = 1;			// [step/s]
static const int max_accel = 5;		    // [step/s^2]

// IMU Calibration
static int gyro_maxsamples = 200;
static int accel_maxsamples = 200;
static int mag_maxsamples = 500;

// QEKF
static const Vector3D_F sigma_gyro = Vector3D_F(0.0027728, 0.0023483, 0.0018954);      // [rad/s]
static const Vector3D_F sigma_accel = Vector3D_F(0.0014530, 0.0017393, 0.0044343);     // [g]
static const float sigma_yaw = 0.018469;                                               // [rad]
static const float sigma_gyro_drift = 1.0f * pow(10, -6);                              // [rad/s^2]

/* ------------------------------------------ Com -----------------------------------------  */



#endif