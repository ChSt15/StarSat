#ifndef FLOATSAT_THREADS_CONFIG_HPP_
#define FLOATSAT_THREADS_CONFIG_HPP_

#include "../hardware/imu.hpp"
#include "../control/PIDController.hpp"

namespace config
{

    /* ----------------------------------------- Threads -----------------------------------------  */
    // Control
    inline const bool control_thread_enable = true;
    inline const int  control_thread_period = 100;     // [ms]

    // Sensor
    inline const bool sensor_thread_enable = true;
    inline const int  sensor_thread_period = 50;       // [ms]

    // Coms
    inline const bool com_thread_enable = true;
    inline const int  com_thread_period = 200;         // [ms]

    // Debug
    inline const bool debug_thread_enable = true;
    inline const int  debug_thread_period = 500;       // [ms]

    /* ---------------------------------------- Hardware ---------------------------------------  */
    // IMU
    inline const IMUCalib gyroCalib{
        Vector3D_F(0.011274, 0.018649, -0.021776),                      // Offset [rad/s]
        Matrix3D_F(YPR_F(0, 0, 0)) };                                   // Yaw, Pitch, Roll [rad]
    inline const IMUCalib accelCalib{
        Vector3D_F(-0.025037, -0.0051067, -0.0089848),                  // Offset [g]
        Matrix3D_F(YPR_F(0, 0, 0)) };                                   // Yaw, Pitch, Roll [rad]
    inline const IMUCalib magCalib{
        Vector3D_F(-0.025, 0.2625, -0.0495),                            // Offset [gauss]
        Matrix3D_F(YPR_F(0, 0, 0)) };                                   // Yaw, Pitch, Roll [rad]

    // HBridge
    inline const int pwmFrequency = 2000;       // [Hz]
    inline const int pwmIncrements = 500;

    /* ---------------------------------------- Control ----------------------------------------  */
    // Speed Controller
    inline const float limitSpeedController = 1.f;                      //
    inline const PIDParams paramsSpeedControl{ 1.0f, 1.0f, 1.0f };      // P, I, D
    // Position Controller
    inline const float limitPosController = 1.f;                        //
    inline const PIDParams paramsPosController{ 1.0f, 1.0f, 1.0f };     // P, I, D
    // Velocity Controller  
    inline const float limitVelController = 1.f;                        // 
    inline const PIDParams paramsVelController{ 1.0f, 1.0f, 1.0f };     // P, I, D

    // Arm Controller
    inline const int max_vel = 50;			// [step/s]
    inline const int min_vel = 1;			// [step/s]
    inline const int max_accel = 5;		    // [step/s^2]

    // IMU Calibration
    inline const int gyro_maxsamples = 200;
    inline const int accel_maxsamples = 200;
    inline const int mag_maxsamples = 500;

    // QEKF
    inline const Vector3D_F sigma_gyro = Vector3D_F(0.0027728, 0.0023483, 0.0018954);      // [rad/s]
    inline const Vector3D_F sigma_accel = Vector3D_F(0.0014530, 0.0017393, 0.0044343);     // [g]
    inline const float sigma_yaw = 0.018469;                                               // [rad]
    inline const float sigma_gyro_drift = 1.0f * pow(10, -6);                              // [rad/s^2]

    /* ------------------------------------------ Com -----------------------------------------  */

}

#endif