#ifndef FLOATSAT_CONFIG_HPP_
#define FLOATSAT_CONFIG_HPP_

#include "OuterLoop/IMU.hpp"
#include "PIDController.hpp"

/**
 * @todo:   - Implement stepper config settings
 *          - Change the number of steps per mm to the correct value.
 *          
*/

namespace config
{
    /* ----------------------------------------- Threads -----------------------------------------  */
    inline const bool skip_init = false;                     // skips init routine

    // Inner Loop
    inline const bool innerloop_thread_enable = true;       // keep in mind it gets woken up by outer loop
    inline const int  innerloop_thread_period = 20;         // [ms]

    // Outer Loop
    inline const bool outerloop_thread_enable = true;
    inline const int  outerloop_thread_period = 200;        // [ms]

    // Docking Loop
    inline const bool docking_thread_enable = false;
    inline const int  docking_thread_period = 500;          // [ms]

    // Comms
    inline const bool com_thread_enable = true;
    inline const int  com_thread_period = 1000;             // [ms]

    // Debug
    inline const bool debug_thread_enable = false;
    inline const int  debug_thread_period = 1000;           // [ms]

    // ELetrical Monitoring. (DONT MESS WITH THIS! OR BEWARE OF THE MAGIC SMOKE/EXPLOSIONS!)
    inline const bool electrical_monitoring_thread_enable = true; //DO NOT DISABLE THIS THREAD. IT IS NEEDED TO PROTECT THE BATTERY! ONLY DISABLE IF YOU KNOW WHAT YOU ARE DOING AND REALLY NEED TO!
    inline const int  electrical_monitoring_thread_period = 100;       // [ms]

    /* ---------------------------------------- Hardware ---------------------------------------  */
    // IMU
    inline const IMUCalib gyroCalib{
        Vector3D_F(-0.00979, 0.01382, 0.03997),                                 // Offset [rad/s]
        Matrix3D_F(Vector3D_F(1,0,0), Vector3D_F(0,1,0), Vector3D_F(0,0,1))};  
    inline const IMUCalib accelCalib{
        Vector3D_F(-0.00616, -0.03305, -0.03945),                               // Offset [g]
        Matrix3D_F(Vector3D_F(1,0,0), Vector3D_F(0,1,0), Vector3D_F(0,0,1)) };  
    inline const IMUCalib magCalib{
        Vector3D_F(0.20510, 0.46420, 0.0),                                    // Offset [gauss]
        Matrix3D_F(Vector3D_F(1,0,0), Vector3D_F(0,1,0), Vector3D_F(0,0,1)) };

    // HBridge
    inline const int pwmFrequency = 2000;       // [Hz]
    inline const int pwmIncrements = 1000;

    // Stepper  
    inline const int microstepping = 2;         // Number of steps to make for a full step
    inline const int stepsPerRevolution = 200;  // Number of steps for a full revolution of the motor shaft
    inline const float steps2mm = 0.015625;     // Convertion from steps to mm Arm extention; 50mm entsprechen 20 Zacken also 1 Umdrehung
    inline const bool invertStepper = false;    // Invert stepper direction
    inline const bool enableStepper = true;     // Disable stepper (Enable pin i set to keeped high to disable driver)

    // Raspberry Pi
    inline const bool enable_rpi = true;       // Will not turn on rpi if true


    /* ---------------------------------------- Control ----------------------------------------  */
    // Speed Controller
    inline const float reactionwheelbase_vel = 300.f;  
    inline const float limitSpeedController = 12.f / 2.f;                           // [V]
    inline const PIDParams paramsSpeedControl{ 0.15f, 0.05f, 0.0f };                // P, I, D
    inline const bool antiwindupSpeedController = true;
    inline const bool derivativofmeasurmentSpeedController = false;
    // Position Controller
    inline const float limitPosController = (6 * 2 * M_PI) / 60.0f / 2.f;           // [rad/s]
    inline const PIDParams paramsPosController{ 1.2f, 0.f, 0.2f };                  // P, I, D
    inline const bool antiwindupPosController = false;
    inline const bool derivativofmeasurmentPosController = true;
    // Velocity Controller                                           
    inline const float limitVelController = (11000.0f * 2 * M_PI) / 60.0f / 2.f;    // [rad/s]
    inline const PIDParams paramsVelController{ -5.0f, -20.0f, 0.0f };              // P, I, D
    inline const bool antiwindupVelController = false;
    inline const bool derivativofmeasurmentVelController = false;

    // Arm Controller
    inline const int max_vel = 100;			// [step/s]
    inline const int min_vel = 10;			// [step/s]
    inline const int max_accel = 20;	   	// [step/s^2]
    inline const int deccel_margin = 10;    // [step]

    // IMU Calibration
    inline const int gyro_maxsamples = 80;
    inline const int accel_maxsamples = 80;
    inline const int mag_maxsamples = 200;

    // QEKF
    inline const Vector3D_F sigma_gyro = Vector3D_F(0.0027728, 0.0023483, 0.0018954);      // [rad/s]
    inline const Vector3D_F sigma_accel = Vector3D_F(0.0014530, 0.0017393, 0.0044343);     // [g]
    inline const float sigma_yaw = 0.018469;                                               // [rad]
    inline const float sigma_gyro_drift = 1.0f * powf(10, -6);                              // [rad/s^2]

    /* ------------------------------------------ Com -----------------------------------------  */

}

#endif
