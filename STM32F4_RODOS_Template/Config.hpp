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
    // Inner Loop
    inline const bool innerloop_thread_enable = false;      // keep in mind it gets woke up by outer loop
    inline const int  innerloop_thread_period = 20;         // [ms]

    // Outer Loop
    inline const bool outerloop_thread_enable = false;
    inline const int  outerloop_thread_period = 100;        // [ms]

    // Docking Loop
    inline const bool docking_thread_enable = false;
    inline const int  docking_thread_period = 500;          // [ms]

    // Coms
    inline const bool com_thread_enable = false;
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
        Vector3D_F(0.011274, 0.018649, -0.021776),                              // Offset [rad/s]
        Matrix3D_F(Vector3D_F(1,0,0), Vector3D_F(0,1,0), Vector3D_F(0,0,1))};  
    inline const IMUCalib accelCalib{
        Vector3D_F(-0.025037, -0.0051067, -0.0089848),                          // Offset [g]
        Matrix3D_F(Vector3D_F(1,0,0), Vector3D_F(0,1,0), Vector3D_F(0,0,1)) };  
    inline const IMUCalib magCalib{
        Vector3D_F(-0.025, 0.2625, -0.0495),                                    // Offset [gauss]
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
    inline const bool enable_rpi = false;       // Will not turn on rpi if true


    /* ---------------------------------------- Control ----------------------------------------  */
    // Speed Controller
    inline const float limitSpeedController = 12.f;                         // [V]
    inline const PIDParams paramsSpeedControl{ 1.0f, 0.03f, 0.05f };        // P, I, D
    inline const bool backcalculationSpeedController = false;
    inline const bool derivativofmeasurmentSpeedController = false;
    // Position Controller
    inline const float limitPosController = (11000.0f * 2 * M_PI) / 60.0f;  // [rad/s]
    inline const PIDParams paramsPosController{ 1.0f, 0.0f, 0.0f };         // P, I, D
    inline const bool backcalculationPosController = false;
    inline const bool derivativofmeasurmentPosController = false;
    // Velocity Controller  
    inline const float limitVelController = (11000.0f * 2 * M_PI) / 60.0f;  // [rad/s]
    inline const PIDParams paramsVelController{ 1.0f, 0.0f, 0.0f };         // P, I, D
    inline const bool backcalculationVelController = false;
    inline const bool derivativofmeasurmentVelController = false;

    // Arm Controller
    inline const int max_vel = 100;			// [step/s]
    inline const int min_vel = 10;			// [step/s]
    inline const int max_accel = 20;	   	// [step/s^2]
    inline const int deccel_margin = 10;    // [step]

    // IMU Calibration
    inline const int gyro_maxsamples = 40;
    inline const int accel_maxsamples = 40;
    inline const int mag_maxsamples = 300;

    // QEKF
    inline const Vector3D_F sigma_gyro = Vector3D_F(0.0027728, 0.0023483, 0.0018954);      // [rad/s]
    inline const Vector3D_F sigma_accel = Vector3D_F(0.0014530, 0.0017393, 0.0044343);     // [g]
    inline const float sigma_yaw = 0.018469;                                               // [rad]
    inline const float sigma_gyro_drift = 1.0f * powf(10, -6);                              // [rad/s^2]

    /* ------------------------------------------ Com -----------------------------------------  */

}

#endif
