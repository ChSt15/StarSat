#ifndef FLOATSAT_COMMUNICATION_TELEMETRYLIST_HPP_
#define FLOATSAT_COMMUNICATION_TELEMETRYLIST_HPP_

#define TelemetryContinuousTopicID 40
#define TelemetryContinuousExtendedTopicID 41
#define TelemetryIMUCalibTopicID 42
#define TelemetryControlParamsTopicID 43


// @brief Structure countinuos telemetry
struct TelemetryContinuous
{
    // Status
    int modeid;
    int cmdCnt;
    float time;             // [sec]
    // IMU
    float wx, wy, wz;       // [rad/s]
    float ax, ay, az;       // [g]
    float mx, my, mz;       // [gauss]
    float temp;             // [°C]
    // Attitude Quaternion
    float q0, q1, q2, q3;
    // Encoder
    float speed;            // [rad/s]
    // Arm
    float arm_extension;    // [cm]
    // Electrical
    float U_bat;            // [V]
    float I_total;          // [A]
};

// @brief Structure extended countinuos telemetry
struct TelemetryContinuousExtended
{
    // Control
    float speedControlOut, posControlOut, velControlOut;
    // Arm
    bool arm_Calib;
    // Electrical
    float I_reac;           // [A]
    // ...
};

// @brief Structure IMU calib telemetry
struct TelemetryCalibIMU
{
    float gyro_offx, gyro_offy, gyro_offz;          // [rad/s]
    float accel_offx, accel_offy, accel_offz;       // [g]
    float mag_offx, mag_offy, mag_offz;             // [gauss]
};


// @brief Structure control params telemetry
struct TelemetryControlParams
{
    float speed_P, speed_I, speed_D, speed_lim;
    float pos_P, pos_I, pos_D, pos_lim;
    float vel_P, vel_I, vel_D, vel_lim;
};

// not used in STM
struct TelemetryCamera
{
    float rx, ry, rz;       // [Rodriguez Rot]
    float px, py, pz;       // [mm]
    uint32_t MeasurmentCnt;
};


#endif