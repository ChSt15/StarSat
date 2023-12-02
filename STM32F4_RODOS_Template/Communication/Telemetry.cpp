#include "Telemetry.hpp"


// IMU topic subscriber setup
static CommBuffer<TimestampedData<IMUData>> IMUDataBuffer;
static Subscriber IMUDataSubsciber(IMUDataTopic, IMUDataBuffer);
TimestampedData<IMUData> IMUDataReceiver;

// Attitude topic subscriber setup
static CommBuffer<TimestampedData<Attitude_Data>> AttitudeDataBuffer;
static Subscriber AttitudeDataSubsciber(AttitudeDataTopic, AttitudeDataBuffer);
TimestampedData<Attitude_Data> AttitudeDataReceiver;

// Encoder topic subscriber setup
static CommBuffer<TimestampedData<float>> EncoderDataBuffer;
static Subscriber EncoderDataSubsciber(EncoderDataTopic, EncoderDataBuffer);
TimestampedData<float> EncoderDataReceiver;


// Countinuos telemetry topic
Topic<TelemetryContinuous> telemetryContinuousTopic(TelemetryContinuousTopicID, "Continuous telemetry topic");
// Extended countinuos telemetry topic
Topic<TelemetryContinuousExtended> telemetryExtendedContinuousTopic(TelemetryContinuousExtendedTopicID, "Extended continuous telemetry topic");
// IMU calib telemetry topic
Topic<TelemetryCalibIMU> telemetryCalibIMUTopic(TelemetryIMUCalibTopicID, "IMU calib telemetry topic");
// Control params telemetry topic
Topic<TelemetryControlParams> telemetryControlParamsTopic(TelemetryControlParamsTopicID, "Control params telemetry topic");


void Telemetry::send_Continuous()
{
	
}

void Telemetry::send_ContinuousExtended()
{

}

void Telemetry::send_CalibIMU()
{

}

void Telemetry::send_ControlParams()
{

}


Telemetry telemetry;