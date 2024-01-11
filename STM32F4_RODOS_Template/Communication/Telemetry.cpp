#include "Telemetry.hpp"


// IMU topic subscriber setup
static CommBuffer<TimestampedData<IMUData>> IMUDataBuffer;
static Subscriber IMUDataSubsciber(IMUDataTopic, IMUDataBuffer, "Telemetry Class");
TimestampedData<IMUData> IMUDataReceiver;

// Attitude topic subscriber setup
static CommBuffer<TimestampedData<Attitude_Data>> AttitudeDataBuffer;
static Subscriber AttitudeDataSubsciber(AttitudeDataTopic, AttitudeDataBuffer, "Telemetry Class");
TimestampedData<Attitude_Data> AttitudeDataReceiver;

// Encoder topic subscriber setup
static CommBuffer<TimestampedData<float>> EncoderDataBuffer;
static Subscriber EncoderDataSubsciber(EncoderDataTopic, EncoderDataBuffer, "Telemetry Class");
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
	// Status
	telemetry_continuous.modeid = getMode();
	telemetry_continuous.cmdCnt = telecommand.getCommandCounter();
	telemetry_continuous.lastcmdid = telecommand.getLastCommand();
	telemetry_continuous.time = SECONDS_NOW();

	// IMU
	IMUDataBuffer.get(IMUDataReceiver);
	telemetry_continuous.wx = IMUDataReceiver.data.angularVelocity.x;
	telemetry_continuous.wy = IMUDataReceiver.data.angularVelocity.y;
	telemetry_continuous.wz = IMUDataReceiver.data.angularVelocity.z;
	telemetry_continuous.ax = IMUDataReceiver.data.acceleration.x;
	telemetry_continuous.ay = IMUDataReceiver.data.acceleration.y;
	telemetry_continuous.az = IMUDataReceiver.data.acceleration.z;
	telemetry_continuous.mx = IMUDataReceiver.data.magneticField.x;
	telemetry_continuous.my = IMUDataReceiver.data.magneticField.y;
	telemetry_continuous.mz = IMUDataReceiver.data.magneticField.z;
	telemetry_continuous.temp = IMUDataReceiver.data.temperature;

	// Attitude
	AttitudeDataBuffer.get(AttitudeDataReceiver);
	telemetry_continuous.q0 = AttitudeDataReceiver.data.attitude.q0;
	telemetry_continuous.q1 = AttitudeDataReceiver.data.attitude.q.x;
	telemetry_continuous.q2 = AttitudeDataReceiver.data.attitude.q.y;
	telemetry_continuous.q3 = AttitudeDataReceiver.data.attitude.q.z;

	// Encoder
	EncoderDataBuffer.get(EncoderDataReceiver);
	telemetry_continuous.speed = EncoderDataReceiver.data;

	// Arm
	telemetry_continuous.arm_extension = armController.getArmExtention();

	// Electrical
	// TODO: not thread safe yet
	telemetry_continuous.U_bat = electricalMonitor.getBatteryVoltage();
	telemetry_continuous.I_total = electricalMonitor.getAuxCurrent() + electricalMonitor.getReactionWheelCurrent() + electricalMonitor.getRPICurrent() + electricalMonitor.getStepperCurrent();

	telemetryContinuousTopic.publish(telemetry_continuous);
	

	if (this->enable_extendedtelem)
	{
		// TODO
		telemetry_extended.speedControlOut = 0.;
		telemetry_extended.posControlOut = 0.;
		telemetry_extended.velControlOut = 0.;
		telemetry_extended.arm_Calib = false;
		telemetry_extended.I_reac = 0.;

		telemetryExtendedContinuousTopic.publish(telemetry_extended);
	}
}

void Telemetry::send_CalibIMU()
{
	IMUCalib calib;

	// Gyro
	calib = imu.getGyroCalib();
	telemetry_calib.gyro_offx = calib.bias.x;
	telemetry_calib.gyro_offy = calib.bias.y;
	telemetry_calib.gyro_offz = calib.bias.z;

	// Accelerometer
	calib = imu.getAccelCalib();
	telemetry_calib.accel_offx = calib.bias.x;
	telemetry_calib.accel_offy = calib.bias.y;
	telemetry_calib.accel_offz = calib.bias.z;

	// Magnetometer
	calib = imu.getMagCalib();
	telemetry_calib.mag_offx = calib.bias.x;
	telemetry_calib.mag_offy = calib.bias.y;
	telemetry_calib.mag_offz = calib.bias.z;

	telemetryCalibIMUTopic.publish(telemetry_calib);
}

void Telemetry::send_ControlParams()
{
	PIDParams params;

	// Speed control
	params = reactionwheelControl.getParams();
	telemetry_control.speed_P = params.kp;
	telemetry_control.speed_I = params.ki;
	telemetry_control.speed_D = params.kd;
	telemetry_control.speed_lim = reactionwheelControl.getMaxVoltage();

	// Position control
	params = positionControl.getParams();
	telemetry_control.pos_P = params.kp;
	telemetry_control.pos_I = params.ki;
	telemetry_control.pos_D = params.kd;
	telemetry_control.pos_lim = positionControl.getMaxAngularVelocity();

	// Velocity control
	params = velocitycontrol.getParams();
	telemetry_control.vel_P = params.kp;
	telemetry_control.vel_I = params.ki;
	telemetry_control.vel_D = params.kd;
	telemetry_control.vel_lim = velocitycontrol.getMaxSpeed();

	telemetryControlParamsTopic.publish(telemetry_control);
}

void Telemetry::enable_ExtendedTelemetry(bool enable)
{
	this->enable_extendedtelem = enable;
}


Telemetry telemetry;
