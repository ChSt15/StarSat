#include "OuterLoopThread.hpp"


static CommBuffer<TimestampedData<Attitude_Data>> AttitudeDataBuffer;
static Subscriber AttitudeDataSubsciber(AttitudeDataTopic, AttitudeDataBuffer, "Control Thread");

static CommBuffer<TimestampedData<float>> EncoderDataBuffer;
static Subscriber EncoderDataSubsciber(EncoderDataTopic, EncoderDataBuffer, "Control Thread");

static CommBuffer<TelemetryCamera> CameraDataBuffer;
static Subscriber CameraDataSubsciber(cameraDataTopic, CameraDataBuffer, "Control Thread");


HAL_GPIO ledorange(GPIO_061);

void OuterLoopThread::init()
{
	ledorange.init(true, 1, 0);
}


void OuterLoopThread::run()
{
	// Config
	using namespace config;
	{
		// Thread
		this->period = control_thread_period;
		if (!control_thread_enable) suspendCallerUntil(END_OF_TIME);

		//IMU 
		imu.initialization();

		// IMU Calibparams
		imu.setGyroCalib(gyroCalib);
		imu.setAccelCalib(accelCalib);
		imu.setMagCalib(magCalib);
		imucalib.config(gyro_maxsamples, accel_maxsamples, mag_maxsamples);

		// QEKF
		qekf.config(sigma_gyro, sigma_accel, sigma_yaw, sigma_gyro_drift);

		// Controllers
		positionControl.init(paramsPosController, limitPosController, backcalculationPosController, derivativofmeasurmentPosController);
		velocitycontrol.init(paramsVelController, limitVelController, maxVelocity, backcalculationVelController, derivativofmeasurmentVelController);

	}

	while (true)
	{
		// IMU
		IMUDataTopic.publish(imu.readData());

		// Atitude estimation
		AttitudeDataTopic.publish(qekf.estimate(imu.getData()));

		switch (getMode())
		{
		/* ---------------------------- Calib ---------------------------- */
		case Calib_Gyro:
			if (!imucalib.calibrateGyro(imu.getDataRaw())) break;
			setMode(Idle);
			break;

		case Calib_Accel:
			if (!imucalib.calibrateAccel(imu.getDataRaw())) break;
			setMode(Idle);
			break;

		case Calib_Mag:
			velocitycontrol.setDesiredAngularVelocity(M_PI / 16.f);
			publishSpeed(velocitycontrol.update(qekf.estimate());
			if (!imucalib.calibrateMag(imu.getDataRaw())) break;
			setMode(Idle);
			break;

		/* ---------------------------- Controller ---------------------------- */
		case Control_Vel:
			publishSpeed(velocitycontrol.update(qekf.estimate());
			break;

		case Control_Pos:
			publishSpeed(positionControl.update(qekf.estimate());
			break;

		/* ---------------------------- Mission ----------------------------- */
		case Mission_Locate:
			velocitycontrol.setDesiredAngularVelocity(M_PI / 16.f);
			publishSpeed(velocitycontrol.update(qekf.estimate());
			break;

		case Mission_Point:
		case Mission_Dock_initial:
		case Mission_Dock_final:
			positionControl.setDesiredAngle(CameraDataReceiver.getYawtoMockup() + AttitudeDataReceiver.data.attitude.toYPR().yaw);
			publishSpeed(positionControl.update(qekf.estimate());
			break;

		default:
			break;
		}

		ledorange.setPins(~ledorange.readPins());
		suspendCallerUntil(NOW() + period * MILLISECONDS);
	}
}

void OuterLoopThread::publishSpeed(float speed)
{
	speedSetpointTopic.publish(speed);
	innerLoopThread.resume();
}

ControlThread controlthread;