#include "OuterLoopThread.hpp"


static CommBuffer<TelemetryCamera> CameraDataBuffer;
static Subscriber CameraDataSubsciber(cameraDataTopic, CameraDataBuffer, "Control Thread");
static TelemetryCamera CameraDataReceiver;

static CommBuffer<float> VelocitySetpointBuffer;
static Subscriber VelocitySetpointSubsciber(AngularVelocitySetpointTopic, VelocitySetpointBuffer, "Control Thread");
static float VelocitySetpointReceiver;

static CommBuffer<float> PositionSetpointBuffer;
static Subscriber PositionSetpointSubsciber(AngularPositionSetpointTopic, PositionSetpointBuffer, "Control Thread");
static float PositionSetpointReceiver;

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
		this->period = outerloop_thread_period;
		if (!outerloop_thread_enable) suspendCallerUntil(END_OF_TIME);

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
		positionControl.config(paramsPosController, limitPosController, antiwindupPosController, derivativofmeasurmentPosController);
		velocitycontrol.config(paramsVelController, limitVelController, antiwindupVelController, derivativofmeasurmentVelController);

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
			velocitycontrol.setSetpoint(M_PI / 16.f);
			publishSpeed(velocitycontrol.update(qekf.getestimit()));
			if (!imucalib.calibrateMag(imu.getDataRaw())) break;
			setMode(Idle);
			break;

		/* ---------------------------- Controller ---------------------------- */
		case Control_Vel:
			VelocitySetpointBuffer.get(VelocitySetpointReceiver);
			velocitycontrol.setSetpoint(VelocitySetpointReceiver);

			publishSpeed(velocitycontrol.update(qekf.getestimit()));
			break;

		case Control_Pos:
			PositionSetpointBuffer.getOnlyIfNewData(PositionSetpointReceiver);
			positionControl.setSetpoint(PositionSetpointReceiver);

			publishSpeed(positionControl.update(qekf.getestimit()));
			break;

		/* ---------------------------- Mission ----------------------------- */
		case Mission_Locate:
			velocitycontrol.setSetpoint(M_PI / 16.f);
			publishSpeed(velocitycontrol.update(qekf.getestimit()));
			break;

		case Mission_Point:
		case Mission_Dock_initial:
		case Mission_Dock_final:

			// Get new Cameradata if availible
			CameraDataBuffer.getOnlyIfNewData(CameraDataReceiver);

			positionControl.setSetpoint(CameraDataReceiver.getYawtoMockup() + qekf.getestimit().data.attitude.toYPR().yaw);
			publishSpeed(positionControl.update(qekf.getestimit()));
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

OuterLoopThread outerLoopThread;
