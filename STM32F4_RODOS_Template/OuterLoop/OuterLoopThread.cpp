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
	// Wait for Electrical
	while (getMode() == Electrical_Startup) suspendCallerUntil(NOW() + 200 * MILLISECONDS);

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

    // default values
	float temp1 = grad2Rad(0);
	float temp2 = M_PI/8.f;
	AngularPositionSetpointTopic.publish(temp1);
	AngularVelocitySetpointTopic.publish(temp2);

	int meas_cnt = 0;
    float mission_timeout;
	while (true)
	{	
		// Skip fist IMU values
		if (meas_cnt < 10) 
		{
			meas_cnt++;
			suspendCallerUntil(NOW() + period * MILLISECONDS);
			continue;
		}

		// IMU
		IMUDataTopic.publish(imu.readData());

		// Atitude estimation
		AttitudeDataTopic.publish(qekf.estimate(imu.getData()));

		switch (getMode())
		{
		case Standby:
            if (config::skip_init) setMode(Idle);

			velocitycontrol.setSetpoint(0.f);
            publishSpeed(velocitycontrol.update(qekf.getestimit()));
			break;
		/* ---------------------------- Calib ---------------------------- */
		case Calib_Gyro:
			if (!imucalib.calibrateGyro(imu.getDataRaw())) break;
			qekf.reset();
			telemetry.send_CalibIMU();
			setMode(Standby);
			break;

		case Calib_Accel:
			if (!imucalib.calibrateAccel(imu.getDataRaw())) break;
			qekf.reset();
			telemetry.send_CalibIMU();
			setMode(Standby);
			break;

		case Calib_Mag:
			velocitycontrol.setSetpoint(M_PI / 16.f);
			publishSpeed(velocitycontrol.update(qekf.getestimit()));
			if (!imucalib.calibrateMag(imu.getDataRaw())) break;
			telemetry.send_CalibIMU();
			qekf.reset();
			setMode(Standby);
			break;

		/* ---------------------------- Controller ---------------------------- */
		case Control_Vel:
			VelocitySetpointBuffer.getOnlyIfNewData(VelocitySetpointReceiver);
			velocitycontrol.setSetpoint(VelocitySetpointReceiver);

			publishSpeed(velocitycontrol.update(qekf.getestimit()));
			break;

		case Control_Pos:
            {
                PositionSetpointBuffer.getOnlyIfNewData(PositionSetpointReceiver);
                positionControl.setSetpoint(PositionSetpointReceiver);

                velocitycontrol.setSetpoint(positionControl.update(qekf.getestimit()));
                publishSpeed(velocitycontrol.update(qekf.getestimit()));
            }
			break;

		/* ---------------------------- Mission ----------------------------- */
		case Mission_Locate:
            {   
                mission_timeout = SECONDS_NOW();

                CameraDataBuffer.getOnlyIfNewData(CameraDataReceiver);
                CameraData camera;
                camera.telemetryCamera = CameraDataReceiver;

                float speed = M_PI / 16.f;
                if (camera.telemetryCamera.numLEDs >= 1) speed /= 2.f;

			    velocitycontrol.setSetpoint(speed);
			    publishSpeed(velocitycontrol.update(qekf.getestimit()));
			    break;
            }

		case Mission_Point:
            {   
                if (SECONDS_NOW() - mission_timeout > 2) setMode(Mission_Locate);

                CameraDataBuffer.getOnlyIfNewData(CameraDataReceiver);
                CameraData camera;
                camera.telemetryCamera = CameraDataReceiver;

                if (camera.validFrame())
                {
                    positionControl.setSetpoint(camera.getYawtoMockup() + qekf.getestimit().data.attitude.toYPR().yaw);
                    mission_timeout = SECONDS_NOW();
                }
                velocitycontrol.setSetpoint(positionControl.update(qekf.getestimit()));
                publishSpeed(velocitycontrol.update(qekf.getestimit()));

                if (abs(camera.getYawtoMockup()) < 0.1 && abs(qekf.getestimit().data.angularVelocity.z) < 0.2) setMode(Mission_Dock_initial);
	
            }
            break;

		case Mission_Dock_initial:
		case Mission_Dock_final:
			{   
                if (SECONDS_NOW() - mission_timeout > 2) setMode(Mission_Locate);

                // Get new Cameradata if availible
                CameraDataBuffer.getOnlyIfNewData(CameraDataReceiver);
                CameraData camera;
                camera.telemetryCamera = CameraDataReceiver;

                if (camera.validFrame())
                {
                    positionControl.setSetpoint(camera.getYawtoMockup() + qekf.getestimit().data.attitude.toYPR().yaw);
                    mission_timeout = SECONDS_NOW();
                }
                velocitycontrol.setSetpoint(positionControl.update(qekf.getestimit()));
                publishSpeed(velocitycontrol.update(qekf.getestimit()));
            }
			break;

		default:

			break;
		}

		//ledorange.setPins(~ledorange.readPins());
		suspendCallerUntil(NOW() + period * MILLISECONDS);
	}
}

void OuterLoopThread::publishSpeed(float speed)
{
	speedSetpointTopic.publish(speed);
	innerLoopThread.resume();
}


OuterLoopThread outerLoopThread;
