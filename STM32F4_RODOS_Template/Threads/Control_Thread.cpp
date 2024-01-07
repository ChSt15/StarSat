#include "Control_Thread.hpp"

#include "rodos.h"
#include "matlib.h"

#include "../control/AttitudeEstimation.hpp"
#include "../control/PIDController.hpp"
#include "../control/ReactionwheelControl.hpp"
#include "../control/AngularVelocityControl.hpp"
#include "../control/AngularPositionControl.hpp"
#include "../control/ArmController.hpp"
#include "../hardware/ReactionwheelEncoder.hpp"
#include "../hardware/hbridge.hpp"

#include "Debug_Thread.hpp"


static CommBuffer<TimestampedData<Attitude_Data>> AttitudeDataBuffer;
static Subscriber AttitudeDataSubsciber(AttitudeDataTopic, AttitudeDataBuffer, "Control Thread");

static CommBuffer<TimestampedData<float>> EncoderDataBuffer;
static Subscriber EncoderDataSubsciber(EncoderDataTopic, EncoderDataBuffer, "Control Thread");

static CommBuffer<TelemetryCamera> CameraDataBuffer;
static Subscriber CameraDataSubsciber(cameraDataTopic, CameraDataBuffer, "Control Thread");

/**
 * @todo NEEDS TO BE SPECIFIED
*/
float maxVoltage = 10.0f;									// [V]
float maxSpeed = (10000.0f * 2 * M_PI) / 60.0f;				// [rad/s]
float maxVelocity = M_PI_2;  								// [rad/s]


HAL_GPIO ledorange(GPIO_061);

void ControlThread::init()
{
	ledorange.init(true, 1, 0);
}



void ControlThread::run()
{
	// Config
	using namespace config;
	{
		// Thread
		this->period = control_thread_period;
		if (!control_thread_enable) suspendCallerUntil(END_OF_TIME);

		// Controllers
		reactionwheelControl.init(paramsSpeedControl, maxVoltage, maxSpeed, backcalculationSpeedController, derivativofmeasurmentSpeedController);
		positionControl.init(paramsPosController, maxVelocity, backcalculationPosController, derivativofmeasurmentPosController);
		velocitycontrol.init(paramsVelController, maxSpeed, maxVelocity, backcalculationVelController, derivativofmeasurmentVelController);

		// ArmController
		armController.config(max_vel, min_vel, max_accel, deccel_margin);

		// HBridge
		hbridge.initialization(pwmFrequency, pwmIncrements);
	}

	TimestampedData<Attitude_Data> AttitudeDataReceiver;
	TimestampedData<float> EncoderDataReceiver;
	TelemetryCamera CameraDataReceiver;

	float desiredSpeed;
	float desiredVoltage;


	while(true)
	{
		AttitudeDataBuffer.get(AttitudeDataReceiver);
		EncoderDataBuffer.get(EncoderDataReceiver);

		modes current_mode = getMode();

		switch (current_mode)
		{
		case Idle:
			hbridge.setVoltage(0.f);
			break;
		case Calib_Mag:
			velocitycontrol.setDesiredAngularVelocity(M_PI / 16.f);
			desiredSpeed = velocitycontrol.update(AttitudeDataReceiver);
			reactionwheelControl.setDesiredSpeed(desiredSpeed);
			desiredVoltage = reactionwheelControl.update(EncoderDataReceiver);
			hbridge.setVoltage(desiredVoltage);

			break;

		/* ---------------------------- Controller ---------------------------- */
		case Control_Speed:
			desiredVoltage = reactionwheelControl.update(EncoderDataReceiver);

			hbridge.setVoltage(desiredVoltage);

			break;

		case Control_Vel:
			desiredSpeed = velocitycontrol.update(AttitudeDataReceiver);

			reactionwheelControl.setDesiredSpeed(desiredSpeed);
			desiredVoltage = reactionwheelControl.update(EncoderDataReceiver);

			hbridge.setVoltage(desiredVoltage);

			break;

		case Control_Pos:
			desiredSpeed = positionControl.update(AttitudeDataReceiver);

			reactionwheelControl.setDesiredSpeed(desiredSpeed);
			desiredVoltage = reactionwheelControl.update(EncoderDataReceiver);

			hbridge.setVoltage(desiredVoltage);
			
			break;

		/* ---------------------------- Mission ----------------------------- */
		case Mission_Locate:
			velocitycontrol.setDesiredAngularVelocity(M_PI / 16.f);
			desiredSpeed = velocitycontrol.update(AttitudeDataReceiver);
			reactionwheelControl.setDesiredSpeed(desiredSpeed);
			desiredVoltage = reactionwheelControl.update(EncoderDataReceiver);
			hbridge.setVoltage(desiredVoltage);

			if (CameraDataBuffer.getOnlyIfNewData(CameraDataReceiver)) break;

			setMode(Mission_Point);
			break;


		case Mission_Point:
			CameraDataBuffer.get(CameraDataReceiver);
			positionControl.setDesiredAngle(CameraDataReceiver.getYawtoMockup() + AttitudeDataReceiver.data.attitude.toYPR().yaw);
			desiredSpeed = positionControl.update(AttitudeDataReceiver);
			reactionwheelControl.setDesiredSpeed(desiredSpeed);
			desiredVoltage = reactionwheelControl.update(EncoderDataReceiver);
			hbridge.setVoltage(desiredVoltage);

			if (!positionControl.isSettled()) break;

			setMode(Mission_Dock_initial);
			break;

		case Mission_Dock_initial:
			CameraDataBuffer.get(CameraDataReceiver);
			positionControl.setDesiredAngle(CameraDataReceiver.getYawtoMockup() + AttitudeDataReceiver.data.attitude.toYPR().yaw);
			desiredSpeed = positionControl.update(AttitudeDataReceiver);
			reactionwheelControl.setDesiredSpeed(desiredSpeed);
			desiredVoltage = reactionwheelControl.update(EncoderDataReceiver);
			hbridge.setVoltage(desiredVoltage);

			if (!armController.InitialExtension())
			{
				armController.CalcAngularVelocity();
				break;
			}

			setMode(Mission_Dock_final);
			break;

		case Mission_Dock_final:
			CameraDataBuffer.get(CameraDataReceiver);
			positionControl.setDesiredAngle(CameraDataReceiver.getYawtoMockup() + AttitudeDataReceiver.data.attitude.toYPR().yaw);
			desiredSpeed = positionControl.update(AttitudeDataReceiver);
			reactionwheelControl.setDesiredSpeed(desiredSpeed);
			desiredVoltage = reactionwheelControl.update(EncoderDataReceiver);
			hbridge.setVoltage(desiredVoltage);

			if (!armController.FinalExtension()) break;

			setMode(Idle);
			break;


		default:
			break;
		}

		ledorange.setPins(~ledorange.readPins());
		suspendCallerUntil(NOW() + period * MILLISECONDS);
	}
}


ControlThread controlthread;
