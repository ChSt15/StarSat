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


static CommBuffer<TimestampedData<Attitude_Data>> AttitudeDataBuffer;
static Subscriber AttitudeDataSubsciber(AttitudeDataTopic, AttitudeDataBuffer, "Control Thread");

static CommBuffer<TimestampedData<float>> EncoderDataBuffer;
static Subscriber EncoderDataSubsciber(EncoderDataTopic, EncoderDataBuffer, "Control Thread");

static CommBuffer<TelemetryCamera> CameraDataBuffer;
static Subscriber CameraDataSubsciber(cameraDataTopic, CameraDataBuffer, "Control Thread");

/**
 * @todo NEEDS TO BE SPECIFIED
*/
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
		reactionwheelControl.init(paramsSpeedControl, limitSpeedController, maxSpeed, backcalculationSpeedController, derivativofmeasurmentSpeedController);
		positionControl.init(paramsPosController, limitPosController, backcalculationPosController, derivativofmeasurmentPosController);
		velocitycontrol.init(paramsVelController, limitVelController, maxVelocity, backcalculationVelController, derivativofmeasurmentVelController);

		// ArmController
		armController.config(max_vel, min_vel, max_accel, deccel_margin, steps2mm);

		// HBridge
		hbridge.initialization(pwmFrequency, pwmIncrements);
	}

	int frameCnt = -42;

	setMode(Control_Speed); 
	reactionwheelControl.setDesiredSpeed(200.f);

	while(true)
	{
		AttitudeDataBuffer.get(AttitudeDataReceiver);
		EncoderDataBuffer.get(EncoderDataReceiver);
		CameraDataBuffer.get(CameraDataReceiver);
		modes current_mode = getMode();

		switch (current_mode)
		{
		case Idle:
			hbridge.setVoltage(0.f);
			break;

		case Calib_Mag:
			velocitycontrol.setDesiredAngularVelocity(M_PI / 16.f);
			VelController_inControl();

			break;

		/* ---------------------------- Controller ---------------------------- */
		case Control_Speed:
			SpeedController_inControl();
			break;

		case Control_Vel:
			VelController_inControl();
			break;

		case Control_Pos:
			PosController_inControl();
			break;

		/* ---------------------------- Mission ----------------------------- */
		case Mission_Locate:
			velocitycontrol.setDesiredAngularVelocity(M_PI / 16.f);
			VelController_inControl();

			if (!CameraDataReceiver.validFrame(frameCnt)) break;

			setMode(Mission_Point);
			break;


		case Mission_Point:
			positionControl.setDesiredAngle(CameraDataReceiver.getYawtoMockup() + AttitudeDataReceiver.data.attitude.toYPR().yaw);
			PosController_inControl();

			if (!positionControl.isSettled()) break;

			setMode(Mission_Dock_initial);
			break;

		case Mission_Dock_initial:
			positionControl.setDesiredAngle(CameraDataReceiver.getYawtoMockup() + AttitudeDataReceiver.data.attitude.toYPR().yaw);
			PosController_inControl();

			if (!armController.InitialExtension(CameraDataReceiver))
			{
				armController.CalcAngularVelocity(CameraDataReceiver);
				break;
			}

			setMode(Mission_Dock_final);
			break;

		case Mission_Dock_final:
			positionControl.setDesiredAngle(CameraDataReceiver.getYawtoMockup() + AttitudeDataReceiver.data.attitude.toYPR().yaw);
			PosController_inControl();

			if (!armController.FinalExtension(CameraDataReceiver)) break;

			setMode(Idle);
			break;

		default:
			break;
		}

		ledorange.setPins(~ledorange.readPins());
		suspendCallerUntil(END_OF_TIME);
		//suspendCallerUntil(NOW() + period * MILLISECONDS);
	}
}

void ControlThread::SpeedController_inControl()
{
	hbridge.setVoltage(reactionwheelControl.update(EncoderDataReceiver));
}

void ControlThread::PosController_inControl()
{
	reactionwheelControl.setDesiredSpeed(positionControl.update(AttitudeDataReceiver));
	hbridge.setVoltage(reactionwheelControl.update(EncoderDataReceiver));
}

void ControlThread::VelController_inControl()
{
	reactionwheelControl.setDesiredSpeed(velocitycontrol.update(AttitudeDataReceiver));
	hbridge.setVoltage(reactionwheelControl.update(EncoderDataReceiver));
}

ControlThread controlthread;
