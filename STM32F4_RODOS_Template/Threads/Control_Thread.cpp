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


/**
 * @todo NEEDS TO BE SPECIFIED
*/
float maxVoltage = 10.0f;									// [V]
float maxSpeed = (10000.0f * 2 * M_PI) / 60.0f;				// [rad/s]
float maxVelocity = M_PI_2;  								// [rad/s]


HAL_GPIO ledorange(GPIO_061);

void ControlThread::init()
{
	/**
	 * Initialize HBridge
	*/
	hbridge.initialization();

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
		reactionwheelControl.init(paramsSpeedControl, maxVoltage, maxSpeed);
		positionControl.init(paramsPosController, maxVelocity);
		velocitycontrol.init(paramsVelController, maxSpeed, maxVelocity);
	}

	TimestampedData<Attitude_Data> AttitudeDataReceiver;
	TimestampedData<float> EncoderDataReceiver;

	float desiredVelocity;
	float desiredSpeed;
	float desiredVoltage;

	setMode(Mission_Dock_initial);

	while(true)
	{
		AttitudeDataBuffer.get(AttitudeDataReceiver);
		EncoderDataBuffer.get(EncoderDataReceiver);

		modes current_mode = getMode();

		switch (current_mode)
		{
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
			desiredVelocity = positionControl.update(AttitudeDataReceiver);

			velocitycontrol.setDesiredAngularVelocity(desiredVelocity);
			desiredSpeed = velocitycontrol.update(AttitudeDataReceiver);

			reactionwheelControl.setDesiredSpeed(desiredSpeed);
			desiredVoltage = reactionwheelControl.update(EncoderDataReceiver);

			hbridge.setVoltage(desiredVoltage);
			
			break;

		/* ---------------------------- Mission ----------------------------- */
		case Mission_Locate:
			break;

		case Mission_Point:
			break;

		case Mission_Dock_initial:
			// not complete, just to show principle
			if (!armController.InitialExtension()) break;

			setMode(Mission_Dock_final);
			break;

		case Mission_Dock_final:
			break;


		default:
			break;
		}

		ledorange.setPins(~ledorange.readPins());
		suspendCallerUntil(NOW() + period * MILLISECONDS);
	}
}


ControlThread controlthread;
