#include "Control_Thread.hpp"

#include "rodos.h"
#include "matlib.h"

#include "../control/AttitudeEstimation.hpp"
#include "../control/PIDController.hpp"
#include "../control/ReactionwheelControl.hpp"
#include "../control/AngularVelocityControl.hpp"
#include "../control/AngularPositionControl.hpp"

#include "../hardware/ReactionwheelEncoder.hpp"
#include "../hardware/hbridge.hpp"

#include "Debug_Thread.hpp"


static CommBuffer<TimestampedData<Attitude_Data>> AttitudeDataBuffer;
static Subscriber AttitudeDataSubsciber(AttitudeDataTopic, AttitudeDataBuffer);

static CommBuffer<TimestampedData<float>> EncoderDataBuffer;
static Subscriber EncoderDataSubsciber(EncoderDataTopic, EncoderDataBuffer);


/**
 * @todo NEEDS TO BE SPECIFIED
*/
PIDParams paramsReactionWheelControl{ 1.0f, 1.0f, 1.0f };
PIDParams paramsVelocityControl{ 1.0f, 1.0f, 1.0f };
PIDParams paramsPositionControl{ 1.0f, 1.0f, 1.0f };
float maxVoltage = 10.0f;									// [V]
float maxSpeed = (10000.0f * 2 * M_PI) / 60.0f;				// [rad/s]
float maxVelocity = M_PI_2;  								// [rad/s]

void ControlThread::init()
{
	/**
	 * Initialize controller with PID parameters and max. Input/Output values
	 * @todo specify values in "Control_Thread.cpp", dummy data is used right now
	*/
	reactionwheelControl.init(paramsReactionWheelControl, maxVoltage, maxSpeed);
	velocitycontrol.init(paramsVelocityControl, maxSpeed, maxVelocity);
	positionControl.init(paramsPositionControl, maxVelocity);

	/**
	 * Initialize HBridge
	*/
	hbridge.initialization();
}



void ControlThread::run()
{
	TimestampedData<Attitude_Data> AttitudeDataReceiver;
	TimestampedData<float> EncoderDataReceiver;

	float desiredVelocity;
	float desiredSpeed;
	float desiredVoltage;

	while (true)
	{
		AttitudeDataBuffer.get(AttitudeDataReceiver);
		EncoderDataBuffer.get(EncoderDataReceiver);

		modes current_mode = getMode();

		switch (current_mode)
		{

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
		
		default:
			break;
		}


		suspendCallerUntil(NOW() + period * MILLISECONDS);
	}
}


ControlThread controlthread;