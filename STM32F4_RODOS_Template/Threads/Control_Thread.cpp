#include "Control_Thread.hpp"

#include "rodos.h"
#include "matlib.h"

#include "../control/AttitudeEstimation.hpp"
#include "../hardware/ReactionwheelEncoder.hpp"
#include "Debug_Thread.hpp"
#include "../hardware/hbridge.hpp"

static CommBuffer<TimestampedData<Attitude_Data>> AttitudeDataBuffer;
static Subscriber AttitudeDataSubsciber(AttitudeDataTopic, AttitudeDataBuffer);

static CommBuffer<TimestampedData<float>> EncoderDataBuffer;
static Subscriber EncoderDataSubsciber(EncoderDataTopic, EncoderDataBuffer);


void ControlThread::init()
{

}

void ControlThread::run()
{
	TimestampedData<Attitude_Data> AttitudeDataReceiver;
	TimestampedData<float> EncoderDataReceiver;

	while (true)
	{
		AttitudeDataBuffer.get(AttitudeDataReceiver);
		EncoderDataBuffer.get(EncoderDataReceiver);

		hbridge.setVoltage(0.05);

		suspendCallerUntil(NOW() + period * MILLISECONDS);
	}
}


ControlThread controlthread;