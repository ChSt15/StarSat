#include "InnerLoopThread.hpp"


// SetPoint for Reactionwheel Subscriber
static CommBuffer<float> setPointBuffer;
static Subscriber setPointSubsciber(speedSetpointTopic, setPointBuffer, "Inner Loop Thread");
static float setPointReceiver = 0.f;


HAL_GPIO ledred(GPIO_062);

void InnerLoopThread::init()
{
	ledred.init(true, 1, 0);
}

void InnerLoopThread::run()
{
	// Config
	using namespace config;
	{
		// Thread 
		this->period = innerloop_thread_period;
		if (!innerloop_thread_enable) suspendCallerUntil(END_OF_TIME);

		// Encoder
		encoder.Init();

		// HBridge
		hbridge.initialization(pwmFrequency, pwmIncrements);

		// Controller
		reactionwheelControl.config(paramsSpeedControl, limitSpeedController, backcalculationSpeedController, derivativofmeasurmentSpeedController);
	}
	
	setMode(Control_Speed);
	reactionwheelControl.setSetpoint(200.f);


	while (true)
	{
		// Update setPoint if changed
		if (setPointBuffer.getOnlyIfNewData(setPointReceiver)) reactionwheelControl.setSetpoint(setPointReceiver);

		// Encoder measurment
		TimestampedData<float> encoder_speed = encoder.getSpeed();

        if (getMode() != Idle)
            hbridge.setVoltage(reactionwheelControl.update(encoder_speed));
        else 
            hbridge.setVoltage(0);

		// Publish Encoder measurment
		EncoderDataTopic.publish(encoder_speed);

		//ledred.setPins(~ledred.readPins());
		suspendCallerUntil(NOW() + period * MILLISECONDS);
	}
}


InnerLoopThread innerLoopThread;
