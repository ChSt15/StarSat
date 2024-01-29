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
	// Wait for Electrical
	while (getMode() == Electrical_Startup) suspendCallerUntil(NOW() + 200 * MILLISECONDS);

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
		if (skip_init) reactionwheelControl.config(paramsSpeedControl, limitSpeedController, antiwindupSpeedController, derivativofmeasurmentSpeedController, 0.f);
		else reactionwheelControl.config(paramsSpeedControl, limitSpeedController, antiwindupSpeedController, derivativofmeasurmentSpeedController, reactionwheelbase_vel);
	}


	//setMode(Control_Speed);
	//reactionwheelControl.setSetpoint(0.f);

	while (true)
	{
		//if (SECONDS_NOW() > 10.f) reactionwheelControl.setSetpoint(100.f);
		
		// Update setPoint if changed
		if (setPointBuffer.getOnlyIfNewData(setPointReceiver)) reactionwheelControl.setSetpoint(setPointReceiver);
		
		// Encoder measurment
		TimestampedData<float> encoder_speed = encoder.getSpeed();

        //setMode(Mission_Locate);

		switch (getMode())
		{
		case Idle:
			hbridge.setVoltage(0.f);
			break;
		case Reactionwheel_Spinup:
			reactionwheelControl.setSetpoint(0.f);
			hbridge.setVoltage(reactionwheelControl.update(encoder_speed));

			if (!reactionwheelControl.isSettled()) break;

			setMode(Standby);
			break;
		default:
            {
                /*reactionwheelControl.setSetpoint(100);
                if (SECONDS_NOW() > 10)
                    reactionwheelControl.setSetpoint(-100);*/
                float v = reactionwheelControl.update(encoder_speed);
                //PRINTF("IS: %f, O: %f\n", encoder_speed.data, v);
                hbridge.setVoltage(v);
            }
			break;
		}

		// Publish Encoder measurment
		EncoderDataTopic.publish(encoder_speed);

		ledred.setPins(~ledred.readPins());
		suspendCallerUntil(NOW() + period * MILLISECONDS);
	}
}


InnerLoopThread innerLoopThread;
