#include "Control_Thread.hpp"

#include "rodos.h"
#include "matlib.h"

#include "../Modes.hpp"
#include "../OuterLoop/AngularVelocityControl.hpp"
#include "../InnerLoop/Hbridge.hpp"
#include "Adc.hpp"

void ControlThread::init()
{
	// Config
	using namespace config;

	// Initialize HBridge
	hbridge.initialization(pwmFrequency, pwmIncrements);

	// Initialize ADC
	adc.initialization();
}



void ControlThread::run()
{
	float desiredVoltage_percent;

	while(true)
	{
		desiredVoltage_percent = (adc.getVoltage() / 1500.f) - 1.f;

		int percent = int(desiredVoltage_percent * 100);

        int incrementsSize = 20;

        // Make percent in increments of 20
        percent = (percent / incrementsSize) * incrementsSize;

        float out = float(desiredVoltage_percent) * 0.2f;
        AngularVelocitySetpointTopic.publish(out);
		
		setMode(Control_Vel);
		suspendCallerUntil(NOW() + period * MILLISECONDS);
	}
}


ControlThread controlthread;
