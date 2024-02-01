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
		desiredVoltage_percent = adc.getVoltage() / 3000.f;

		int percent = int(desiredVoltage_percent * 100);

        int incrementsSize = 20;
        // Make percent in increments of 20
        percent = (percent / incrementsSize) * incrementsSize;

		//percent *= 2000/100;

		//hbridge.setVoltage(desiredVoltage_percent);
		//PRINTF("Desired Voltage:    %f\n", desiredVoltage_percent);

        //speedSetpointTopic.publish(desiredVoltage_percent);
        float out = float(desiredVoltage_percent)/100*2*3/6;// percent;
        AngularVelocitySetpointTopic.publish(out);

        if (percent < 1) {
            setMode(Control_Vel);
        } else {
            setMode(Idle);
        }

		suspendCallerUntil(NOW() + period * MILLISECONDS);
	}
}


ControlThread controlthread;
