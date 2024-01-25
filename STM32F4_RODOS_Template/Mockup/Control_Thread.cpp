#include "Control_Thread.hpp"

#include "rodos.h"
#include "matlib.h"

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
	float desiredVoltage;

	while(true)
	{
		desiredVoltage = adc.getVoltage();
		// hbridge.setVoltage(desiredVoltage);

		suspendCallerUntil(NOW() + period * MILLISECONDS);
	}
}


ControlThread controlthread;
