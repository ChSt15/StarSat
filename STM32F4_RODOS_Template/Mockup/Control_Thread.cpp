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
	float desiredVoltage_percent;

	while(true)
	{
		desiredVoltage_percent = adc.getVoltage() / 3000.f;

		if (desiredVoltage_percent > 0.95) desiredVoltage_percent = 1.f;
		else if (desiredVoltage_percent > 0.75) desiredVoltage_percent = 0.8f;
		else if (desiredVoltage_percent > 0.55) desiredVoltage_percent = 0.6f;
		else if (desiredVoltage_percent > 0.35) desiredVoltage_percent = 0.4f;
		else if (desiredVoltage_percent > 0.15) desiredVoltage_percent = 0.2f;
		else desiredVoltage_percent = 0.f;

		desiredVoltage_percent *= 0.2;



		hbridge.setVoltage(desiredVoltage_percent);
		//PRINTF("Desired Voltage:    %f\n", desiredVoltage_percent);

		suspendCallerUntil(NOW() + period * MILLISECONDS);
	}
}


ControlThread controlthread;
