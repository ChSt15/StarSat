#include "ArmController.hpp"

#define STEP2LENGTH 1 // [mm]


// Camera topic subscriber setup
static CommBuffer<TelemetryCamera> CameraDataBuffer;
static Subscriber CameraDataSubsciber(cameraDataTopic, CameraDataBuffer);
TelemetryCamera CameraDataReceiver;


bool inital = true;
float timestamp;

bool ArmController::InitialExtension()
{
	if (inital)
	{
		// TODO Coordinate transfrom
		CameraDataBuffer.get(CameraDataReceiver);
		this->distance = sqrt(pow(CameraDataReceiver.px, 2) + pow(CameraDataReceiver.py, 2) + pow(CameraDataReceiver.pz, 2));

		steppermotorthread.setPeriond((int) (0.1 * max_vel));
		steppermotorthread.setStepsToDo((int)(0.9f * distance / STEP2LENGTH));
		timestamp = SECONDS_NOW();
		inital = false;

		return false;
	}
	else
	{
		// schon da ?
		if (true)
		{
			return true;
		}

		// Abbremsen ? 
		if (true)
		{

		}
		else
		{
			// schon auf max vel ?
			if (true)
			{

			}
		}
		return false;
	}
}

bool ArmController::FinalExtension()
{

}


bool ArmController::CalcAngularVelocity()
{
	// mittel 10 werte
}

float ArmController::getAngularvelocityMockup()
{
	return this->w_Mockup.get();
}


ArmController armController;
