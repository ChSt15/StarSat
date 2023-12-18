#include "ArmController.hpp"

#define STEP2LENGTH 1		// [mm]
#define DECCELMARGIN 50		// [steps]


// Camera topic subscriber setup
static CommBuffer<TelemetryCamera> CameraDataBuffer;
static Subscriber CameraDataSubsciber(cameraDataTopic, CameraDataBuffer, "Arm Controller Class");
TelemetryCamera CameraDataReceiver;


float dt;
float last_t;

bool ArmController::InitialExtension()
{
	if (!moving)
	{
		// TODO Coordinate transfrom
		/*
		CameraDataBuffer.get(CameraDataReceiver);
		this->distance = sqrt(pow(CameraDataReceiver.px, 2) + pow(CameraDataReceiver.py, 2) + pow(CameraDataReceiver.pz, 2));
		*/ 

		// just for test
		this->distance = 200;

		steppermotorthread.setDirection(true);
		steppermotorthread.setPeriod((int) (1.f/ min_vel));
		steppermotorthread.setStepsToDo((int)(0.9f * distance / STEP2LENGTH));
		last_t = SECONDS_NOW();
		inital = false;

		return false;
	}
	else
	{
		// Check if done
		if (steppermotorthread.getStatus())
		{
			moving = false;
			return true;
		}

		dt = SECONDS_NOW() - last_t;
		last_t = SECONDS_NOW();

		// Decelerate to vel_min
		if ((steppermotorthread.getStepsToDo() - DECCELMARGIN) < pow((1.f /steppermotorthread.getPeriod()), 2) / max_accel) // conservativ calc
		{
			int t = (int)(1.f / (1.f / steppermotorthread.getPeriod() - max_accel * dt));
			(t < 1.f / min_vel) ? steppermotorthread.setPeriod(t) : steppermotorthread.setPeriod((int) (1.f / min_vel));
			return false;
		}

		// Accelerate to vel_max
		if (steppermotorthread.getPeriod() < 1.f/max_vel)
		{
			int t = (int)(1.f / (1.f / steppermotorthread.getPeriod() + max_accel * dt));
			(t > 1.f / max_vel) ? steppermotorthread.setPeriod(t) : steppermotorthread.setPeriod((int) (1.f / max_vel));
			return false;
		}
	}

	return false;
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
	float w;
	PROTECT_WITH_SEMAPHORE(sem) w = this->w_Mockup;
	return w;
}

float ArmController::getArmExtention()
{
	return steppermotorthread.getStepCounter() * STEP2LENGTH;
}


ArmController armController;
