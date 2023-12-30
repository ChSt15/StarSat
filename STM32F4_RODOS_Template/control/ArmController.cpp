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
		this->distance = 1600 / 0.9f;

		steppermotorthread.setDirection(true);
		steppermotorthread.setPeriod((int) (1.f/min_vel * 1000.f * 1000.f));
		steppermotorthread.setStepsToDo((int)(0.9f * distance / STEP2LENGTH));
		steppermotorthread.resume();
		last_t = SECONDS_NOW();
		moving = true;
		deccel = false;

		return false;
	}
	else
	{
		// Check if done
		if (steppermotorthread.getStatus())
		{
			moving = false;
			deccel = false;
			return true;
		}

		dt = SECONDS_NOW() - last_t;
		last_t = SECONDS_NOW();
		//PRINTF("%f\n", dt);

		// Avoid multiple accesses
		int period = steppermotorthread.getPeriod();

		// Decelerate to vel_min
		float deccel_time = ((1.f / period * 1000.f * 1000.f) - min_vel) / max_accel;
		float deccel_distance = -0.5 * max_accel * pow(deccel_time, 2) + (1.f / period * 1000.f * 1000.f) * deccel_time;
		if (deccel || (steppermotorthread.getStepsToDo() - DECCELMARGIN) < deccel_distance)
		{
			float t = (1.f / (1.f / period * 1000.f * 1000.f - max_accel * dt));
			(t < 1.f / min_vel && t > 0) ? steppermotorthread.setPeriod(t * 1000.f * 1000.f) : steppermotorthread.setPeriod((int) (1.f / min_vel * 1000.f * 1000.f));
			deccel = true;
			return false;
		}

		// Accelerate to vel_max
		if (period > 1.f/max_vel * 1000 * 1000)
		{
			float t = (1.f / (1.f / period * 1000.f * 1000.f + max_accel * dt));
			(t > 1.f / max_vel) ? steppermotorthread.setPeriod(t * 1000.f * 1000.f) : steppermotorthread.setPeriod((int) (1.f / max_vel * 1000.f * 1000.f));
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
