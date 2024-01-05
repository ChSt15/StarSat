#include "ArmController.hpp"

#define STEP2LENGTH 1		// [mm]


// Camera topic subscriber setup
static CommBuffer<TelemetryCamera> CameraDataBuffer;
static Subscriber CameraDataSubsciber(cameraDataTopic, CameraDataBuffer, "Arm Controller Class");
TelemetryCamera CameraDataReceiver;


void ArmController::config(int max_vel, int min_vel, int max_accel, int deccel_margin)
{
	this->max_vel = max_vel;
	this->min_vel = min_vel;
	this->max_accel = max_accel;
	this->deccel_margin = deccel_margin;
}

float last_t;

float last_time;
float last_yaw;

bool ArmController::InitialExtension()
{
	if (!moving)
	{
		// TODO Coordinate transfrom
		/*
		CameraDataBuffer.get(CameraDataReceiver);
		this->distance = CameraDataReceiver.getDistance();
		*/

		// just for test
		this->distance = 1600 / 0.9f;

		steppermotorthread.setDirection(true);
		steppermotorthread.setPeriod((int) (1.f/min_vel * 1000.f * 1000.f));
		steppermotorthread.setStepsToDo((int)(0.95f * distance / STEP2LENGTH));
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

		float dt = SECONDS_NOW() - last_t;
		last_t = SECONDS_NOW();
		//PRINTF("%f\n", dt);

		// Avoid multiple accesses
		int period = steppermotorthread.getPeriod();

		// Decelerate to vel_min
		float deccel_time = ((1.f / period * 1000.f * 1000.f) - min_vel) / max_accel;
		float deccel_distance = -0.5 * max_accel * pow(deccel_time, 2) + (1.f / period * 1000.f * 1000.f) * deccel_time;
		if (deccel || (steppermotorthread.getStepsToDo() - deccel_margin) < deccel_distance)
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
	if (moving)
	{
		// Check if done
		if (steppermotorthread.getStatus())
		{
			moving = false;
			return true;
		}
		return false;
	}

	CameraDataBuffer.get(CameraDataReceiver);

	float time_to_target_arm = 0.95f * this->distance / (this->min_vel * STEP2LENGTH);
	// WARNING:
	// where is 0 yaw defined to be ?
	float time_to_target_mockup;
	float yaw = CameraDataReceiver.getYawofMockup();
	float w = getAngularvelocityMockup();
	if (w > 0)
	{
		while (yaw > 0) yaw -= 2 * M_PI;
		time_to_target_mockup = -yaw / w;
	}
	else
	{
		while (yaw < 0) yaw += 2 * M_PI;
		time_to_target_mockup = yaw / w;
	}
	
	// change margin to what works best
	if (time_to_target_arm - time_to_target_mockup < 0.01)
	{
		steppermotorthread.setDirection(true);
		steppermotorthread.setPeriod((int)(1.f / min_vel * 1000.f * 1000.f));
		steppermotorthread.setStepsToDo((int)(0.05 * distance / STEP2LENGTH));
		steppermotorthread.resume();
		moving = true;
	}
	return false;
}


void ArmController::CalcAngularVelocity()
{
	CameraDataBuffer.get(CameraDataReceiver);

	if (lastframe > 0 && CameraDataReceiver.MeasurmentCnt > 0);
	{
		lastframe = CameraDataReceiver.MeasurmentCnt;
		last_time = SECONDS_NOW();
		last_yaw = CameraDataReceiver.getYawofMockup();
		return;
	}

	if (CameraDataReceiver.MeasurmentCnt == lastframe + 1)
	{
		float yaw = CameraDataReceiver.getYawofMockup();
		float time = SECONDS_NOW();
		float w = yaw - last_yaw / (time - last_time);

		PROTECT_WITH_SEMAPHORE(sem)
		{
			if (isnan(this->w_Mockup)) this->w_Mockup = w;
			else this->w_Mockup = (this->w_Mockup + w) / 2.f;
		}

		lastframe = CameraDataReceiver.MeasurmentCnt;
		last_time = time;
		last_yaw = yaw;
	}

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
