#include "ArmControl.hpp"


// Subscriber for stepper status
static CommBuffer<StepperStatus> statusBuffer;
static Subscriber statusSubsciber(stepperStatusTopic, statusBuffer, "Arm Control");


void ArmController::config(int max_vel, int min_vel, int dock_vel, int max_accel, int deccel_margin, float steps2mm)
{
	this->max_vel = max_vel;
	this->min_vel = min_vel;
    this->dock_vel = dock_vel;
	this->max_accel = max_accel;
	this->deccel_margin = deccel_margin;
	this->steps2mm = steps2mm;
}

bool ArmController::InitialExtension(CameraData& camera)
{
	// update Mockup data
	if (camera.validFrame() && updateTelemetryMockup(camera))
	{
		instructions.stepTarget = (int)((telemetry.mockupDistance - 10) / steps2mm);
	}
    // skip if no measurment is available and no distance is set
    else if (!moving) return false;

	// Get new status if available
	statusBuffer.getOnlyIfNewData(status);

	if (!moving)
	{
		instructions.period = (int)(1.f / min_vel * 1000.f * 1000.f);
		stepperInstructionsTopic.publish(instructions);
		steppermotorthread.resume();
		updateTelemetryArm();

		last_time_dock = SECONDS_NOW();
		moving = true;
		deccel = false;

		return false;
	}
	else
	{
		// Check if done
		if (status.status_execution)
		{
			instructions.period = 0;
			stepperInstructionsTopic.publish(instructions);
			updateTelemetryArm();
			moving = false;
			deccel = false;
			return true;
		}

		float dt = SECONDS_NOW() - last_time_dock;
		last_time_dock = SECONDS_NOW();

		int period = instructions.period;

		// Decelerate to vel_min
		float deccel_time = ((1.f / period * 1000.f * 1000.f) - min_vel) / max_accel;
		float deccel_distance = -0.5 * max_accel * pow(deccel_time, 2) + (1.f / period * 1000.f * 1000.f) * deccel_time;
		if (deccel || (instructions.stepTarget - status.stepCounter - deccel_margin) < deccel_distance)
		{
			float t = (1.f / (1.f / period * 1000.f * 1000.f - max_accel * dt));
			(t < 1.f / min_vel && t > 0) ? instructions.period = (int) (t * 1000.f * 1000.f) : instructions.period = (int) (1.f / min_vel * 1000.f * 1000.f);
			stepperInstructionsTopic.publish(instructions);
			updateTelemetryArm();
			deccel = true;
			return false;
		}

		// Accelerate to vel_max
		if (period > 1.f/max_vel * 1000 * 1000)
		{
			float t = (1.f / (1.f / period * 1000.f * 1000.f + max_accel * dt));
			(t > 1.f / max_vel) ? instructions.period = (int) (t * 1000.f * 1000.f) : instructions.period = (int) (1.f / max_vel * 1000.f * 1000.f);
			stepperInstructionsTopic.publish(instructions);
			updateTelemetryArm();
			return false;
		}
	}
	return false;
}

bool ArmController::FinalExtension(CameraData& camera)
{
	if (moving)
	{
		// Get new status if available
		statusBuffer.getOnlyIfNewData(status);

		// Check if done
		if (status.status_execution)
		{
			instructions.period = 0;
			stepperInstructionsTopic.publish(instructions);
			updateTelemetryArm();

            reset();
			moving = false;
			return true;
		}

		updateTelemetryArm();
		return false;
	}
	
	if (camera.validFrame() && updateTelemetryMockup(camera))
	{
		float time_to_target_arm = (telemetry.mockupDistance / steps2mm - status.stepCounter ) / this->dock_vel;
		float time_to_target_mockup;
		float yaw = camera.getYawofMockup();
		float w = telemetry.mockupAngularvelocity;

		if (w > 0)
		{
			if (yaw > 0) yaw -= 2 * M_PI;
			time_to_target_mockup = -yaw / w;
		}
		else
		{
			if (yaw < 0) yaw += 2 * M_PI;
			time_to_target_mockup = yaw / -w;
		}

		if (time_to_target_mockup - time_to_target_arm < 1)
		{
			instructions.stepTarget = (int)(telemetry.mockupDistance / steps2mm);
			instructions.period = (int)(1.f / dock_vel * 1000.f * 1000.f);
			stepperInstructionsTopic.publish(instructions);
			updateTelemetryArm();

			Thread::suspendCallerUntil(NOW() + (time_to_target_mockup - time_to_target_arm) * SECONDS);
			steppermotorthread.resume();
			moving = true;
		}
	}
	return false;
}

void ArmController::updateTelemetryArm()
{
	telemetry.armVelocity = 1.f / instructions.period * 1000.f * 1000.f;
	telemetry.armExtention = status.stepCounter * steps2mm;

	dockingTelemetryTopic.publish(telemetry);
}

bool ArmController::updateTelemetryMockup(CameraData& camera)
{
	float time = SECONDS_NOW();
	float yaw = camera.getYawofMockup();

	if (valid_LastYaw)
	{
		float dyaw = (yaw - last_yaw);
		float dt = time - last_time_w;
		while (dyaw > M_PI) dyaw -= 2 * M_PI;
		while (dyaw < -M_PI) dyaw += 2 * M_PI;
		if (!valid_CameraData)
		{
			telemetry.mockupAngularvelocity = dyaw / dt;
            telemetry.mockupDistance = camera.getDistance();
            valid_CameraData = true;
		}                                                                                                                           
		else
		{
			float w = dyaw / dt;
			telemetry.mockupAngularvelocity = telemetry.mockupAngularvelocity * 0.5 + 0.5 * w;
            telemetry.mockupDistance = telemetry.mockupDistance * 0.5 + camera.getDistance() * 0.5;
		}
	}
    else
    {
        valid_LastYaw = true;
    }

	last_time_w = time;
	last_yaw = yaw;

	dockingTelemetryTopic.publish(telemetry);

    //PRINTF("%d, %d\n", valid_CameraData, valid_LastYaw);

    return valid_CameraData;
}


void ArmController::reset()
{
    valid_CameraData = false;
    valid_LastYaw = false;
}

void ArmController::Stop()
{
	instructions.period = 0;
	stepperInstructionsTopic.publish(instructions);
	moving = false;
}

bool ArmController::Calibrate()
{
	Stop();
	return steppermotorthread.calibrate();
}


ArmController armController;