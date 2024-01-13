#include "DockingThread.hpp"


void DockingThread::init()
{
}

void DockingThread::run()
{
	// Config
	using namespace config;
	{
		// Thread
		this->period = control_thread_period;
		if (!control_thread_enable) suspendCallerUntil(END_OF_TIME);

		// ArmController
		armController.config(max_vel, min_vel, max_accel, deccel_margin, steps2mm);
	}

	while (true)
	{
		switch (getMode())
		{
		case Idle:
			armController.Stop();
			break;

		/* ---------------------------- Calib ---------------------------- */
		case: Calib_Arm:
			if (!armController.Calibrate()) break;

			setMode(Idle);
			break;

		/* ---------------------------- Mission ----------------------------- */
		case Mission_Locate:
			if (!CameraDataReceiver.validFrame(frameCnt)) break;

			setMode(Mission_Point);
			break;

		case Mission_Dock_initial:
			if (!armController.InitialExtension(CameraDataReceiver))
			{
				armController.CalcAngularVelocity(CameraDataReceiver);
				break;
			}

			setMode(Mission_Dock_final);
			break;

		case Mission_Dock_final:
			if (!armController.FinalExtension(CameraDataReceiver)) break;

			setMode(Idle);
			break;

		default:
			break;
		}

		suspendCallerUntil(NOW() + period * MILLISECONDS);
	}
}


DockingThread dockingThread;