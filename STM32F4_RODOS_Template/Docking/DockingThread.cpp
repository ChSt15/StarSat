#include "DockingThread.hpp"


HAL_GPIO ledblue(GPIO_063);


void DockingThread::init()
{
	ledblue.init(true, 1, 0);
}

void DockingThread::run()
{
	// Wait for Electrical
	while (getMode() == Electrical_Startup) suspendCallerUntil(NOW() + 200 * MILLISECONDS);

	// Config
	using namespace config;
	{
		// Thread
		this->period = docking_thread_period;
		if (!docking_thread_enable) suspendCallerUntil(END_OF_TIME);

		// ArmController
		armController.config(max_vel, min_vel, max_accel, deccel_margin, steps2mm);
	}

    bool cameraState = false;

	while (true)
	{   

        cameraPwrCmdTopic.publishConst(cameraState);

		// Get new Cameradata if availible
		cameraData.telemetryCamera = getCameraData().telemetryCamera;

		switch (getMode())
		{
		case Idle:
            cameraState = false;
		case Standby:
            cameraState = false;
			armController.Stop();
			break;

		/* ---------------------------- Calib ---------------------------- */
		case Calib_Arm:
            cameraState = false;
			if (!armController.Calibrate()) break;

			setMode(Standby);
			break;

		/* ---------------------------- Mission ----------------------------- */
		case Mission_Locate:
			cameraState = true;
			if (!cameraData.validFrame()) break;
			setMode(Mission_Point);
			break;

		case Mission_Dock_initial:
            cameraState = true;
			if (!armController.InitialExtension(cameraData))
			{
				if (cameraData.validFrame()) armController.CalcAngularVelocity(cameraData);
				break;
			}

			setMode(Mission_Dock_final);
			break;

		case Mission_Dock_final:
            cameraState = true;
			if (!cameraData.validFrame()) break;
			if (!armController.FinalExtension(cameraData)) break;

			setMode(Standby);
			break;

		default:
            cameraState = false;
			break;
		}

		ledblue.setPins(~ledblue.readPins());
		suspendCallerUntil(NOW() + period * MILLISECONDS);
	}
}


DockingThread dockingThread;
