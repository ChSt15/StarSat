#include "DockingThread.hpp"


// Subscriber for cameradata
static CommBuffer<TelemetryCamera> cameraBuffer;
static Subscriber cameraSubsciber(cameraDataTopic, cameraBuffer, "Docking Thread");

HAL_GPIO ledblue(GPIO_063);


void DockingThread::init()
{
	ledblue.init(true, 1, 0);
}

void DockingThread::run()
{
	// Config
	using namespace config;
	{
		// Thread
		this->period = docking_thread_period;
		if (!docking_thread_enable) suspendCallerUntil(END_OF_TIME);

		// ArmController
		armController.config(max_vel, min_vel, max_accel, deccel_margin, steps2mm);
	}

	while (true)
	{
		// Get new Cameradata if availible
		cameraBuffer.getOnlyIfNewData(cameraData.telemetryCamera);

		switch (getMode())
		{
		case Idle:
			armController.Stop();
			break;

		/* ---------------------------- Calib ---------------------------- */
		case Calib_Arm:
			if (!armController.Calibrate()) break;

			setMode(Idle);
			break;

		/* ---------------------------- Mission ----------------------------- */
		case Mission_Locate:
			if (!cameraData.validFrame()) break;

			setMode(Mission_Point);
			break;

		case Mission_Dock_initial:
			if (!armController.InitialExtension(cameraData.telemetryCamera))
			{
				if (cameraData.validFrame()) armController.CalcAngularVelocity(cameraData.telemetryCamera);
				break;
			}

			setMode(Mission_Dock_final);
			break;

		case Mission_Dock_final:
			if (!cameraData.validFrame()) break;
			if (!armController.FinalExtension(cameraData.telemetryCamera)) break;

			setMode(Idle);
			break;

		default:
			break;
		}

		ledblue.setPins(~ledblue.readPins());
		suspendCallerUntil(NOW() + period * MILLISECONDS);
	}
}


DockingThread dockingThread;
