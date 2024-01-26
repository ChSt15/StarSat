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

	while (true)
	{
		// Get new Cameradata if availible
		if (cameraBuffer.getOnlyIfNewData(cameraData.telemetryCamera))
        {
            //Print all data from struct
            auto &data = cameraData.telemetryCamera;
            PRINTF("CameraData: %d %d %d \n %d %d %d \n%d %d\n",
                   data.px, data.py, data.pz, data.rx, data.ry, data.rz, data.MeasurmentCnt, data.numLEDs, data.numPoints);
        }

		switch (getMode())
		{
		case Idle:
		case Standby:
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
			if (!armController.InitialExtension(cameraData))
			{
				if (cameraData.validFrame()) armController.CalcAngularVelocity(cameraData);
				break;
			}

			setMode(Mission_Dock_final);
			break;

		case Mission_Dock_final:
			if (!cameraData.validFrame()) break;
			if (!armController.FinalExtension(cameraData)) break;

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
