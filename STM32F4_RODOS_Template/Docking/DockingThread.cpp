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

    bool cameraState = false;

	while (true)
	{   

        cameraPwrCmdTopic.publishConst(cameraState);
        //setMode(Mission_Locate);
		// Get new Cameradata if availible
		if (cameraBuffer.getOnlyIfNewData(cameraData.telemetryCamera))
        {
            //Print all data from struct
            auto &data = cameraData.telemetryCamera;
            PRINTF("CameraData: %f %f %f \n %f %f %f \n%d %d\n",
                   data.px, data.py, data.pz, data.rx, data.ry, data.rz, data.MeasurmentCnt, data.numLEDs, data.numPoints);
        }

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

			setMode(Idle);
			break;

		/* ---------------------------- Mission ----------------------------- */
		case Mission_Locate:
			cameraState = true;
            PRINTF("Current frame %d, last frame %d\n", cameraData.telemetryCamera.MeasurmentCnt, cameraData.last_frame);
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

			setMode(Idle);
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
