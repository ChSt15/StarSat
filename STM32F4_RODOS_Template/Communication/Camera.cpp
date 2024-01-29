#include "Camera.hpp"


SyncFifo<TelemetryCamera, 4> CameraDataBuffer;
Subscriber CameraDataSubsciber(cameraDataTopic, CameraDataBuffer, "Control Thread");
TelemetryCamera CameraDataReceiver;

/**
 * To protect the camera data
*/
Semaphore cameraDataSem;

Topic<TelemetryCamera> cameraDataTopic(400, "Camera Topic");
Topic<bool> cameraPwrCmdTopic(401, "Camera Power Command");
Topic<bool> cameraShutdownTopic(403, "Camera Shutdown Command");

Topic<float> cameraTest(402, "OrpeTesting");

// everything in mm
const Matrix3D_F Camera2Arm_Rot = Matrix3D_F(Vector3D_F(1, 0, 0), Vector3D_F(0, 1, 0), Vector3D_F(0, 0, -1))  * Matrix3D_F(AngleAxis_F(grad2Rad(-10), 0, 1, 0)) * Matrix3D_F(AngleAxis_F(grad2Rad(20), 1, 0, 0)) * Matrix3D_F(Vector3D_F(1, 0, 0), Vector3D_F(0, 0, -1), Vector3D_F(0, 1, 0));
const Vector3D_F Camera2Arm_Trans = Vector3D_F(20.5, -127.4, 175) + Vector3D_F(0, -95, -70);

Vector3D_F Camera2Arm(Vector3D_F Vec_C)
{
	return Vec_C.matVecMult(Camera2Arm_Rot) + Camera2Arm_Trans;
}

float CameraData::getDistance()
{
	Vector3D_F Mockup_C(telemetryCamera.px, telemetryCamera.py, telemetryCamera.pz);
	return Camera2Arm(Mockup_C).getLen();
}

float CameraData::getYawtoMockup()
{
	Vector3D_F Mockup_C(telemetryCamera.px, telemetryCamera.py, telemetryCamera.pz);
	Vector3D_F Mockup_A = Camera2Arm(Mockup_C);
	return atan2f(Mockup_A.x, Mockup_A.y);
}

float CameraData::getYawofMockup()
{
	Matrix3D_F Camera2Mockup = AngleAxis_F(sqrtf(telemetryCamera.rx * telemetryCamera.rx + telemetryCamera.ry * telemetryCamera.ry + telemetryCamera.rz * telemetryCamera.rz), telemetryCamera.rx, telemetryCamera.ry, telemetryCamera.rz).toMatrix3D();
	Matrix3D_F Arm2Mockup = Camera2Arm_Rot.transpose() * Camera2Mockup;
	return YPR_F(Arm2Mockup).yaw;
}

bool CameraData::validFrame()
{
	return valid_;
}


CameraData getCameraData()
{

    CameraData camera;

    cameraDataSem.enter();

    static TelemetryCamera lastCam;
    static bool isValid = false;

    if (CameraDataBuffer.isFull()) { // Use the older element in fifo as lastCam

        PRINTF("HELLOWORLD\n");

        CameraDataBuffer.get(lastCam);

    }

    TelemetryCamera tCam;
    if (CameraDataBuffer.get(tCam)) { 

        PRINTF("Cnt: %d, %d\n", tCam.MeasurmentCnt, lastCam.MeasurmentCnt);

        if (tCam.MeasurmentCnt == lastCam.MeasurmentCnt + 1) {

            isValid = true;

        } else {

            isValid = false;

        }

        lastCam = tCam;

    }

    camera.telemetryCamera = lastCam;
    camera.valid_ = isValid;

    cameraDataSem.leave();

    return camera;

}

