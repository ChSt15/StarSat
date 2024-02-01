#include "Camera.hpp"


CommBuffer<TelemetryCamera> CameraDataBuffer;
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
//const Matrix3D_F Camera2Arm_Rot = Matrix3D_F(Vector3D_F(1, 0, 0), Vector3D_F(0, 1, 0), Vector3D_F(0, 0, -1))  * Matrix3D_F(AngleAxis_F(grad2Rad(-10), 0, 1, 0)) * Matrix3D_F(AngleAxis_F(grad2Rad(20), 1, 0, 0)) * Matrix3D_F(Vector3D_F(1, 0, 0), Vector3D_F(0, 0, -1), Vector3D_F(0, 1, 0));


const Matrix3D_F Rot_X_minus90deg = Matrix3D_F(Vector3D_F(1, 0, 0), Vector3D_F(0, cos(grad2Rad(-90)), sin(grad2Rad(-90))), Vector3D_F(0, -sin(grad2Rad(-90)), cos(grad2Rad(-90))));

const Matrix3D_F Rot_Y_minus5deg = Matrix3D_F(Vector3D_F(cos(grad2Rad(-90)), 0, -sin(grad2Rad(-90))), Vector3D_F(0,1,0), Vector3D_F(sin(grad2Rad(-90)), 0, cos(grad2Rad(-90))));

const Matrix3D_F Rot_X_minus20deg = Matrix3D_F(Vector3D_F(1, 0, 0), Vector3D_F(0, cos(grad2Rad(-20)), sin(grad2Rad(-20))), Vector3D_F(0, -sin(grad2Rad(-20)), cos(grad2Rad(-20))));

const Matrix3D_F Camera2Arm_Rot = Rot_X_minus90deg * Rot_Y_minus5deg * Rot_X_minus20deg;

const Vector3D_F Camera2Arm_Trans = Vector3D_F(20.5, -128, 157);

const float MockupToDockingPort = 105;

// const Matrix3D_F Camera2Arm_Rot = Matrix3D_F(YPR_F(grad2Rad(-3.75), grad2Rad(2.4), grad2Rad(100.5))).transpose();
// const Vector3D_F Camera2Arm_Trans = Vector3D_F(20.5, -127.4, 175) + Vector3D_F(0, -95, -70);
// const Vector3D_F Camera2Arm_Trans = Vector3D_F(9.5, -141.4, 117) + Vector3D_F(0, -105, -70);
const Vector3D_F Arm2IMU_Trans = Vector3D_F(12, 129, 0);

Vector3D_F Camera2Arm(Vector3D_F Vec_C)
{
	return Vec_C.matVecMult(Camera2Arm_Rot) + Camera2Arm_Trans;
}

float CameraData::getDistance()
{
	Vector3D_F Mockup_C(telemetryCamera.px, telemetryCamera.py, telemetryCamera.pz);
    Vector3D_F Mockup_A = Camera2Arm(Mockup_C);
    float distance = sqrt(Mockup_A.x * Mockup_A.x + Mockup_A.y * Mockup_A.y) - MockupToDockingPort;
	return distance;
}

float CameraData::getYawtoMockup()
{
	Vector3D_F Mockup_C(telemetryCamera.px, telemetryCamera.py, telemetryCamera.pz);
	Vector3D_F Mockup_A = Camera2Arm(Mockup_C);
    Vector3D_F Mockup_IMU = Mockup_A + Arm2IMU_Trans;
	// CHANGED
    // return atan2f(Mockup_IMU.x, Mockup_IMU.y) - grad2Rad(4.f);
    return atan2f(Mockup_IMU.x, Mockup_IMU.y);
}

float CameraData::getYawofMockup()
{
	Matrix3D_F Camera2Mockup = AngleAxis_F(sqrtf(telemetryCamera.rx * telemetryCamera.rx + telemetryCamera.ry * telemetryCamera.ry + telemetryCamera.rz * telemetryCamera.rz), telemetryCamera.rx, telemetryCamera.ry, telemetryCamera.rz).toMatrix3D().transpose();
	Matrix3D_F Arm2Mockup =  Camera2Mockup * Camera2Arm_Rot.transpose();
	return YPR_F(Arm2Mockup).yaw;
}

bool CameraData::validFrame()
{   
    bool res = telemetryCamera.valid && telemetryCamera.MeasurmentCnt != last_frame;
    last_frame = telemetryCamera.MeasurmentCnt;
	return res;
}


CameraData getCameraData()
{
    /*
    TelemetryCamera tCam;


    CameraData camera;

    camera.telemetryCamera

    //cameraDataSem.enter();

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

    //cameraDataSem.leave();

    return camera;*/

}

