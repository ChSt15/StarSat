#include "Camera.hpp"

#define CameraTopicId 400


Topic<TelemetryCamera> cameraDataTopic(CameraTopicId, "Camera Topic");
Topic<bool> cameraPwrCmdTopic(401, "Camera Power Command");
Topic<bool> cameraShutdownTopic(403, "Camera Shutdown Command");

Topic<float> cameraTest(402, "OrpeTesting");

// everthing in mm
Matrix3D_F Camera2Arm_Rot;
Vector3D_F Camera2Arm_Trans;

Vector3D_F Camera2Arm(Vector3D_F Vec_C)
{
	return Vec_C.matVecMult(Camera2Arm_Rot) + Camera2Arm_Trans;
}

Matrix3D_F Camera2Arm(Matrix3D_F orientation_C)
{
	return orientation_C.mMult(Camera2Arm_Rot);
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
	return atan2f(Mockup_A.x, Mockup_A.z); // Z is outwards, x is to the right and y is downwards. Therfore atan2f(px, pz) is the yaw to the mockup
}

float CameraData::getYawofMockup()
{
	Matrix3D_F orientation_C = AngleAxis_F(sqrtf(telemetryCamera.rx * telemetryCamera.rx + telemetryCamera.ry * telemetryCamera.ry + telemetryCamera.rz * telemetryCamera.rz), telemetryCamera.rx, telemetryCamera.ry, telemetryCamera.rz).toMatrix3D();
	Matrix3D_F orientation_A = Camera2Arm(orientation_C);
	return YPR_F(orientation_A).yaw;
}

bool CameraData::validFrame()
{
	bool valid = (this->telemetryCamera.MeasurmentCnt == this->last_frame + 1);
	this->last_frame = this->telemetryCamera.MeasurmentCnt;
	return valid;
}

