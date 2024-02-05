#include "Camera.hpp"

#define CameraTopicId 400


Topic<TelemetryCamera> cameraDataTopic(CameraTopicId, "Camera Topic");
Topic<bool> cameraPwrCmdTopic(401, "Camera Power Command");
Topic<bool> cameraShutdownTopic(403, "Camera Shutdown Command");

Topic<float> cameraTest(402, "OrpeTesting");


float CameraData::getDistance()
{
	return sqrtf(telemetryCamera.px * telemetryCamera.px + telemetryCamera.py * telemetryCamera.py + telemetryCamera.pz * telemetryCamera.pz);
}

float CameraData::getYawtoMockup()
{
	return atan2f(telemetryCamera.px, telemetryCamera.pz); // Z is outwards, x is to the right and y is downwards. Therfore atan2f(px, pz) is the yaw to the mockup
}

float CameraData::getYawofMockup()
{
	AngleAxis_F orientation = AngleAxis_F(sqrtf(telemetryCamera.rx * telemetryCamera.rx + telemetryCamera.ry * telemetryCamera.ry + telemetryCamera.rz * telemetryCamera.rz), telemetryCamera.rx, telemetryCamera.ry, telemetryCamera.rz);
	return YPR_F(orientation).yaw;
}

bool CameraData::validFrame()
{
	bool valid = (this->telemetryCamera.MeasurmentCnt == this->last_frame + 1);
	this->last_frame = this->telemetryCamera.MeasurmentCnt;
	return valid;
}
