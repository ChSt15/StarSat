#include "Camera.hpp"

#define CameraTopicId 100


Topic<TelemetryCamera> cameraDataTopic(CameraTopicId, "Camera Topic");

float TelemetryCamera::getDistance()
{
	return sqrtf(this->px * this->px + this->py * this->py + this->pz * this->pz);
}

float TelemetryCamera::getYawtoMockup()
{
	return atan2f(this->py, this->px);
}

float TelemetryCamera::getYawofMockup()
{
	AngleAxis_F orientation = AngleAxis_F(sqrtf(this->rx * this->rx + this->ry * this->ry + this->rz * this->rz), this->rx, this->ry, this->rz);
	return YPR_F(orientation).yaw;
}

bool TelemetryCamera::validFrame(int& last_frame)
{
	bool valid = this->MeasurmentCnt == last_frame + 1;
	last_frame = this->MeasurmentCnt;
	return valid;
}
