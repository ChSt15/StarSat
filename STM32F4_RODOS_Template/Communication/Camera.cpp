#include "Camera.hpp"

#define CameraTopicId 100


Topic<TelemetryCamera> cameraDataTopic(CameraTopicId, "Camera Topic");