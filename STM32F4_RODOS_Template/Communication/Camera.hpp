#ifndef FLOATSAT_COMMUNICATION_CAMERA_HPP_
#define FLOATSAT_COMMUNICATION_CAMERA_HPP_

#include "rodos.h"


struct TelemetryCamera
{
    float rx, ry, rz;       // [Rodriguez Rot]
    float px, py, pz;       // [mm]
    uint32_t MeasurmentCnt;
};


extern Topic<TelemetryCamera> cameraDataTopic;


#endif
