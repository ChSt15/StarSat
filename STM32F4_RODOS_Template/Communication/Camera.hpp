#ifndef FLOATSAT_COMMUNICATION_CAMERA_HPP_
#define FLOATSAT_COMMUNICATION_CAMERA_HPP_

#include "rodos.h"
#include "matlib.h"
#include <math.h>


struct TelemetryCamera
{
    float rx, ry, rz;       // [Rodriguez Rot]
    float px, py, pz;       // [mm]
    uint32_t MeasurmentCnt;


    float getDistance();
    float getYawtoMockup();
    float getYawofMockup();
};


extern Topic<TelemetryCamera> cameraDataTopic;


#endif
