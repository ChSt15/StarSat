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

    // WARNING: coordinate transformation not implemented
    float getDistance();
    float getYawtoMockup();
    float getYawofMockup();
};

struct CameraData
{
    TelemetryCamera telemetryCamera;
    unsigned int last_frame = 0;

    bool validFrame();
};


extern Topic<TelemetryCamera> cameraDataTopic;

#endif
