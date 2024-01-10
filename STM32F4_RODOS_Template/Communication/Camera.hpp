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

    bool validFrame(int& last_frame);
};


extern Topic<TelemetryCamera> cameraDataTopic;


#endif
