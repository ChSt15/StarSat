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

    uint8_t numLEDs;

    uint16_t numPoints;
    
} __attribute__ ((packed));

struct CameraData
{
    TelemetryCamera telemetryCamera;
    unsigned int last_frame = 0;

    bool validFrame();
    // WARNING: coordinate transformation not implemented
    float getDistance();
    float getYawtoMockup();
    float getYawofMockup();
};


extern Topic<TelemetryCamera> cameraDataTopic;
// Publishing to this topic will start or stop the camera estimator
extern Topic<bool> cameraPwrCmdTopic;
// Publishing true to this topic will shut down the raspberry pi
extern Topic<bool> cameraShutdownTopic;
// Just for testing
extern Topic<float> cameraTest;

#endif
