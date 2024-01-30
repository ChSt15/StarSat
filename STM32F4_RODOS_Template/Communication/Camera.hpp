#ifndef FLOATSAT_COMMUNICATION_CAMERA_HPP_
#define FLOATSAT_COMMUNICATION_CAMERA_HPP_

#include "rodos.h"
#include "matlib.h"
#include <math.h>


struct TelemetryCamera
{
    float px, py, pz;       // [mm]
    float rx, ry, rz;       // [Rodriguez Rot]
    uint32_t MeasurmentCnt = 0;

    bool valid = false;

    uint8_t numLEDs;

    uint16_t numPoints;
    
};// __attribute__ ((packed));

struct CameraData
{
    TelemetryCamera telemetryCamera;
    unsigned int last_frame = 0;

    //bool valid_ = false;

    bool validFrame();
    float getDistance();
    float getYawtoMockup();
    float getYawofMockup();
};


/**
 * Returns the latest camera data
*/
//CameraData getCameraData();


extern Topic<TelemetryCamera> cameraDataTopic;
// Publishing to this topic will start or stop the camera estimator
extern Topic<bool> cameraPwrCmdTopic;
// Publishing true to this topic will shut down the raspberry pi
extern Topic<bool> cameraShutdownTopic;
// Just for testing
extern Topic<float> cameraTest;

#endif
