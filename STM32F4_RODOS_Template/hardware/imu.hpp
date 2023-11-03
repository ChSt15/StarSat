#ifndef FLOATSAT_HARDWARE_IMU_HPP_
#define FLOATSAT_HARDWARE_IMU_HPP_

/**
 * Include the headers ordered my how close they relate to this file.
*/

#include "rodos.h"
#include "matlib.h"

#include "../timestamp.hpp"


/**
 * @brief Struct to contain the data from the IMU
 */
struct IMUData {
    Vector3D acceleration;
    Vector3D angularVelocity;
    Vector3D magneticField;
};


/// @brief definition of the global IMU data topic.
extern Topic<TimestampedData<IMUData>> imuDataTopic;


/**
 * @brief The IMU class to contain all logic for initialising the IMU and reading data from it.
 */
class IMU : public Thread {
private:


public:

	void init() override;

	void run() override;


};


/**
 * @brief The global IMU object. To be used by other applications.
 * @note This is defined in imu.cpp
 */
extern IMU imu;


#endif
