#include "rodos.h"
#include "matlib.h"

#include "../timestamp.hpp"
#include "imu.hpp"


/// @brief definition of the global IMU data topic.
Topic<TimestampedData<IMUData>> imuDataTopic(-1, "IMU Topic");

void IMU::init() {

	PRINTF("plswork");
}

void IMU::run() {

	while(1) {

		PRINTF("IMU Thread!");

		suspendCallerUntil(NOW() + 1*SECONDS);

	}

}


/**
 * @brief The global IMU object. To be used by other applications.
 * @note This is defined in imu.cpp
 */
IMU imu;
