#ifndef FLOATSAT_CONTROL_EKF_HPP_
#define FLOATSAT_CONTROL_EKF_HPP_

/**
 * Include the headers ordered my how close they relate to this file.
*/

#include "rodos.h"
#include "matlib.h"

#include "../timestamp.hpp"
#include "../hardware/imu.hpp"


/**
 * @brief Struct to contain the data from the IMU
 */
struct EKFAttitudeData {
    Quaternion attitude;
    Vector3D angularVelocity;
};


/// @brief Definition of the global EKF attitude data topic.
extern Topic<TimestampedData<EKFAttitudeData>> ekfAttitudeDataTopic;


/**
 * @brief The EKFAttitudeEstimation class to contain all logic for initialising the EKF and prosessing data from the IMU.
 */
class EKFAttitudeEstimation : public Thread {
private:


public:

	void init() override;

	void run() override;


};


/**
 * @brief The global IMU object. To be used by other applications.
 * @note This is defined in imu.cpp
 */
extern EKFAttitudeEstimation ekfAttitudeEstimation;


#endif
