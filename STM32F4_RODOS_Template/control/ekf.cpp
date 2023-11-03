#include "rodos.h"
#include "matlib.h"

#include "../timestamp.hpp"

#include "ekf.hpp"


/// @brief Definition of the global EKF attitude data topic.
Topic<TimestampedData<EKFAttitudeData>> ekfAttitudeDataTopic(-1, "EKF Data Topic");



void EKFAttitudeEstimation::init() {



}

void EKFAttitudeEstimation::run() {

	while(1) {

		PRINTF("EKF Thread!");

		suspendCallerUntil(NOW() + 1*SECONDS);

	}

}


/**
 * @brief The global EKF estimation object. To be used by other applications.
 * @note This is defined in ekf.cpp
 */
EKFAttitudeEstimation ekfAttitudeEstimation;
