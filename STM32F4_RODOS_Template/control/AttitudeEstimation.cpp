#include "rodos.h"
#include "matlib.h"

#include "../timestamp.hpp"
#include "../hardware/imu.hpp"

#include "AttitudeEstimation.hpp"


QEKF::QEKF()
{

}

void QEKF::init(const IMUData& imudata)
{
	// Calc roll, pitch from accelerometer
	double roll  = -atan2(imudata.acceleration.x, sqrt(pow(imudata.acceleration.y, 2) + pow(imudata.acceleration.z, 2)));
	double pitch =  atan2(imudata.acceleration.y, sqrt(pow(imudata.acceleration.x, 2) + pow(imudata.acceleration.z, 2)));

	// Calc yaw from magnetometer
	double magx_h = imudata.magneticField.x * cos(pitch) + imudata.magneticField.z * sin(roll);
	double magy_h = imudata.magneticField.x * sin(roll) * sin(pitch) + imudata.magneticField.y * cos(roll) - imudata.magneticField.z * sin(roll) * cos(pitch);
	double yaw = atan2(magy_h, magx_h);

	// Convert to quaternion
	this->X = YPR(yaw, pitch, roll).toQuaternion();
}

const TimestampedData<Attitude_Data>& QEKF::estimate(const TimestampedData<IMUData>& imudata)
{
	
}

void QEKF::propagate(Vector3D gyro)
{

}

void QEKF::update(Vector3D mag, Vector3D accel)
{

}


QEKF qekf;