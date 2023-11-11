#ifndef FLOATSAT_CONTROL_ATTITUDEESTIMATION_HPP_
#define FLOATSAT_CONTROL_ATTITUDEESTIMATION_HPP_

#include "rodos.h"
#include "matlib.h"

#include "../timestamp.hpp"
#include "../hardware/imu.hpp"

struct Attitude_Data 
{
	Quaternion attitude;
	Vector3D angularVelocity;
};


class QEKF
{
private:

	// TODO: define all Matrices/vectors used

public:


	QEKF();

	// @brief Calculation of initial orientation
	// @param imudata -> IMU data struct defined in imu.hpp (angularVelocity [rad/s], magneticField [gauss], acceleration [g])
	void init(const IMUData& imudata);

	// @brief Orientation estimation
	// @param imudata -> IMU data struct defined in imu.hpp with timestamp of measurement.
	// @return Attitude data defined in AttitudeEstimation.hpp with timestamp of estimation.
	const TimestampedData<Attitude_Data>& estimate(const TimestampedData<IMUData>& imudata);

private:

	// @brief Propagation step using only gyro
	// @param gyro -> angluarvelocity vector in rad/s
	void propagate(Vector3D gyro);

	// @brief Update step using magnetometer and accelerometer
	// @param mag -> magneticfieldstrength vector in gauss
	// @param accel -> linearacceleration vector in g
	void update(Vector3D mag, Vector3D accel);

};


extern QEKF qekf;

#endif