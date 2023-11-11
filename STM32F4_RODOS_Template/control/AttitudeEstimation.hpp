#ifndef FLOATSAT_CONTROL_ATTITUDEESTIMATION_HPP_
#define FLOATSAT_CONTROL_ATTITUDEESTIMATION_HPP_

#include "rodos.h"
#include "matlib.h"


class QEKF
{

public:

	QEKF();

	// @brief Calculation of initial orientation
	// @param mag -> magneticfieldstrength vector in gauss
	// @param accel -> linearacceleration vector in g
	void init(Vector3D mag, Vector3D accel);

	// @brief Orientation estimation
	// @param gyro -> angluarvelocity vector in rad/s
	// @param mag -> magneticfieldstrength vector in gauss
	// @param accel -> linearacceleration vector in g
	// @return normalized quaternion
	Quaternion estimate(Vector3D gyro, Vector3D mag, Vector3D accel);

private:

	// @brief Propagation step using only gyro
	// @param gyro -> angluarvelocity vector in rad/s
	void propagate(Vector3D gyro);

	// @brief Update step using magnetometer and accelerometer
	// @param mag -> magneticfieldstrength vector in gauss
	// @param accel -> linearacceleration vector in g
	void update(Vector3D mag, Vector3D accel);

	// TODO: define all Matrices/vectors used
};


extern QEKF qekf;

#endif