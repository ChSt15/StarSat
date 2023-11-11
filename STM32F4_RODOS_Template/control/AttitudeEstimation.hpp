#ifndef FLOATSAT_CONTROL_ATTITUDEESTIMATION_HPP_
#define FLOATSAT_CONTROL_ATTITUDEESTIMATION_HPP_

#include "rodos.h"
#include "matlib.h"


class QEKF
{

public:

	QEKF(float std_gyro, float std_mag, float std_accel);
	void init(Vector3D gyro, Vector3D mag, Vector3D accel);
	Quaternion estimate(Vector3D gyro, Vector3D mag, Vector3D accel);

private:

	void propagate(Vector3D gyro);
	void update(Vector3D mag, Vector3D accel);

	//TODO: define all Matrices/vectors used
};


extern QEKF qekf;

#endif