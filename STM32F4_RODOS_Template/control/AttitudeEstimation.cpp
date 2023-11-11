#include "AttitudeEstimation.hpp"


QEKF::QEKF(float std_gyro, float std_mag, float std_accel)
{

}

void QEKF::init(Vector3D gyro, Vector3D mag, Vector3D accel)
{

}

Quaternion QEKF::estimate(Vector3D gyro, Vector3D mag, Vector3D accel)
{
	// added just so compiler is happy
	return Quaternion();
}

void propagate(Vector3D gyro)
{

}

void update(Vector3D mag, Vector3D accel)
{

}


QEKF qekf;