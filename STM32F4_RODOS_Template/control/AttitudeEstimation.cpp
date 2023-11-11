#include "AttitudeEstimation.hpp"


QEKF::QEKF()
{

}

void QEKF::init(Vector3D mag, Vector3D accel)
{

}

Quaternion QEKF::estimate(Vector3D gyro, Vector3D mag, Vector3D accel)
{
	// added just so compiler is happy
	return Quaternion();
}

void QEKF::propagate(Vector3D gyro)
{

}

void QEKF::update(Vector3D mag, Vector3D accel)
{

}


QEKF qekf;