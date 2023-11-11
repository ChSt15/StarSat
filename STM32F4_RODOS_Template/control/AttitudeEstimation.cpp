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