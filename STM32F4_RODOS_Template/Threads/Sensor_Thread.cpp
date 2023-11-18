#include "Sensor_Thread.hpp"

#include "rodos.h"

#include "../control/AttitudeEstimation.hpp"
#include "../hardware/imu.hpp"


void SensorThread::init()
{
	
}

void SensorThread::run()
{
	while (true)
	{
		// temporary
		IMUData imudata_dummy;
		imudata_dummy.angularVelocity = Vector3D(1, 1, 1);
		imudata_dummy.acceleration = Vector3D(0, 0, 1);
		imudata_dummy.magneticField = Vector3D(1, 1, 1);
		imudata_dummy.temperature = 42;
		IMUDataTopic.publish(imudata_dummy);

		// temporary
		Attitude_Data attitudedata_dummy;
		attitudedata_dummy.angularVelocity = Vector3D(1, 1, 1);
		attitudedata_dummy.attitude = Quaternion();
		AttitudeDataTopic.publish(attitudedata_dummy);

		suspendCallerUntil(NOW() + period * MILLISECONDS);
	}
}

SensorThread sensorthread;
