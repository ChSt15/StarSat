#include "Sensor_Thread.hpp"

#include "rodos.h"

#include "../control/AttitudeEstimation.hpp"
#include "../hardware/imu.hpp"
#include "../timestamp.hpp"


void SensorThread::init()
{
	
}

void SensorThread::run()
{

	while (true)
	{

		TimestampedData<IMUData> imudata_dummy;
		imudata_dummy.data.angularVelocity = Vector3D(0.0001, -0.0001, 0.001);
		imudata_dummy.data.acceleration = Vector3D(0, 0, -1);
		imudata_dummy.data.magneticField = Vector3D(0, 1, 0);
		imudata_dummy.data.temperature = 42;
		IMUDataTopic.publish(imudata_dummy);

		if (!qekf.is_initialized) qekf.init(imudata_dummy.data);
		else AttitudeDataTopic.publish(qekf.estimate(imudata_dummy));

		suspendCallerUntil(NOW() + period * MILLISECONDS);
	}
}


SensorThread sensorthread;