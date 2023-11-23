#include "Sensor_Thread.hpp"

#include "rodos.h"
#include "math.h"

#include "../control/AttitudeEstimation.hpp"
#include "../hardware/imu.hpp"
#include "../timestamp.hpp"


void SensorThread::init()
{
	imu.initialization();
}

void SensorThread::run()
{
/*
	float sum_gyro_x[200];
	float sum_gyro_y[200];
	float sum_gyro_z[200];
	float sum_accel_x[200];
	float sum_accel_y[200];
	float sum_accel_z[200];
	float sum_mag_x[200];
	int cnt = 0;
*/
	while (true)
	{
		imu.readRawData();

		IMUCalib gyroCalib;
		gyroCalib.bias.x =  0.011274;
		gyroCalib.bias.y =  0.018649;
		gyroCalib.bias.z = -0.021776;
		gyroCalib.scale = Matrix3D();
		gyroCalib.scale.r[0][0] = 1.0;
		gyroCalib.scale.r[1][1] = 1.0;
		gyroCalib.scale.r[2][2] = 1.0;

		imu.setGyroCalib(gyroCalib);

		IMUCalib accelCalib;
		accelCalib.bias.x = -0.025037;
		accelCalib.bias.y = -0.0051067;
		accelCalib.bias.z = -0.0089848;
		accelCalib.scale = Matrix3D();
		accelCalib.scale.r[0][0] = 1.0;
		accelCalib.scale.r[1][1] = 1.0;
		accelCalib.scale.r[2][2] = 1.0;
		imu.setAccelCalib(accelCalib);

		IMUCalib magCalib;
		magCalib.bias.x = 0.068;
		magCalib.bias.y = 0.129;
		magCalib.bias.z = -0.0945;
		magCalib.scale = Matrix3D();
		magCalib.scale.r[0][0] = 1.0;
		magCalib.scale.r[1][1] = 1.0;
		magCalib.scale.r[2][2] = 1.0;
		imu.setMagCalib(magCalib);

		imu.calibrateData();
		imuRawData = imu.getData();
		IMUDataTopic.publish(imuRawData);

		if (!qekf.is_initialized) qekf.init(imuRawData.data);
		else AttitudeDataTopic.publish(qekf.estimate(imuRawData));

		suspendCallerUntil(NOW() + period * MILLISECONDS);
	}
}


SensorThread sensorthread;
