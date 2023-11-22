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
		gyroCalib.bias.x = 0.012;
		gyroCalib.bias.y = 0.019;
		gyroCalib.bias.z = 0.022;
		gyroCalib.scale = Matrix3D();
		gyroCalib.scale.r[0][0] = 1.0;
		gyroCalib.scale.r[1][1] = 1.0;
		gyroCalib.scale.r[2][2] = 1.0;

		imu.setGyroCalib(gyroCalib);

		IMUCalib accelCalib;
		accelCalib.bias.x = 0.010;
		accelCalib.bias.y = 0.038;
		accelCalib.bias.z = 0.010;
		accelCalib.scale = Matrix3D();
		accelCalib.scale = Matrix3D();
		accelCalib.scale.r[0][0] = 1.0;
		accelCalib.scale.r[1][1] = 1.0;
		accelCalib.scale.r[2][2] = 1.0;
		imu.setAccelCalib(accelCalib);

		imu.setMagCalibMin(Vector3D(-0.708, -0.350, -0.565) * 4);
		imu.setMagCalibMax(Vector3D(0.424, 0.772, 0.586));

		imu.calibrateData();
		imuRawData = imu.getData();
		// imuRawData = imu.getDataRaw();
		IMUDataTopic.publish(imuRawData);

		if (!qekf.is_initialized) qekf.init(imuRawData.data);
		else AttitudeDataTopic.publish(qekf.estimate(imuRawData));
/*
		if(cnt < 200)
		{
			sum_gyro_x[cnt] = imuRawData.data.angularVelocity.x;
			sum_gyro_y[cnt] = imuRawData.data.angularVelocity.y;
			sum_gyro_z[cnt] = imuRawData.data.angularVelocity.z;

			sum_accel_x[cnt] = imuRawData.data.acceleration.x;
			sum_accel_y[cnt] = imuRawData.data.acceleration.y;
			sum_accel_z[cnt] = imuRawData.data.acceleration.z;

			sum_mag_x[cnt] = atan2(imuRawData.data.magneticField.y, imuRawData.data.magneticField.x);;

			cnt++;
		}
		else
		{
			float sum_gx = 0; float sum_gy = 0; float sum_gz = 0;
			float sum_ax = 0; float sum_ay = 0; float sum_az = 0;
			float sum_mx = 0;

			for(int i = 0; i < 200; i++)
			{
				sum_gx += sum_gyro_x[i];	sum_gy += sum_gyro_y[i];	sum_gz += sum_gyro_z[i];
				sum_ax += sum_accel_x[i];	sum_ay += sum_accel_y[i];	sum_az += sum_accel_z[i];
				sum_mx += sum_mag_x[i];
			}

			sum_gx /= (200.);	sum_gy /= (200.); sum_gz /= (200.);
			sum_ax /= (200.);	sum_ay /= (200.); sum_az /= (200.);
			sum_mx /= (200.);

			float gx = 0; float gy = 0; float gz = 0;
			float ax = 0; float ay = 0; float az = 0;
			float mx = 0;

			for(int j = 0; j < 200; j++)
			{
				gx += pow(sum_gyro_x[j] - sum_gx, 2);
				gy += pow(sum_gyro_y[j] - sum_gy, 2);
				gz += pow(sum_gyro_z[j] - sum_gz, 2);

				ax += pow(sum_accel_x[j] - sum_ax, 2);
				ay += pow(sum_accel_y[j] - sum_ay, 2);
				az += pow(sum_accel_z[j] - sum_az, 2);

				mx += pow(sum_mag_x[j] - sum_mx, 2);
			}

			gx /= (200.-1.);	gy /= (200.-1.); gz /= (200.-1.);
			ax /= (200.-1.);	ay /= (200.-1.); az /= (200.-1.);
			mx /= (200.-1.);

		}
*/




		suspendCallerUntil(NOW() + period * MILLISECONDS);
	}
}


SensorThread sensorthread;
