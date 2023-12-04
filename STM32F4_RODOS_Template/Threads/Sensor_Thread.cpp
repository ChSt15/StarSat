#include "Sensor_Thread.hpp"

#include "rodos.h"
#include "math.h"

#include "../control/AttitudeEstimation.hpp"
#include "../hardware/imu.hpp"
#include "../timestamp.hpp"
#include "../control/CalibrationIMU.hpp"
#include "../hardware/ReactionwheelEncoder.hpp"

void SensorThread::init()
{
	imu.initialization();

	IMUCalib gyroCalib;
	gyroCalib.bias.x = 0.011274;
	gyroCalib.bias.y = 0.018649;
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
	magCalib.bias.x = -0.025;//0.068;
	magCalib.bias.y = 0.2625;//0.129;
	magCalib.bias.z = -0.0495;//-0.0945;
	magCalib.scale = Matrix3D();
	magCalib.scale.r[0][0] = 1.0;
	magCalib.scale.r[1][1] = 1.0;
	magCalib.scale.r[2][2] = 1.0;
	imu.setMagCalib(magCalib);
}

void SensorThread::run()
{
	while (true)
	{
		
		imu.readRawData();

		switch (mode)
		{
		case Calib_Gyro:

			if(!imucalib.calibrateGyro(imu.getDataRaw())) break;

			setMode(Idle);
			break;

		case Calib_Accel:

			if (!imucalib.calibrateAccel(imu.getDataRaw())) break;

			setMode(Idle);
			break;

		case Calib_Mag:
			
			if (!imucalib.calibrateMag(imu.getDataRaw())) break;

			setMode(Idle);
			break;

		default:
			break;
		}

		// IMU
		imu.calibrateData();
		IMUDataTopic.publish(imu.getData());

		// Atitude estimation
		if (!qekf.is_initialized) qekf.init(imu.getData().data);
		else AttitudeDataTopic.publish(qekf.estimate(imu.getData()));

		// Encoder
		EncoderDataTopic.publish(encoder.getSpeed());

		suspendCallerUntil(NOW() + period * MILLISECONDS);
	}
}


SensorThread sensorthread;
