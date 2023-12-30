#include "Sensor_Thread.hpp"

#include "rodos.h"
#include "math.h"

#include "../control/AttitudeEstimation.hpp"
#include "../hardware/imu.hpp"
#include "../timestamp.hpp"
#include "../control/CalibrationIMU.hpp"
#include "../hardware/ReactionwheelEncoder.hpp"


HAL_GPIO ledred(GPIO_062);


float t_last;

void SensorThread::init()
{
	ledred.init(true, 1, 0);
}

void SensorThread::run()
{
	imu.initialization();

	// Config
	using namespace config;
	{
		// Thread 
		this->period = sensor_thread_period;
		if (!sensor_thread_enable) suspendCallerUntil(END_OF_TIME);

		// IMU Calibparams
		imu.setGyroCalib(gyroCalib);
		imu.setAccelCalib(accelCalib);
		imu.setMagCalib(magCalib);
		imucalib.config(gyro_maxsamples, accel_maxsamples, mag_maxsamples);

		// QEKF
		qekf.config(sigma_gyro, sigma_accel, sigma_yaw, sigma_gyro_drift);
	}

	while (true)
	{
		imu.readRawData();

		switch (getMode())
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

		ledred.setPins(~ledred.readPins());
		suspendCallerUntil(NOW() + period * MILLISECONDS);
	}
}


SensorThread sensorthread;
