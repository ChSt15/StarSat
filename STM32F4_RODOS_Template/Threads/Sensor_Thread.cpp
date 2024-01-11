#include "Sensor_Thread.hpp"

#include "rodos.h"
#include "math.h"

#include "../Threads/Control_Thread.hpp"

#include "../control/AttitudeEstimation.hpp"
#include "../hardware/imu.hpp"
#include "../timestamp.hpp"
#include "../control/CalibrationIMU.hpp"
#include "../hardware/ReactionwheelEncoder.hpp"

HAL_GPIO ledred(GPIO_062);

void SensorThread::init()
{
	ledred.init(true, 1, 0);
}

void SensorThread::run()
{
	float last_t = SECONDS_NOW();
	int cnt = 0;

	// Config
	using namespace config;
	{
		// Thread 
		this->period = sensor_thread_period;
		if (!sensor_thread_enable) suspendCallerUntil(END_OF_TIME);

		//IMU 
		imu.initialization();

		//Encoder
		encoder.Init();

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


		// IMU
		IMUDataTopic.publish(imu.readData());

		// Atitude estimation
		AttitudeDataTopic.publish(qekf.estimate(imu.getData()));

		// Encoder
		EncoderDataTopic.publish(encoder.getSpeed());

		controlthread.resumeAndYield();

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
		/*
		if (cnt % 10 == 0) PRINTF("%f\n", SECONDS_NOW() - last_t);
		last_t = SECONDS_NOW();
		cnt++;
		*/
		ledred.setPins(~ledred.readPins());
		suspendCallerUntil(NOW() + period * MILLISECONDS);
	}
}


SensorThread sensorthread;
