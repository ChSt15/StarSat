#include "Sensor_Thread.hpp"

#include "rodos.h"
#include "math.h"

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
	}

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

		ledred.setPins(~ledred.readPins());
		suspendCallerUntil(NOW() + period * MILLISECONDS);
	}
}


//SensorThread sensorthread;
