#include "IMUCalibration.hpp"
#include "rodos.h"
#include "matlib.h"

void CalibrationIMU::config(int gyro_maxsamples, int accel_maxsamples, int mag_maxsamples)
{
	this->gyro_maxsamples = gyro_maxsamples;
	this->accel_maxsamples = accel_maxsamples;
	this->mag_maxsamples = mag_maxsamples;
}

bool CalibrationIMU::calibrateGyro(TimestampedData<IMUData>& imurawdata)
{
	if (gyro_samples <= gyro_maxsamples)
	{
		gyro_sum = gyro_sum + imurawdata.data.angularVelocity;
		gyro_samples++;
		return false;
	}
	else
	{
		IMUCalib gyro_calib;
		gyro_calib.bias = gyro_sum / gyro_maxsamples;
		gyro_calib.scale = Matrix3D_F();
		gyro_calib.scale.r[0][0] = 1.0;
		gyro_calib.scale.r[1][1] = 1.0;
		gyro_calib.scale.r[2][2] = 1.0;
		imu.setGyroCalib(gyro_calib);

		gyro_sum = Vector3D_F(0, 0, 0);
		gyro_samples = 0;
		return true;
	}
}

bool CalibrationIMU::calibrateAccel(TimestampedData<IMUData>& imurawdata)
{
	if (accel_samples <= accel_maxsamples)
	{
		accel_sum = accel_sum + imurawdata.data.acceleration + Vector3D_F(0, 0, 1);
		accel_samples++;
		return false;
	}
	else
	{
		IMUCalib accel_calib;
		accel_calib.bias = accel_sum / accel_maxsamples;
		accel_calib.scale = Matrix3D_F();
		accel_calib.scale.r[0][0] = 1.0;
		accel_calib.scale.r[1][1] = 1.0;
		accel_calib.scale.r[2][2] = 1.0;
		imu.setAccelCalib(accel_calib);

		accel_sum = Vector3D_F(0, 0, 0);
		accel_samples = 0;
		return true;
	}
}

bool CalibrationIMU::calibrateMag(TimestampedData<IMUData>& imurawdata)
{
	if (mag_samples <= mag_maxsamples)
	{
		if (imurawdata.data.magneticField.x < mag_minx) mag_minx = imurawdata.data.magneticField.x;
		if (imurawdata.data.magneticField.x > mag_maxx) mag_maxx = imurawdata.data.magneticField.x;
		if (imurawdata.data.magneticField.y < mag_miny) mag_miny = imurawdata.data.magneticField.y;
		if (imurawdata.data.magneticField.y > mag_maxy) mag_maxy = imurawdata.data.magneticField.y;
		mag_samples++;
		return false;
	}
	else
	{
		IMUCalib mag_calib;
		mag_calib.bias.x = (mag_maxx + mag_minx)/2.f;
		mag_calib.bias.y = (mag_maxy + mag_miny)/2.f;
		mag_calib.bias.z = 0.0f;
		mag_calib.scale = Matrix3D_F();
		mag_calib.scale.r[0][0] = 1.0;
		mag_calib.scale.r[1][1] = 1.0;
		mag_calib.scale.r[2][2] = 1.0;
		imu.setMagCalib(mag_calib);

		//PRINTF("x = %f,y = %f\n", mag_calib.bias.x, mag_calib.bias.y);

		mag_minx = mag_miny =  42000.0f;
		mag_maxx = mag_maxy = -42000.0f;
		mag_samples = 0;
		return true;
	}
}


CalibrationIMU imucalib;
