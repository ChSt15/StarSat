#include "imu.hpp"

#define LSM9DS1_M_ADDR 0x1E
#define LSM9DS1_XLG_ADDR 0x6B


IMU::IMU()
{

}

IMU_Data IMU::getData()
{
	return this->data;
}

IMU_Calib IMU::getCalib()
{
	return this->calib;
}

void IMU::setCalib(IMU_Calib calib)
{

}

void IMU::Check_I2C_Enable()
{

}

void IMU::Check_WHOAMI()
{

}

void IMU::gyroInit()
{

}

void IMU::gyroRead()
{

}

void IMU::accelInit()
{

}

void IMU::accelRead()
{

}

void IMU::magInit()
{

}

void IMU::magRead()
{

}

void IMU::TempRead()
{

}


IMU imu;
