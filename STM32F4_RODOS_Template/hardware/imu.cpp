#include "imu.hpp"

#define LSM9DS1_M_ADDR 0x1E
#define LSM9DS1_XLG_ADDR 0x6B

Topic<TimestampedData<IMUData>> IMUDataTopic(-1, "IMUData");


IMU::IMU(RODOS::I2C_IDX i2c):
	i2c(i2c)
{

}

TimestampedData<IMUData> IMU::getData()
{
	return this->dataCalibrated;
}

TimestampedData<IMUData> IMU::getDataRaw()
{
	return this->dataRaw;
}

void IMU::setGyroCalib(const IMUCalib& calib)
{

}

const IMUCalib& IMU::getGyroCalib()
{
	return this->gyroCalib;
}

void IMU::setAccelCalib(const IMUCalib& calib)
{

}

const IMUCalib& IMU::getAccelCalib()
{
	return this->accelCalib;
}

void IMU::setMagCalib(const IMUCalib& calib)
{

}

const IMUCalib& IMU::getMagCalib()
{
	return this->magCalib;
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
