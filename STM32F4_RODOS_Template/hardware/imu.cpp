#include "imu.hpp"

/*
Define addresses of sensors
*/
#define LSM9DS1_M_ADDR 0x1E                 // address of magnetometer of IMU
#define LSM9DS1_AG_ADDR 0x6B                // address of accelerometer + gyroscope of IMU

Topic<TimestampedData<IMUData>> IMUDataTopic(-1, "IMUData");


IMU::IMU(RODOS::I2C_IDX i2c):
	i2c(i2c)
{

}


TimestampedData<IMUData>& IMU::getData()
{
	return this->dataCalibrated;
}


TimestampedData<IMUData>& IMU::getDataRaw()
{
	return this->dataRaw;
}


void IMU::setGyroCalib(IMUCalib calib)
{
	this->gyroCalib.set(calib);
}


IMUCalib IMU::getGyroCalib()
{
	return this->gyroCalib.get();
}


void IMU::setAccelCalib(IMUCalib calib)
{
	this->accelCalib.set(calib);
}


IMUCalib IMU::getAccelCalib()
{
	return this->accelCalib.get();
}


void IMU::setMagCalib(IMUCalib calib)
{
	this->magCalib.set(calib);
}


IMUCalib IMU::getMagCalib()
{
	return this->magCalib.get();
}


void IMU::Check_I2C_Enable()
{

}


void IMU::Check_WHOAMI()
{

}

void IMU::initialization()
{
	i2c.init(400000);
	gyroInit();
	magInit();
	accelInit();
}

TimestampedData<IMUData>& IMU::readRawData()
{
	magRead();
	accelRead();
	gyroRead();
	tempRead();
	return this->dataRaw;
}

void IMU::gyroInit()
{
	// lsm9ds1.pdf: p.45 (current configuration: output data rate: 952 Hz; scale: 245 dps)
    uint8_t LSM9DS1_CTRL_REG1_G[2] = { 0x10, 0b11000000 };
	this->i2c.write(LSM9DS1_AG_ADDR, LSM9DS1_CTRL_REG1_G, 2);
}


void IMU::gyroRead()
{
	uint8_t data[2];

	this->i2c.writeRead(LSM9DS1_AG_ADDR, LSM9DS1_G_OUT_X, 1, data, 2);
	this->dataRaw.data.angularVelocity.x = grad2Rad(bytes_to_float(data) * gyroScale);

	this->i2c.writeRead(LSM9DS1_AG_ADDR, LSM9DS1_G_OUT_Y, 1, data, 2);
	this->dataRaw.data.angularVelocity.y = grad2Rad(bytes_to_float(data) * gyroScale);

	this->i2c.writeRead(LSM9DS1_AG_ADDR, LSM9DS1_G_OUT_Z, 1, data, 2);
	this->dataRaw.data.angularVelocity.z = grad2Rad(bytes_to_float(data) * gyroScale);

	this->dataRaw.timestamp = NOW();
}


void IMU::accelInit()
{
	// lsm9ds1.pdf: p.51 (current configuration: output data rate 952 Hz; scale: 2g; anti-aliasing filter bandwidth: 50 Hz)
	uint8_t LSM9DS1_CTRL_REG6_XL[2] = { 0x20, 0b11000011 };
	this->i2c.write(LSM9DS1_AG_ADDR, LSM9DS1_CTRL_REG6_XL, 2);
}


void IMU::accelRead()
{
	uint8_t data[2];
	// x-axis
	this->i2c.writeRead(LSM9DS1_AG_ADDR, LSM9DS1_XL_OUT_X, 1, data, 2);
	this->dataRaw.data.acceleration.x = bytes_to_float(data) * accelScale;
	// y-axis
	this->i2c.writeRead(LSM9DS1_AG_ADDR, LSM9DS1_XL_OUT_Y, 1, data, 2);
	this->dataRaw.data.acceleration.y = bytes_to_float(data) * accelScale;
	// z-axis
	this->i2c.writeRead(LSM9DS1_AG_ADDR, LSM9DS1_XL_OUT_Z, 1, data, 2);
	this->dataRaw.data.acceleration.z = bytes_to_float(data) * accelScale;

	this->dataRaw.timestamp = NOW();
}


void IMU::magInit()
{
	// lsm9ds1.pdf: p.63 (current configuration: output data rate 80 Hz -> max. value)
	uint8_t LSM9DS1_CTRL_REG1_M[2] = { 0x20, 0b00011100 };
	this->i2c.write(LSM9DS1_M_ADDR, LSM9DS1_CTRL_REG1_M, 2);
	// lsm9ds1.pdf: p.64 (current configuration: scale +- 4 gauss -> min. value)
	uint8_t LSM9DS1_CTRL_REG2_M[2] = { 0x21, 0b00000000 };
	this->i2c.write(LSM9DS1_M_ADDR, LSM9DS1_CTRL_REG2_M, 2);
	// lsm9ds1.pdf: p.64 (current configuration: i2c interface enabled)
	uint8_t LSM9DS1_CTRL_REG3_M[2] = { 0x22, 0b00000000 };
	this->i2c.write(LSM9DS1_M_ADDR, LSM9DS1_CTRL_REG3_M, 2);
}


void IMU::magRead()
{
	uint8_t data[2];

	this->i2c.writeRead(LSM9DS1_M_ADDR, LSM9DS1_M_OUT_X, 1, data, 2);
	this->dataRaw.data.magneticField.x = bytes_to_float(data) * magScale;

	this->i2c.writeRead(LSM9DS1_M_ADDR, LSM9DS1_M_OUT_Y, 1, data, 2);
	this->dataRaw.data.magneticField.y = bytes_to_float(data) * magScale;

	this->i2c.writeRead(LSM9DS1_M_ADDR, LSM9DS1_M_OUT_Z, 1, data, 2);
	this->dataRaw.data.magneticField.z = bytes_to_float(data) * magScale;

	this->dataRaw.timestamp = NOW();
}


void IMU::tempRead()
{	
	uint8_t data[2];

	this->i2c.writeRead(LSM9DS1_AG_ADDR, LSM9DS1_OUT_TEMP, 1, data, 2);
	this->dataRaw.data.temperature = (int16_t) ((data[1] << 8 | data[0]));
	this->dataRaw.timestamp = NOW();
}


float IMU::bytes_to_float(uint8_t* data)
{
	return (int16_t) ((data[1] << 8 | data[0]));
}


void IMU::calibrateData()
{
	IMUCalib gyro_calib = this->getGyroCalib();
	dataCalibrated.data.angularVelocity = dataRaw.data.angularVelocity;
	dataCalibrated.data.angularVelocity.z = -dataCalibrated.data.angularVelocity.z;
	dataCalibrated.data.angularVelocity = dataCalibrated.data.angularVelocity.vecSub(gyro_calib.bias).matVecMult(gyro_calib.scale);

	IMUCalib accel_calib = this->getAccelCalib();
	dataCalibrated.data.acceleration = dataRaw.data.acceleration;
	dataCalibrated.data.acceleration.z = -dataCalibrated.data.acceleration.z;
	dataCalibrated.data.acceleration = dataCalibrated.data.acceleration.vecSub(accel_calib.bias).matVecMult(accel_calib.scale);

	IMUCalib mag_calib = this->getMagCalib();
	dataCalibrated.data.magneticField = dataRaw.data.magneticField;
	dataCalibrated.data.magneticField.x = -dataRaw.data.magneticField.x;
	dataCalibrated.data.magneticField.z = -dataRaw.data.magneticField.z;
	dataCalibrated.data.magneticField = dataCalibrated.data.magneticField.vecSub(mag_calib.bias).matVecMult(mag_calib.scale);

	dataCalibrated.data.temperature = (dataRaw.data.temperature / 16.0) + 25.0;
	dataCalibrated.timestamp = NOW();
}


/**
 * ------- ONLY FOR TESTING -------
*/
bool IMU::isCalibRunning()
{
	return this->calibRunning;
}

void IMU::setCalibRunning(bool status)
{
	this->calibRunning = status;
}

bool IMU::isCalibDone()
{
	return this->calibDone;
}

void IMU::setCalibDone(bool status)
{
	this->calibDone = status;
}



IMU imu(I2C_IDX2);
