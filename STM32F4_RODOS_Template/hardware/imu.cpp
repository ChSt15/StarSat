#include "imu.hpp"

/*
Define addresses of sensors
*/
#define LSM9DS1_M_ADDR 0x1E                 // address of magnetometer of IMU
#define LSM9DS1_AG_ADDR 0x6B                // address of accelerometer + gyroscope of IMU

Topic<TimestampedData<IMUData>> IMUDataTopic(-1, "IMUData");

static IMUCalib gyroCalib_local;
static IMUCalib accelCalib_local;
static IMUCalib magCalib_local;

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
	PROTECT_WITH_SEMAPHORE(sem) this->gyroCalib = calib;
	newCalib = true;
}


IMUCalib IMU::getGyroCalib()
{
	IMUCalib calib; 
	PROTECT_WITH_SEMAPHORE(sem) calib = this->gyroCalib;
	return calib;
}


void IMU::setAccelCalib(IMUCalib calib)
{
	PROTECT_WITH_SEMAPHORE(sem) this->accelCalib = calib;
	newCalib = true;
}


IMUCalib IMU::getAccelCalib()
{
	IMUCalib calib;
	PROTECT_WITH_SEMAPHORE(sem) calib = this->accelCalib;
	return calib;
}


void IMU::setMagCalib(IMUCalib calib)
{
	PROTECT_WITH_SEMAPHORE(sem) this->magCalib = calib;
	newCalib = true;
}


IMUCalib IMU::getMagCalib()
{
	IMUCalib calib;
	PROTECT_WITH_SEMAPHORE(sem) calib = this->magCalib;
	return calib;
}


void IMU::Check_I2C_Enable()
{

	if (CS_XLG.readPins() == 1)
	{
		PRINTF("I2C Enable Check for Accel/Gyro was successfull\n");
	}
	else
	{
		PRINTF("I2C Enable Check for Accel/Gyro failed\n");
	}
	if (CS_M.readPins() == 1)
	{
		PRINTF("I2C Enable Check for Magnetometer was successfull\n");
	}
	else
	{
		PRINTF("I2C Enable Check for Magnetometer failed\n");
	}
}


void IMU::Check_WHOAMI()
{
	uint8_t DATA[1];

	uint8_t LSM9DS1_WHO_AM_I[1] = { 0x0F };

	i2c.writeRead(LSM9DS1_AG_ADDR, LSM9DS1_WHO_AM_I, 1, DATA, 1);
	if (DATA[0] == 0b01101000)
	{
		PRINTF("WHO AM I Check for Accel/Gyro was successfull\n");
	}
	else
	{
		PRINTF("WHO AM I Check for Accel/Gyro failed\n");
	}

	i2c.writeRead(LSM9DS1_M_ADDR, LSM9DS1_WHO_AM_I, 1, DATA, 1);
	if (DATA[0] == 0b00111101)
	{
		PRINTF("WHO AM I Check for Magnetometer was successfull\n");
	}
	else
	{
		PRINTF("WHO AM I Check for Magnetometer failed\n");
	}
	PRINTF("\n");
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
	this->dataRaw.timestamp = NOW();
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
	this->dataRaw.data.angularVelocity.x =  grad2Rad(bytes_to_float(data) * gyroScale);

	this->i2c.writeRead(LSM9DS1_AG_ADDR, LSM9DS1_G_OUT_Y, 1, data, 2);
	this->dataRaw.data.angularVelocity.y =  grad2Rad(bytes_to_float(data) * gyroScale);

	this->i2c.writeRead(LSM9DS1_AG_ADDR, LSM9DS1_G_OUT_Z, 1, data, 2);
	this->dataRaw.data.angularVelocity.z = -grad2Rad(bytes_to_float(data) * gyroScale);
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
	this->dataRaw.data.acceleration.x =  bytes_to_float(data) * accelScale;
	// y-axis
	this->i2c.writeRead(LSM9DS1_AG_ADDR, LSM9DS1_XL_OUT_Y, 1, data, 2);
	this->dataRaw.data.acceleration.y =  bytes_to_float(data) * accelScale;
	// z-axis
	this->i2c.writeRead(LSM9DS1_AG_ADDR, LSM9DS1_XL_OUT_Z, 1, data, 2);
	this->dataRaw.data.acceleration.z = -bytes_to_float(data) * accelScale;
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
	this->dataRaw.data.magneticField.x = -bytes_to_float(data) * magScale;

	this->i2c.writeRead(LSM9DS1_M_ADDR, LSM9DS1_M_OUT_Y, 1, data, 2);
	this->dataRaw.data.magneticField.y =  bytes_to_float(data) * magScale;

	this->i2c.writeRead(LSM9DS1_M_ADDR, LSM9DS1_M_OUT_Z, 1, data, 2);
	this->dataRaw.data.magneticField.z = -bytes_to_float(data) * magScale;
}


void IMU::tempRead()
{	
	uint8_t data[2];

	this->i2c.writeRead(LSM9DS1_AG_ADDR, LSM9DS1_OUT_TEMP, 1, data, 2);
	this->dataRaw.data.temperature = (int16_t) ((data[1] << 8 | data[0]));
}


float IMU::bytes_to_float(uint8_t* data)
{
	return (int16_t) ((data[1] << 8 | data[0]));
}


void IMU::calibrateData()
{
	if (newCalib)
	{
		gyroCalib_local = this->getGyroCalib();
		accelCalib_local = this->getAccelCalib();
		magCalib_local = this->getMagCalib();
		newCalib = false;
	}

	dataCalibrated.data.angularVelocity = dataRaw.data.angularVelocity.vecSub(gyroCalib_local.bias).matVecMult(gyroCalib_local.scale);
	dataCalibrated.data.acceleration = dataRaw.data.acceleration.vecSub(accelCalib_local.bias).matVecMult(accelCalib_local.scale);
	dataCalibrated.data.magneticField = dataRaw.data.magneticField.vecSub(magCalib_local.bias).matVecMult(magCalib_local.scale);

	dataCalibrated.data.temperature = (dataRaw.data.temperature / 16.0) + 25.0;
	dataCalibrated.timestamp = dataRaw.timestamp;
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
