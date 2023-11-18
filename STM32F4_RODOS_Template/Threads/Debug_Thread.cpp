#include "Debug_Thread.hpp"

#include "rodos.h"
#include "matlib.h"

#include "../control/AttitudeEstimation.hpp"
#include "../hardware/imu.hpp"

static CommBuffer<TimestampedData<IMUData>> IMUDataBuffer;
static Subscriber IMUDataSubsciber(IMUDataTopic, IMUDataBuffer);

static CommBuffer<TimestampedData<Attitude_Data>> AttitudeDataBuffer;
static Subscriber AttitudeDataSubsciber(AttitudeDataTopic, AttitudeDataBuffer);


void DebugThread::init()
{

}

void DebugThread::run()
{
	TimestampedData<IMUData> IMUDataReceiver;
	TimestampedData<Attitude_Data> AttitudeDataReceiver;

	while (true)
	{
		IMUDataBuffer.get(IMUDataReceiver);
		PRINTF("Gyro: %f, %f, %f\n", IMUDataReceiver.data.angularVelocity.x, IMUDataReceiver.data.angularVelocity.y, IMUDataReceiver.data.angularVelocity.z);
		PRINTF("Accel: %f, %f, %f\n", IMUDataReceiver.data.acceleration.x, IMUDataReceiver.data.acceleration.y, IMUDataReceiver.data.acceleration.z);
		PRINTF("Mag: %f, %f, %f\n", IMUDataReceiver.data.magneticField.x, IMUDataReceiver.data.magneticField.y, IMUDataReceiver.data.magneticField.z);
		PRINTF("Temp: %f\n\n", IMUDataReceiver.data.temperature);

		AttitudeDataBuffer.get(AttitudeDataReceiver);
		YPR ypr = AttitudeDataReceiver.data.attitude.toYPR();
		PRINTF("Yaw: %f, Pitch: %f, Roll: %f\n\n", rad2Grad(ypr.yaw), rad2Grad(ypr.pitch), rad2Grad(ypr.roll));
		
		suspendCallerUntil(NOW() + period * MILLISECONDS);
	}
}


DebugThread debugthread;