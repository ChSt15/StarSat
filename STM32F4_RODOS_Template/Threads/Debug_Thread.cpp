#include "Debug_Thread.hpp"

#include "rodos.h"
#include "matlib.h"

#include "../control/AttitudeEstimation.hpp"
#include "../hardware/imu.hpp"

static CommBuffer<IMUData> IMUDataBuffer;
static Subscriber IMUDataSubsciber(IMUDataTopic, IMUDataBuffer);

static CommBuffer<Attitude_Data> AttitudeDataBuffer;
static Subscriber AttitudeDataSubsciber(AttitudeDataTopic, AttitudeDataBuffer);


void DebugThread::init()
{

}

void DebugThread::run()
{

	Attitude_Data AttitudeDataReceiver;
	IMUData IMUDataReceiver;

	while (true)
	{
		IMUDataBuffer.get(IMUDataReceiver);
		PRINTF("Gyro: %f, %f, %f\n", IMUDataReceiver.angularVelocity.x, IMUDataReceiver.angularVelocity.y, IMUDataReceiver.angularVelocity.z);
		PRINTF("Accel: %f, %f, %f\n", IMUDataReceiver.acceleration.x, IMUDataReceiver.acceleration.y, IMUDataReceiver.acceleration.z);
		PRINTF("Mag: %f, %f, %f\n", IMUDataReceiver.magneticField.x, IMUDataReceiver.magneticField.y, IMUDataReceiver.magneticField.z);
		PRINTF("Temp: %f\n\n", IMUDataReceiver.temperature);

		AttitudeDataBuffer.get(AttitudeDataReceiver);
		YPR ypr = AttitudeDataReceiver.attitude.toYPR();
		PRINTF("Yaw: %f, Pitch: %f, Roll: %f\n\n", ypr.yaw, ypr.pitch, ypr.roll);
		
		suspendCallerUntil(NOW() + period * MILLISECONDS);
	}
}


DebugThread debugthread;