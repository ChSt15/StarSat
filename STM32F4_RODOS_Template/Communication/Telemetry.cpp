#include "Telemetry.hpp"

// IMU topic sub
static CommBuffer<TimestampedData<IMUData>> IMUDataBuffer;
static Subscriber IMUDataSubsciber(IMUDataTopic, IMUDataBuffer);
TimestampedData<IMUData> IMUDataReceiver;

// IMU telemetry topic
Topic<IMUData> IMUTelemetryTopic(40, "IMU_Telemetry");

// Set up uart gatway
static HAL_UART uart(UART_IDX3);
static LinkinterfaceUART uart_linkerinterface(&uart);
static Gateway uart_gateway(&uart_linkerinterface);


Telemety::Telemety()
{
	// topics to forward
	topics.add(IMUTelemetryTopic.topicId);
	//..
	uart_gateway.setTopicsToForward(&(this->topics));
}


void Telemety::send_Continuous()
{
	// Collect data
	IMUDataBuffer.get(IMUDataReceiver);
	// ...

	// Send data
	IMUTelemetryTopic.publish(IMUDataReceiver.data);
}


void Telemety::send_CalibGyro()
{
	// Collect data
	//imu.getGyroCalib();

	// Send data
}

void Telemety::send_CalibAccel()
{
	// Collect data
	// imu.getAccelCalib();
	
	// Send data
}

void Telemety::send_CalibMag()
{
	// Collect data
	// imu.getMagCalib();

	// Send data
}

Telemety telemetry;