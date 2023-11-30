#include "Telemetry.hpp"

// IMU topic sub
static CommBuffer<TimestampedData<IMUData>> IMUDataBuffer;
static Subscriber IMUDataSubsciber(IMUDataTopic, IMUDataBuffer);
TimestampedData<IMUData> IMUDataReceiver;

// IMU telemetry topic
Topic<IMUData> IMUTelemetryTopic(40, "IMU_Telemetry");

/*
struct IMUTelemetry
{
	float wx, wy, wz;       // [rad/s]
	float mx, my, mz;       // [gauss]
	float ax, ay, az;       // [g]
	float temperature;      // [°C]
};
Topic<IMUTelemetry> IMUTelemetryTopic(40, "IMU_Telemetry");
*/

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
	/*
	IMUTelemetry temp;
	temp.wx = IMUDataReceiver.data.angularVelocity.x;
	temp.wy = IMUDataReceiver.data.angularVelocity.y;
	temp.wz = IMUDataReceiver.data.angularVelocity.z;

	temp.ax = IMUDataReceiver.data.acceleration.x;
	temp.ay = IMUDataReceiver.data.acceleration.y;
	temp.az = IMUDataReceiver.data.acceleration.z;

	temp.mx = IMUDataReceiver.data.magneticField.x;
	temp.my = IMUDataReceiver.data.magneticField.y;
	temp.mz = IMUDataReceiver.data.magneticField.z;

	temp.temperature = IMUDataReceiver.data.temperature;

	IMUTelemetryTopic.publish(temp);
	*/

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