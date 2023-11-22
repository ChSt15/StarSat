#include "rodos.h"
#include <stdio.h>
#include "hal.h"
#include "math.h"
#include "../hardware/imu.hpp"

#define LED_GREEN GPIO_060

HAL_GPIO greed_led(LED_GREEN);


class Testing: public Thread {

private:
    TimestampedData<IMUData> imu_data;
    float gx = 0.0;
    float gy = 0.0;
    float gz = 0.0;

    float ax = 0.0;
    float ay = 0.0;
    float az = 0.0;

    float mx = 0.0;
    float my = 0.0;
    float mz = 0.0;

    float temp = 0.0;

public:

	Testing(const char* name) : Thread(name) {
        
	}

	void init() {
		greed_led.init(true, 1, 0);
        imu.initialization();
	}

	void run() {

        bool calibration_done = false;
		while (1) {

            if(!imu.isCalibRunning())
            {
                greed_led.setPins(~greed_led.readPins());

                imu_data = imu.readRawData();

                if(imu.isCalibDone()) {
                    imu.calibrateData();
                    imu_data = imu.getData();
                }

                gx = imu_data.data.angularVelocity.x;
                gy = imu_data.data.angularVelocity.y;
                gz = imu_data.data.angularVelocity.z;

                ax = imu_data.data.acceleration.x;
                ay = imu_data.data.acceleration.y;
                az = imu_data.data.acceleration.z;

                mx = imu_data.data.magneticField.x;
                my = imu_data.data.magneticField.y;
                mz = imu_data.data.magneticField.z;

                temp = imu_data.data.temperature;

			    PRINTF("gx: %f, gy: %f, gz: %f \t ax: %f, ay: %f, az: %f \t mx: %f, my: %f, mz: %f \t temp: %f \n", gx, gy, gz, ax, ay, az, mx, my, mz, temp);

                suspendCallerUntil(NOW() + 100 * MILLISECONDS);
            } else {
                greed_led.setPins(0);
                suspendCallerUntil(NOW() + 1 * SECONDS);
            }
		}
	}
};

Testing test("Testing");
