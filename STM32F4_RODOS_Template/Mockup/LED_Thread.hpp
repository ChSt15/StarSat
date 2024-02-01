#ifndef ORPE_LED_THREAD_HPP_
#define ORPE_LED_THREAD_HPP_

#include "rodos.h"

/**
 * Pins of the LEDs
 */
#define LEDPIN_1 GPIO_059           // PD11
#define LEDPIN_2 GPIO_060           // PD12
#define LEDPIN_3 GPIO_061           // PD13
#define LEDPIN_4 GPIO_062           // PD14
#define LEDPIN_5 GPIO_063           // PD15
#define LEDPIN_6 GPIO_079           // PE15
#define LEDPIN_7 GPIO_028           // PB12
#define LEDPIN_8 GPIO_029           // PB13
#define LEDPIN_9 GPIO_030           // PB14
#define LEDPIN_10 GPIO_031          // PB15
#define LEDPIN_11 GPIO_078          // PE14
#define LEDPIN_12 GPIO_076          // PE12


/**
 * @brief Namespace used by ORPE (Optical relative pose estimation)
 */
namespace ORPE {

//Number of LEDs to control.
constexpr size_t NUMLEDS = 12;

/**
 * @brief This class will control the LEDs for Optical Relative Pose Estimation.
 */
class ORPE_LEDControl : public Thread {
private:

	///@brief Number of frames untill the code repeats.
	static constexpr size_t codeRepeat = 47;

	bool ledActive_[NUMLEDS];
	HAL_GPIO gpio_[NUMLEDS];
	bool code_[NUMLEDS][codeRepeat];
	size_t codeBit_ = 0;

	bool ledPower_ = false;
	bool ledForcedOn_ = false;

	int64_t frameInterval_ns_ = END_OF_TIME;

public:

	ORPE_LEDControl(int64_t frameInterval_ns);

	/**
	 * @brief Sets up an led using the given GPIO pin and LEDID.
	 * @param ledIndex The LED index, from 0 to NUMLED.
	 * @param ledPin The GPIO pin to use for this led.
	 * @param ledID What ID to give this LED.
	 * @param startFrame How many frames to wait before starting first code.
	 */
	void setupLED(size_t ledIndex, GPIO_PIN ledGPIO, uint8_t ledID, size_t startFrame);

	/**
	 * @brief Starts the led coding.
	 */
	void setCoding(bool coding);

	/**
	 * @brief Forces leds to stay on (no coding).
	 */
	void setPower(bool power);

private:

	void init() override;

	void run() override;

};

}

#endif