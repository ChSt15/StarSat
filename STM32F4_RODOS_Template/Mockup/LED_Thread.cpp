#include "rodos.h"
#include "LED_Thread.hpp"

using namespace ORPE;


ORPE_LEDControl::ORPE_LEDControl(int64_t frameInterval_ns) : 
    Thread("LED Thread",100, 5000)
    {
	frameInterval_ns_ = frameInterval_ns;
	for (size_t i = 0; i < NUMLEDS; i++) {
		ledActive_[i] = false;
	}
    /**
     * @todo Specify last two parameters: ledID and startFrame
    */
   /*
    this->setupLED(0, LEDPIN_1, 0b00000001, 1);
	this->setupLED(1, LEDPIN_2, 0b00000001, 2);
	this->setupLED(2, LEDPIN_3, 0b00000001, 3);
	this->setupLED(3, LEDPIN_4, 0b00000001, 4);
	this->setupLED(4, LEDPIN_5, 0b00000001, 5);
	this->setupLED(5, LEDPIN_6, 0b00000001, 6);
	this->setupLED(6, LEDPIN_7, 0b00000001, 7);
	this->setupLED(7, LEDPIN_8, 0b00000001, 8);
    this->setupLED(8, LEDPIN_9, 0b00000001, 9);
	this->setupLED(9, LEDPIN_10, 0b00000001, 10);
    this->setupLED(10, LEDPIN_11, 0b00000001, 11);
	this->setupLED(11, LEDPIN_12, 0b00000001, 12);*/

	this->setupLED(0, LEDPIN_6, 1, 0);
	this->setupLED(1, LEDPIN_8, 2, 4);
	this->setupLED(2, LEDPIN_10, 3, 8);
	this->setupLED(3, LEDPIN_1, 4, 12);
	this->setupLED(4, LEDPIN_4, 5, 16);
	this->setupLED(5, LEDPIN_11, 6, 20);
	this->setupLED(6, LEDPIN_9, 7, 24);
	this->setupLED(7, LEDPIN_5, 8, 28);
    this->setupLED(8, LEDPIN_3, 9, 32);
	this->setupLED(9, LEDPIN_7, 10, 36);
    this->setupLED(10, LEDPIN_2, 11, 40);
	this->setupLED(11, LEDPIN_12, 12, 44);

    // Set coding
    this->setCoding(true);
}

void ORPE_LEDControl::setupLED(size_t ledIndex, GPIO_PIN ledGPIO, uint8_t ledID, size_t startFrame) {
    //return;
	//Setup gpio
	gpio_[ledIndex] = HAL_GPIO(ledGPIO);
	//gpio_[ledIndex].init(true, 1, 0);

	//Set coding begin
	code_[ledIndex][startFrame] = false;

	size_t parity0 = false;
	size_t parity1 = false;
	for (size_t i = 0; i < codeRepeat-1; i++) { //We will start coding at index of first bit, once coding finished set remaining bits until next start to true.

		size_t index = i + startFrame + 1; //Index of current bit in the coding timing.

		if (i < 8) { //Calculate parity bits during coding time

			if (i%2 == 0) //Even or odd bit?
				parity0 += 0b00000001 & (ledID>>i);
			else
				parity1 += 0b00000001 & (ledID>>i);

		}

		if (i < 8)
			code_[ledIndex][index%codeRepeat] = 0b00000001 & (ledID>>(7-i)); //Set ID bits. If end of array, wrap around to beginning.
		else if (i == 8)
			code_[ledIndex][index%codeRepeat] = parity0%2 == 0; //Set parity 0 if odd number even bits. If end of array, wrap around to beginning.
		else if (i == 9)
			code_[ledIndex][index%codeRepeat] = parity1%2 == 0; //Set parity 1 if odd number uneven bits. If end of array, wrap around to beginning.
		else
			code_[ledIndex][index%codeRepeat] = true; //Set bits to true. If end of array, wrap around to beginning.

	}

	ledActive_[ledIndex] = true;

}


void ORPE_LEDControl::init() {
}

void ORPE_LEDControl::run() {


    // Setup LEDs
    
    for (size_t i = 0; i < NUMLEDS; i++) {
        gpio_[i].init(true, 1, 0);
    }

	int64_t deadline = NOW();

	while(1) {

		//Calculates the next frame.
		deadline = NOW() - NOW() % frameInterval_ns_ + frameInterval_ns_;

		for (size_t i = 0; i < NUMLEDS; i++) {

			if (!ledActive_[i]) continue; //Skip, led is not setup

			gpio_[i].setPins((code_[i][codeBit_] && ledPower_) || ledForcedOn_);

		}

		suspendCallerUntil(deadline);

		codeBit_++;
		if (codeBit_ == codeRepeat) codeBit_ = 0;

	}

}


void ORPE_LEDControl::setCoding(bool coding) {
	ledPower_ = coding;
}


void ORPE_LEDControl::setPower(bool power) {
	ledForcedOn_ = power;
}



/**
 * @todo set correct frameInterval 
*/
ORPE_LEDControl LED_THREAD(SECONDS/20);
