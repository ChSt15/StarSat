#ifndef FLOATSAT_ELECTRICAL_ELECTRICALMONITORING_HPP_
#define FLOATSAT_ELECTRICAL_ELECTRICALMONITORING_HPP_

#include "rodos.h"

#include "ina3221/Beastdevices_INA3221.hpp"
#include "../i2c_init.hpp"
#include "../Modes.hpp"


/**
 * @brief 	This class monitors the electrical system of the satellite and does some protection logic.
 * @note 	The INA3221 is used to measure most electrical parameters. 
 * 			The INA3221 channels are connected in the following:
 * 				-0: RPI zero 2
 * 				-1: STM32 and other small devices (LSM9DS1, switches etc.)
 * 				-2: TMC2209 Stepper driver on input side. (This also gives us the Battery voltage)
 * 			The Reaction wheel current is measured using the H-Bridge current output pin connected to an ADC pin directly.
 * 				
 * 
*/
class ElectricalMonitoring : public RODOS::Thread
{
private:

    /// @brief The time between sensor reads in nanoseconds.
    static constexpr int64_t sensorReadInterval_ns = 0.2*SECONDS; // 5 Hz
	/// @brief Shunt resistor used by board in mOhms.
	static constexpr float SHUNT_RESISTOR = 100;
	/// @brief Factor used to convert H-Bridge current output to amps in Volts per Amp
	static constexpr float HBRIDGE_I_FACTOR = 0.5f;
    /// @brief Current threshold for the rpi to be considered running in Amps.
    static constexpr float RPI_RUNNING_CURRENT = 0.310f;
    /// @brief How much the 5V bus voltage can deviate from 5V before it is considered bad.
    static constexpr float BUS_VOLTAGE_TOLERANCE = 0.2f;
    /// @brief How long to wait after powering the chips off before powering them back on.
    static constexpr int64_t CHIP_RESET_WAIT_TIME = 500*MILLISECONDS;

    static constexpr ina3221_ch_t STM32I = INA3221_CH1;    // Channel with STM32 and other small devices (LSM9DS1, switches etc.) current measurement
    static constexpr ina3221_ch_t RPII = INA3221_CH2;      // Channel with RPI zero 2 current measurement
    static constexpr ina3221_ch_t STEPPERI = INA3221_CH3;  // Channel with TMC2209 Stepper driver on input side current measurement

    static constexpr ina3221_ch_t BUS5VU = INA3221_CH1;    // Channel with 5V Bus voltage measurement
    static constexpr ina3221_ch_t VBATU = INA3221_CH3;     // Channel with Battery voltage measurement

	/**
	 * @brief A simple thread to control beeper timing.
	*/
	class BeeperThread : public RODOS::Thread
	{
	private:

        RODOS::Semaphore beepSem_;

		RODOS::HAL_PWM beeper;

		int64_t beepTime_ns;

        bool isBeeping_ = false;

	public:

		BeeperThread(RODOS::PWM_IDX beeper);

		void beepForTime_ns(int64_t time_ns);

		void init();

		void run();

	};

    Beastdevices_INA3221 ina3221_;

	BeeperThread beeperThread_;

	float voltageBattery_;
	float voltage5VBus_;

	float currentStepper_;
	float currentAux_;
	float currentReactionWheel_;
	float currentRPI_;

	bool powerGood_ = false;
	bool rpiRunning_ = false;

    bool rpiPowerOn_ = false;

	RODOS::HAL_I2C i2cBus_;

	RODOS::ADC_CHANNEL adcWheelPin_;
	RODOS::PWM_IDX beeperIDX_;

    RODOS::HAL_GPIO chipPower_;
    RODOS::HAL_GPIO rpiPower_;
	RODOS::HAL_GPIO extPower_;
	RODOS::HAL_ADC adcWheel_;
	RODOS::HAL_PWM beeper_;

	enum class SystemState_t {
		INIT,
		CONV_INIT,
		CONV_WAIT,
		POWERDOWN_INIT,
		POWERDOWN_WAIT,
		POWERDOWN_COMPLETE
	};

    enum class ChipResetState_t {
		IDLE,
        RESET_INIT,
        CHIP_WAIT,
        STARTUP_WAIT
	};

	SystemState_t state_ = SystemState_t::INIT;
    int64_t convWaitBeginTime_ns_ = 0;

    ChipResetState_t chipResetState_ = ChipResetState_t::IDLE;
    int64_t chipResetStartTime_ns_ = 0;

public:

	/// @brief The maximum voltage the battery should have.
	static constexpr float batteryMaxVoltage = 3.6*4;
	/// @brief The minimum operating voltage the battery should reach. After this point the RPI will be shutdown.
	static constexpr float batteryWarningVoltage = 3.2*4;
	/// @brief The voltage at which the system will power down.
	static constexpr float batteryCutoffVoltage = 2.8*4; 

	/**
	 * @brief Constructs the electrical monitoring object.
	 * @param powerOff GPIO connected to the external power off switch.
	 * @param adcWheel ADC channel connected to the reaction wheel driver current output.
	 * @param beeperPin PWM pin connected to the warning beeper.
	 * @param ina3221_i2cBus I2C bus connected to the INA3221 chip.
	*/
	ElectricalMonitoring(RODOS::GPIO_PIN rpiPowerPin, RODOS::GPIO_PIN chipPowerPin, RODOS::GPIO_PIN powerOffPin, RODOS::ADC_CHANNEL adcWheelPin, RODOS::PWM_IDX beeperPin, RODOS::I2C_IDX ina3221_i2cBus);

	/**
	 * Call once quickly after system startup.
	*/
	void initialize();

	/**
	 * @brief This function is to be called at 10 Hz.
	 * 
	*/
	void update();

    /**
     * Will power off the IMU, INA3221 for 500ms and then power them back on.
    */
    void startChipReset();

    /**
     * @brief Get the current power state of the IMU and INA3221. Used to check if the reset is complete.
     * @returns true if the IMU and INA3221 are powered on.
    */
    bool getChipPower();

	/**
	 * @brief Gets the battery voltage.
	 * @note  This is also the Stepper driver and reaction wheel operating voltage.
	*/
	float getBatteryVoltage();

	/**
	 * @brief Gets the voltage of the 5V bus.
	 * @note  The RPI, STM32 operating voltage.
	*/
	float get5VBusVoltage();

	/**
	 * @brief Gets the current used by the RPI in Amps.
	 * @note  The RPI is connected to the 5V Bus.
	*/
	float getRPICurrent();

	/**
	 * @brief Gets the current used by the Stepper driver (And therefore the Stepper motor) in Amps.
	 * @note  This is the sum of the Stepper motor and stepper driver current usage.
	*/
	float getStepperCurrent();

	/**
	 * @brief Gets the current used by all other systems, e.g. STM32, LSM9DS1, switches etc
	*/
	float getAuxCurrent();

	/**
	 * @returns the current drawn by the reaction wheels in Amps.
	*/
	float getReactionWheelCurrent();

	/**
	 * @returns true if battery and 5V Bus voltages are good and the RPI has been turned on. 
	*/
	bool powerGood();

	/**
	 * @returns true if the RPI has powered on and gave the signal its ready.
	*/
	bool rpiRunning();

	/**
	 * @brief Will begin the proccess of turning off the system.
	 * @note Will first command the RPI to power down, once finished will disconnect RPI power and then call openExtSwitch() to power down.
	*/
	void powerDown();

	/**
	 * @brief Will forcefully power off everything immediately
	 * 
	*/
	void openExtSwitch();

private:

    void init() override;

    void run() override;

	/**
	 * @brief Reads all values off the INA3221 chip.
	 * @param setValue If true, then no low pass filter will be used and read values used directly.
	*/
	void readValues(bool setValue = false);

};


extern ElectricalMonitoring electricalMonitor;

#endif