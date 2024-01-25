#include "../Config.hpp"

#include "ElectricalMonitoring.hpp"

ElectricalMonitoring::BeeperThread::BeeperThread(RODOS::PWM_IDX beeper) :
    Thread("BeeperThread", 100),
    beeper(beeper)
{}

void ElectricalMonitoring::BeeperThread::beepForTime_ns(int64_t time_ns) {

    beepSem_.enter();
    beepTime_ns = time_ns;
    beeper.write(50);
    //this->suspendCallerUntil(NOW() + beepTime_ns);
    if (!isBeeping_)
        this->resume();

    beepSem_.leave();

}

void ElectricalMonitoring::BeeperThread::init()
{
    beeper.init(1000, 100);
    beeper.write(0);
}

void ElectricalMonitoring::BeeperThread::run()
{

    while(1)
    {   

        suspendCallerUntil(END_OF_TIME);

        beepSem_.enter();
        auto val = beepTime_ns;
        beepSem_.leave();

        suspendCallerUntil(NOW() + beepTime_ns);

        beeper.write(0);
    
    }

}


ElectricalMonitoring::ElectricalMonitoring(RODOS::GPIO_PIN rpiPowerPin, RODOS::GPIO_PIN chipPowerPin, RODOS::GPIO_PIN powerOffPin, RODOS::ADC_CHANNEL adcWheelPin, RODOS::PWM_IDX beeperPin, RODOS::I2C_IDX ina3221_i2cBus) :
    Thread("ElectricalMonitoring"),
    ina3221_(INA3221_ADDR40_GND),
    beeperThread_(beeperPin),
    chipPower_(chipPowerPin),
    rpiPower_(rpiPowerPin),
    extPower_(powerOffPin),
    adcWheelPin_(adcWheelPin),
    beeperIDX_(beeperPin),
    adcWheel_(RODOS::ADC_IDX0),
    beeper_(beeperPin),
    i2cBus_(ina3221_i2cBus)
{}

void ElectricalMonitoring::initialize()
{
    state_ = SystemState_t::INIT;
}

void ElectricalMonitoring::update()
{   
    //Main state machine
    switch (state_)
    {
    case SystemState_t::INIT :
        {
            beeperThread_.beepForTime_ns(800*MILLISECONDS);
            chipPower_.init(true, 1, 1);
            rpiPower_.init(true, 1, 0);
            extPower_.init(true, 1, 1);
            adcWheel_.init(adcWheelPin_);
            init_i2c();

            ina3221_.begin(&i2cBus_);
            ina3221_.reset();

            // Set shunt resistors for all channels
            ina3221_.setShuntRes(SHUNT_RESISTOR, SHUNT_RESISTOR, SHUNT_RESISTOR);

            readValues(true);
            state_ = SystemState_t::CONV_INIT;

            setMode(Idle);

            //suspendCallerUntil(END_OF_TIME);
        }
        break;

    case SystemState_t::CONV_INIT :
        
        {
            //Read values
            readValues(false);



            //Check if power should be turned off
            if (!powerGood_) {
                openExtSwitch();
            }

            //Check if battery is low
            if (voltageBattery_ < batteryWarningVoltage && voltageBattery_ > 6.0f) {
                beeperThread_.beepForTime_ns(50*MILLISECONDS);
                //beeperThread_.resume();
            }

            //Check if we can turn on the RPI
            if (!rpiPowerOn_ && powerGood_ && config::enable_rpi) {
                rpiPowerOn_ = true;
                rpiPower_.setPins(1);
            }

            //Begin waiting for next conversion
            convWaitBeginTime_ns_ = NOW();
            state_ = SystemState_t::CONV_WAIT;
        }

        break;

    case SystemState_t::CONV_WAIT :
        {
            //Wait for next conversion
            if (NOW() - convWaitBeginTime_ns_ > sensorReadInterval_ns) {
                state_ = SystemState_t::CONV_INIT;
            }
            
        }
        break;

    case SystemState_t::POWERDOWN_INIT :
        {
            readValues(false);

            if (!powerGood_) {
                openExtSwitch();
            } 

            if (!rpiRunning_)
                state_ = SystemState_t::POWERDOWN_COMPLETE;

            convWaitBeginTime_ns_ = NOW();
            state_ = SystemState_t::POWERDOWN_WAIT;
        }
        break;

    case SystemState_t::POWERDOWN_WAIT :
        {
            //Wait for next conversion
            if (NOW() - convWaitBeginTime_ns_ > sensorReadInterval_ns) {
                state_ = SystemState_t::POWERDOWN_INIT;
            }
            
        }
        break;

    case SystemState_t::POWERDOWN_COMPLETE :
        {
            openExtSwitch(); //Adios
        }
        break;
    
    default:
        state_ = SystemState_t::INIT;
        break;
    }

    //Chip reset state machine
    switch (chipResetState_)
    {
    case ChipResetState_t::IDLE:
        break;

    case ChipResetState_t::RESET_INIT:
        if (state_ != SystemState_t::INIT) {
            
            chipPower_.setPins(0); //Power off
            chipResetStartTime_ns_ = NOW();
            chipResetState_ = ChipResetState_t::CHIP_WAIT;

            beeperThread_.beepForTime_ns(200*MILLISECONDS);
        }

        break;

    case ChipResetState_t::CHIP_WAIT:
        {
            if (NOW() - chipResetStartTime_ns_ > CHIP_RESET_WAIT_TIME) {
                chipPower_.setPins(1); //Power on
                chipResetState_ = ChipResetState_t::IDLE;
            }
        }

        break;

    default:
        {
            chipPower_.setPins(1); //Power on
            chipResetState_ = ChipResetState_t::IDLE;
        }

        break;
    }

}	

void ElectricalMonitoring::startChipReset() {
    chipResetState_ = ChipResetState_t::RESET_INIT;
}

bool ElectricalMonitoring::getChipPower() {
    return chipResetState_ == ChipResetState_t::IDLE;
}

void ElectricalMonitoring::readValues(bool setValue) {

    //Measure voltages
    voltageBattery_ = ina3221_.getVoltage(VBATU);
    voltage5VBus_ = ina3221_.getVoltage(BUS5VU);

    //Measure currents
    currentStepper_ = ina3221_.getCurrent(STEPPERI);
    currentAux_ = ina3221_.getCurrent(STM32I);
    currentRPI_ = ina3221_.getCurrent(RPII);

    currentReactionWheel_ = float(adcWheel_.read(adcWheelPin_)) / 1023 * 3.3f / HBRIDGE_I_FACTOR;

    //Check if power is good
    powerGood_ = (voltage5VBus_ < 5.0f + BUS_VOLTAGE_TOLERANCE) && (voltage5VBus_ >  5.0f - BUS_VOLTAGE_TOLERANCE) && (voltageBattery_ > batteryCutoffVoltage);

    //Check if RPI is running
    rpiRunning_ = (currentRPI_ > RPI_RUNNING_CURRENT);

    //Print everything
    //PRINTF("VBAT: %f V, 5V: %f V, Stepper: %f A, Aux: %f A, RPI: %f A, RW: %f A, PowerGood: %d, RPI Running: %d\n", voltageBattery_, voltage5VBus_, currentStepper_, currentAux_, currentRPI_, currentReactionWheel_, powerGood_, rpiRunning_);

}

float ElectricalMonitoring::getBatteryVoltage() {
    return voltageBattery_;
}

float ElectricalMonitoring::getAuxCurrent() {
    return currentAux_;
}

float ElectricalMonitoring::getReactionWheelCurrent() {
    return currentReactionWheel_;
}

float ElectricalMonitoring::get5VBusVoltage() {
    return voltage5VBus_;
}

float ElectricalMonitoring::getRPICurrent() {
    return currentRPI_;
}

float ElectricalMonitoring::getStepperCurrent() {
    return currentStepper_;
}

void ElectricalMonitoring::powerDown() {
    state_ = SystemState_t::POWERDOWN_INIT;
}

void ElectricalMonitoring::openExtSwitch() {


//      /̵͇̿̿/’̿’̿ ̿ ̿̿ ̿̿ ̿̿ (X__X)

    extPower_.init(false, 1, 0); //Open switch via High-Z.

    //Pull-up resistor will do the rest (RIP). Program will run for an unknown amount of time.

}

bool ElectricalMonitoring::powerGood() {
    return powerGood_;
}

bool ElectricalMonitoring::rpiRunning() {
    return rpiRunning_ && rpiPowerOn_;
}



void ElectricalMonitoring::init() {
    initialize();
}

void ElectricalMonitoring::run() {

    bool printWarning = true; //Set to false to disable warning message. Disable this if you know what you are doing!

    if (!config::electrical_monitoring_thread_enable) {
        while (1) {

            if (!printWarning)
                suspendCallerUntil(END_OF_TIME);

            PRINTF("Electrical Monitoring Thread disabled! PLEASE REENABLE!\n");
            suspendCallerUntil(NOW() + 1 * SECONDS);

        }
    }

    while (1) {

        update();

        suspendCallerUntil(NOW() + config::electrical_monitoring_thread_period*MILLISECONDS);

    }
    
}


ElectricalMonitoring electricalMonitor(RODOS::GPIO_PIN::GPIO_050, RODOS::GPIO_PIN::GPIO_000, RODOS::GPIO_PIN::GPIO_058, RODOS::ADC_CHANNEL::ADC_CH_009, RODOS::PWM_IDX::PWM_IDX13, RODOS::I2C_IDX::I2C_IDX2);
