#include "ElectricalMonitoring.hpp"

ElectricalMonitoring::BeeperThread::BeeperThread(RODOS::PWM_IDX beeper) :
    beeper(beeper)
{}

void ElectricalMonitoring::BeeperThread::beepForTime_ns(int64_t time_ns) {

    beeper.write(50);

    beepTime_ns = time_ns;

    this->resume();

}

void ElectricalMonitoring::BeeperThread::init()
{
    beeper.init(200, 100);
    beeper.write(0);
}

void ElectricalMonitoring::BeeperThread::run()
{

    while(1)
    {

        beeper.write(0);

        suspendCallerUntil(END_OF_TIME);
        suspendCallerUntil(NOW() + beepTime_ns);
    
    }

}


ElectricalMonitoring::ElectricalMonitoring(RODOS::GPIO_PIN powerOffPin, RODOS::ADC_CHANNEL adcWheelPin, RODOS::PWM_IDX beeperPin, RODOS::I2C_IDX ina3221_i2cBus) :
    beeperThread(beeperPin),
    extPowerPin_(powerOffPin),
    adcWheelPin_(adcWheelPin),
    beeperIDX_(beeperPin),
    extPower_(powerOffPin),
    adcWheel_(RODOS::ADC_IDX0),
    beeper_(beeperPin),
    i2cBus_(ina3221_i2cBus)
{}

void ElectricalMonitoring::initialize()
{
    extPower_.init(true, 1, 0);
    adcWheel_.init(adcWheelPin_);
}

void ElectricalMonitoring::update()
{   



    switch (state_)
    {
    case SystemState_t::INIT :
        {
            beeperThread.beepForTime_ns(500*MILLISECONDS);
        }
        break;

    case SystemState_t::CONV_INIT :
        /* code */
        break;

    case SystemState_t::CONV_WAIT :
        /* code */
        break;

    case SystemState_t::POWERDOWN_INIT :
        /* code */
        break;

    case SystemState_t::POWERDOWN_WAIT :
        /* code */
        break;

    case SystemState_t::POWERDOWN_COMPLETE :
        /* code */
        break;
    
    default:
        state_ = SystemState_t::INIT;
        break;
    }

}	

void ElectricalMonitoring::readValues(bool setValue) {

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
    //return 0;
}

void ElectricalMonitoring::openExtSwitch() {
    //return 0;
}

bool ElectricalMonitoring::powerGood() {
    return powerGood_;
}

bool ElectricalMonitoring::rpiRunning() {
    return rpiRunning_;
}




//ElectricalMonitoring electricalMonitor;
