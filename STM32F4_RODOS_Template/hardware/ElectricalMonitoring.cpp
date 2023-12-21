#include "ElectricalMonitoring.hpp"

ElectricalMonitoring::BeeperThread::BeeperThread(RODOS::PWM_IDX beeper) :
    beeper(beeper)
{}

void ElectricalMonitoring::BeeperThread::beepForTime_ns(int64_t time_ns) {

    beepSem_.enter();
    beepTime_ns = time_ns + NOW();
    beepSem_.leave();

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
        int64_t val;
        beepSem_.enter();
        val = beepTime_ns;
        beepSem_.leave();

        if (val > NOW()) {
            beeper.write(100);
        } else {
            beeper.write(0);
        }

        suspendCallerUntil(NOW() + 1*MILLISECONDS);
        //suspendCallerUntil(NOW() + beepTime_ns);
    
    }

}


ElectricalMonitoring::ElectricalMonitoring(RODOS::GPIO_PIN chipPowerPin, RODOS::GPIO_PIN powerOffPin, RODOS::ADC_CHANNEL adcWheelPin, RODOS::PWM_IDX beeperPin, RODOS::I2C_IDX ina3221_i2cBus) :
    ina3221_(INA3221_ADDR41_VCC),
    beeperThread_(beeperPin),
    chipPowerPin_(chipPowerPin),
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
    state_ = SystemState_t::INIT;
}

void ElectricalMonitoring::update()
{   
    //Main state machine
    switch (state_)
    {
    case SystemState_t::INIT :
        {
            beeperThread_.beepForTime_ns(500*MILLISECONDS);
            chipPower_.init(true, 1, 1);
            extPower_.init(true, 1, 1);
            adcWheel_.init(adcWheelPin_);

            ina3221_.begin(&i2cBus_);
            ina3221_.reset();

            // Set shunt resistors for all channels
            ina3221_.setShuntRes(SHUNT_RESISTOR, SHUNT_RESISTOR, SHUNT_RESISTOR);

            readValues(true);
            state_ = SystemState_t::CONV_INIT;
        }
        break;

    case SystemState_t::CONV_INIT :
        
        {
            //Read values
            readValues(false);

            //Check if power should be turned off
            if (voltageBattery_ < batteryCutoffVoltage) {
                openExtSwitch();
            }

            //Check if battery is low
            if (voltageBattery_ < batteryWarningVoltage) {
                beeperThread_.beepForTime_ns(100*MILLISECONDS);
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

            if (voltageBattery_ < batteryCutoffVoltage) {
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
            
            chipPower_.write(0); //Power off
            chipResetStartTime_ns_ = NOW();
            chipResetState_ = ChipResetState_t::CHIP_WAIT;
        }

        break;

    case ChipResetState_t::CHIP_WAIT:
        {
            if (NOW() - chipResetStartTime_ns_ > CHIP_RESET_WAIT_TIME) {
                chipPower_.write(1); //Power on
                chipResetState_ = ChipResetState_t::IDLE;
            }
        }

        break;

    default:
        {
            chipPower_.write(1); //Power on
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
    powerGood_ = (voltage5VBus_ < 5.0f + BUS_VOLTAGE_TOLERANCE) < (voltage5VBus_ >  5.0f - BUS_VOLTAGE_TOLERANCE);

    //Check if RPI is running
    rpiRunning_ = (currentRPI_ > RPI_RUNNING_CURRENT);

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
    //Adios
/*
⠄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣠⣶⣾⣿⣶⣿⡄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣠⣴⣿⣟⣻⣽⣯⣘⣿⣷⡄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣠⣴⣿⣿⣿⣿⣿⠿⠿⠿⣿⣿⣻⣿⡄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣀⣴⠎
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣠⣴⣿⣿⣿⣿⣿⡿⠛⠁⠀⠀⠀⠙⣿⣿⣽⣿⣄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣤⣴⣾⣿⣽⣾
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣠⣾⡿⣭⣷⣿⣿⠟⠁⠀⠀⠀⠀⠀⠀⠀⢸⣿⣷⣿⣿⣆⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣴⣾⣿⣿⣿⣿⣿⠿⠛
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣀⣤⣾⣿⣷⣿⣿⣿⠏⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣿⣿⣶⣿⣿⣀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣴⣾⣿⣿⣿⣿⡿⠟⠉⠀⠀⠀
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣤⣿⣿⣿⣿⣿⣿⣿⠟⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⣿⣿⠸⣿⣿⣷⣶⣶⣤⣀⣀⣀⢀⣠⣴⣿⣿⣿⣿⡿⠟⠉⠀⠀⠀⠀⢀⣶
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣼⣿⣿⣿⣿⣿⣿⠟⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢨⣿⣿⣿⣿⣿⣿⣶⣼⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⠋⠀⠀⠀⠀⠀⠀⣴⡿⢃
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣼⣿⣿⣿⣿⣿⡿⠃⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣠⣤⣶⣿⣿⣿⠟⠛⠛⠛⠿⢿⣿⣿⣿⣿⣿⣿⣿⠟⠁⠀⠀⠀⠀⠀⠀⢀⣼⠟⡁⢂
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣼⣿⣿⣿⣿⣿⡟⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢠⣿⣿⣿⣿⣿⡟⠉⠀⠀⠀⢀⣴⣿⣿⣟⣿⣿⡿⠛⠁⠀⠀⠀⠀⠀⠀⠀⢀⡾⢁⠒⠨⠄
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣼⣿⣿⣿⣿⣿⡿⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣾⣿⣿⣿⣿⡿⠁⠀⠀⠀⢠⣼⣿⣿⣿⣽⣿⡿⠁⠀⠀⠀⠀⠀⠀⠀⠀⢀⣼⠁⠎⡐⠢⠐
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣠⣴⣿⣿⣿⣿⣿⠟⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢀⣾⣿⣿⣿⣿⠏⠀⠀⠀⠀⣠⣿⣿⢯⣿⣿⣿⠋⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠸⡧⠛⠛⣶⠁⡃
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣠⣤⣾⣿⣿⣿⣿⣿⡿⠋⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⣿⣿⣿⣿⠃⠀⠀⠀⠀⢰⣿⣿⣯⣾⣿⡿⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢠⡞⠱⡈⠔
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⡀⠀⠀⠀⠀⠀⣀⣴⣿⣿⣯⣿⣿⣿⣿⣿⠟⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠿⣿⡿⠋⠀⠀⠀⠀⠀⢾⣿⣿⣷⣿⡿⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠙⢶⣡⠘⠄
⠀⠀⠀⠀⠀⠀⠀⠀⣰⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡿⠁⠀⠀⠀⠀⠀⠀⢀⣠⣴⣶⣶⣄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠻⣿⡿⠟⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠹⣷⣈
⠀⠀⠀⠀⢀⣀⣤⣿⣿⣿⣿⣿⣿⣿⣿⣧⣴⣿⣿⣿⡿⠙⠿⠟⠁⠀⠀⠀⠀⠀⠀⣠⣿⣿⣿⣿⣿⣿⡆⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠘⢻
⠀⠀⠀⣠⣾⣿⣿⣿⣿⣿⡿⠉⠙⣿⣿⣿⣿⣿⣿⣿⣧⣀⠀⠀⠀⠀⠀⠀⠀⠀⣼⣿⣿⣿⣿⣿⣿⣿⡏⠀⠀⠀⠀⠀⠀⠀⣀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣀⣤⣶⣿⣶⣦⣤⣀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈
⢀⣴⣾⣿⣿⣿⣿⣿⣿⡿⠁⠠⠘⣿⣿⣿⣿⣿⣿⣿⣿⣿⣶⣤⡀⠀⠀⠀⠀⢸⣿⣿⣿⣿⣿⣿⣿⣿⣷⢀⣠⣴⣶⡿⢿⠿⡿⢿⡾⣶⣦⣄⡀⠀⠀⠀⠀⠀⢰⣶⣼⣿⣿⣿⣿⣿⣿⣿⣿⣷⡀⠀⠀⠀⠀⠀⠀⠀⠀
⣾⣿⣿⣿⣿⠟⠁⠀⠀⠀⠄⡁⢀⠀⠀⠈⢿⣿⣿⣿⣿⣿⣿⣿⣿⣶⣤⡄⠀⢸⣿⣿⣿⣿⣿⣿⣿⡟⣿⣿⣿⠋⠀⠀⠀⠀⠀⠀⠀⠀⠉⠙⢿⣦⣄⠀⠀⠀⠀⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡇⠀⠀⠀⠀⠀⠀⠀⠀
⣿⣿⣿⡿⠃⠀⠄⡁⣬⣶⣾⣶⣦⡀⠁⡈⠀⠛⢿⢿⣿⣾⣿⣿⢿⣿⣿⣿⣷⣄⠈⠙⠻⠿⠛⠉⠀⠀⠀⢸⣿⡀⠀⠀⠀⢀⠀⠀⠀⠀⠀⠀⠀⠈⠻⣶⡀⠀⠀⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⡇⠀⠀⠀⠀⠀⠀⠀⠀
⣿⣿⣿⠁⠠⠁⣤⣿⣿⣿⣿⣿⣿⣇⠀⢀⠂⠈⣾⡷⣌⣙⠻⢿⣿⣯⢿⣿⣿⣿⣿⣦⣄⠀⠀⠀⠀⠀⠀⠘⣿⣷⣄⣀⣠⣿⣷⣄⠀⠀⠀⢠⣾⣷⡀⢹⣷⠀⠀⠘⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠁⠀⠀⠀⠀⠀⠀⠀⠀
⢸⣿⣿⠀⠐⠈⣿⣿⣿⣿⣿⣿⣿⣿⡆⠀⡀⠂⣽⣿⡹⢮⡷⣮⣔⠻⢿⣮⣟⣻⢿⣿⣿⣿⣶⣄⡀⠀⠀⠀⠀⠙⠛⠛⠛⠁⠘⠻⠿⣷⣾⡿⠿⠋⠀⠀⢿⡆⠀⠀⠙⠻⣿⣿⣿⣿⣿⣿⠟⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀
⢸⣿⣿⠀⠠⠑⣿⣿⣿⣿⣿⣿⣿⡿⠁⠀⠄⠠⢽⣾⡹⢧⡻⣵⢫⣟⣦⣍⡚⡽⢯⣿⣟⣿⣿⣿⣿⣷⣤⣀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣾⡇⠀⠀⠀⠀⠀⠉⠉⠉⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⢸⣿⣿⠀⡐⠀⡀⠿⢿⣿⣿⡿⠟⠁⡀⠁⡀⠂⣿⣷⢫⣏⡷⣝⣻⡼⣞⡽⣻⠶⣍⡻⢿⣿⣿⣿⣿⣿⣿⣿⣷⣤⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣸⡿⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⢸⣿⣿⡀⢄⡐⠀⡀⢀⢀⡀⢀⠠⠐⣀⡐⣠⢠⡝⣾⡹⢮⡽⣛⡶⣝⣮⠷⣏⡿⣭⣻⢭⣻⢿⣿⣯⢿⣿⣿⣿⣿⣿⣷⣦⣠⣤⣤⣤⣤⣄⠀⠀⠀⠐⠋⠁⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⢸⣿⣿⡽⢮⣽⡃⠀⢢⣿⣿⣶⠀⢹⣟⢿⡱⣏⣾⢳⣿⣿⣼⣳⡝⡾⣜⡻⣝⢯⡷⡽⣎⢷⣻⣿⣯⢿⣿⡽⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣦⡀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⣿⣿⣿⠉⠙⣾⣧⠀⠹⠿⠻⠉⢠⣻⠎⠣⠙⠡⣞⡽⣎⣿⣿⣿⣿⣷⣭⢻⡼⣫⣝⣻⢼⡳⣿⣿⣯⢿⣿⡽⣿⣿⣞⣿⣟⣿⣿⣿⣿⣿⣻⢿⣿⣿⣿⣄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⣿⣿⣿⣧⠀⢈⠻⢿⣶⣤⣢⠵⠯⠁⢈⠀⠐⢠⢻⡜⣧⣻⡿⣼⣿⣛⣿⣿⣷⣽⣎⢷⣫⢗⣿⣿⣿⢯⣿⣳⢿⣿⡞⣿⣎⣿⣿⡿⣿⣿⣿⣿⣿⣿⣿⣿⡄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⢹⣿⣿⣿⣧⣀⠠⠀⠉⠁⠁⠌⠣⣄⣷⣿⣿⣿⣿⣽⣖⣯⣿⣳⣿⢧⣿⡗⣿⣟⢿⣿⣷⣏⣾⣿⣿⣿⣿⣭⣿⣿⢞⣿⣎⣿⣿⣟⣽⣧⢆⣉⠛⣿⣿⣿⣿⡄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⣾⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣟⣿⣿⣿⣿⣿⣷⣯⣿⣳⣿⡽⣿⣯⢞⡶⣹⢻⢿⡿⣿⣿⣿⣿⣿⣿⣯⣿⣎⣿⣿⡟⣾⣯⢻⣜⣳⣤⡹⣿⣿⣿⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀
⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣟⣿⣿⣿⣿⣿⣿⣿⡵⣿⡿⣜⡳⢯⣞⡼⣣⡟⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣷⣿⡿⣎⠿⣭⢻⣳⢿⣿⣿⡄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣀
⠈⠙⠿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣽⣿⣿⣿⣿⣿⣿⣿⣷⣽⡳⣞⡽⣣⠿⡼⣭⣛⠿⣿⢿⣿⣿⣿⣿⣿⣿⣿⣭⣛⢾⣹⣎⢿⣿⣿⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣠⣤⣤⣶⣿⣿
⠀⠀⠀⠈⠉⠙⠿⣿⣯⣿⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣾⣯⣿⣿⣿⣿⣿⣿⣷⣯⣗⣻⢵⡳⣭⢟⣮⢳⣏⢿⡻⣿⣿⣿⣿⣶⣭⣳⢳⢮⣛⣿⣿⣿⡆⠀⠀⠀⠀⠀⠀⢩⠳⣦⣄⠀⠀⠀⣿⣿⡹⢛⠟⡻
⠀⠀⠀⠀⠀⠀⠀⠀⠉⠻⢿⣷⣾⣽⣿⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣿⣾⣽⣻⢿⣿⣿⣿⣿⣷⣿⣜⣯⣞⣳⢮⣳⡝⣧⢻⡟⣿⢿⣷⣯⣿⣮⣝⣿⣿⣿⡇⢎⠤⣀⡀⠀⠀⢤⠓⡔⡛⢿⣶⡾⢟⠳⢌⠣⢎⡱
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠉⠉⠛⣿⣿⣿⣿⣿⡿⠿⠿⠿⠿⢿⣿⣿⣿⣿⣿⣿⣿⣿⣿⠿⠿⠿⠿⠿⣿⣿⣿⣾⣷⣿⣞⣧⡟⡽⣎⢷⣛⢿⣿⣿⣿⣿⣿⡏⡜⢢⠱⡘⠦⣙⢢⡙⠴⣉⠖⡤⡙⣌⠳⣈⠳⢌⡒
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣼⣿⣿⣿⣿⠿⠀⠀⠀⠀⠀⠀⠈⠙⢿⣿⡿⠛⠉⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠉⠛⠿⣿⣿⣿⣿⣽⢮⡽⢾⣿⣿⣿⣿⡿⠱⣌⢃⠧⣙⢢⡑⠦⣉⢖⡡⢚⡰⠱⣌⠲⡡⢍⠦⡑
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣿⣿⣿⣿⡿⠀⠀⠀⠀⠀⠀⠀⠀⠀⣴⣿⠃⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠙⠿⣿⣿⣮⣽⣻⣿⣿⡟⣍⠲⡱⢌⢎⠲⡡⢆⡱⠣⢜⢢⡑⢣⠜⣑⢢⢣⡑⢎⡒⢍
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⣿⣿⣿⣿⡇⠀⠀⠀⠀⠀⠀⠀⠀⠀⣿⣏⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠈⠻⣿⣿⣿⣿⣿⡗⡬⡑⢦⡉⢎⡱⡑⢎⠴⣉⠎⡆⡍⢦⡙⢤⢃⠦⣉⠦⣉⠦
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⣿⣿⣿⣿⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣿⣿⡆⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠹⢿⣿⣿⣿⣶⢉⠦⡘⠦⡱⡘⣌⠲⣡⠚⡔⡩⢆⡜⢢⢍⠲⣡⠚⢤⢃
⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⣿⣿⣿⣿⡧⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢸⣿⣿⣶⡄⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⠀⢉⣿⣿⣿⣾⡒⣍⠲⣡⠓⣌⠱⢢⡙⠴⣑⠪⢔⡃⢎⡱⢢⢙⢢⢃*/

    extPower_.init(false, 1, 0); //Open switch via High-Z.

    //Pull-up resistor will do the rest (RIP). Only god knows what happens after this point.

}

bool ElectricalMonitoring::powerGood() {
    return powerGood_;
}

bool ElectricalMonitoring::rpiRunning() {
    return rpiRunning_;
}




ElectricalMonitoring electricalMonitor(RODOS::GPIO_PIN::GPIO_032, RODOS::GPIO_PIN::GPIO_000, RODOS::ADC_CHANNEL::ADC_CH_009, RODOS::PWM_IDX::PWM_IDX13, RODOS::I2C_IDX::I2C_IDX2);
