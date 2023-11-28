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


ElectricalMonitoring::ElectricalMonitoring(RODOS::GPIO_PIN powerOffPin, RODOS::ADC_CHANNEL adcVBATPin, RODOS::ADC_CHANNEL adcShuntPin, RODOS::ADC_CHANNEL adcWheelPin, RODOS::PWM_IDX beeperPin) :
    beeperThread(beeperPin),
    extPowerPin_(powerOffPin),
    adcVBATPin_(adcVBATPin),
    adcShuntPin_(adcShuntPin),
    adcWheelPin_(adcWheelPin),
    beeperIDX_(beeperPin),
    extPower(powerOffPin),
    adcVBAT(RODOS::ADC_IDX0),
    adcShunt(RODOS::ADC_IDX0),
    adcWheel(RODOS::ADC_IDX0),
    beeper(beeperPin)
{}

void ElectricalMonitoring::initialize()
{
    extPower.init(true, 1, 0);
    adcVBAT.init(adcVBATPin_);
    adcShunt.init(adcShuntPin_);
    adcWheel.init(adcWheelPin_);
}

void ElectricalMonitoring::update()
{   

    float lpf_alpha = 0.1f;
    
    if (firstRun) {
        firstRun = false;

        beeperThread.beepForTime_ns(500*MILLISECONDS);

        lpf_alpha = 1.0f; // no LPF on first run

    }

    // Read ADCs
    //float adcVBAT_raw = adcVBAT.read();
    //float adcShunt_raw = adcShunt.read();
    

}	



ElectricalMonitoring electricalMonitor;
