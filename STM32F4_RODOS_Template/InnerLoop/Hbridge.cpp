#include "rodos.h"	

#include "InnerLoopTopics.hpp"
#include "hbridge.hpp"
#include "math.h"


CommBuffer<TimestampedData<float>> EncoderDataBuffer;
Subscriber EncoderDataSubsciber(EncoderDataTopic, EncoderDataBuffer, "Debug Thread");


HBridge::HBridge(RODOS::PWM_IDX pwm1, RODOS::PWM_IDX pwm2): 
    pwm1(pwm1), pwm2(pwm2)
{
    
}


void HBridge::initialization(int pwmFrequency, int pwmIncrements)
{
    pwm1.init(pwmFrequency, pwmIncrements);
    pwm2.init(pwmFrequency, pwmIncrements);
    this->pwmFrequency = pwmFrequency;
    this->pwmIncrements = pwmIncrements;
}


bool HBridge::setVoltage(float voltagePercentage)
{      
    float old = voltagePercentage;

    // Dynamically limit the output voltage based on the current reaction wheel speed
    voltagePercentage = checkDynamicOutputLimit(voltagePercentage);

    // Check if parameter exceeds absolute limits
    voltagePercentage = checkVoltagePercentage(voltagePercentage);

    //PRINTF(" %.1f%\n", voltagePercentage*100);
    
    // Set increments accordingly
    int increments = pwmIncrements * voltagePercentage;
    if (increments >= 0) {
        pwm1.write(increments);
        pwm2.write(0);
    } else {
        pwm1.write(0);
        pwm2.write(-increments);
    }

    return old != voltagePercentage;
}


float HBridge::checkVoltagePercentage(float voltagePercentage){
    if(voltagePercentage > MAX_OUTPUT_PERCENTAGE){
        return MAX_OUTPUT_PERCENTAGE;
    } else if (voltagePercentage < - MAX_OUTPUT_PERCENTAGE)
    {
        return (- MAX_OUTPUT_PERCENTAGE);
    } else {
        return voltagePercentage;    
    }
}


float HBridge::checkDynamicOutputLimit(float voltagePercentage){

    static bool firstRun = true;

    TimestampedData<float> encoder;
    if (firstRun && !EncoderDataBuffer.getOnlyIfNewData(encoder)){ // If no data is available, return input
        return voltagePercentage;
    }
    firstRun = false;
    
    // Get reaction wheel speed
    EncoderDataBuffer.get(encoder);

    // Calculate the flyback voltage of reaction wheel
    float flybackVoltagePercentage = encoder.data * DYNAMIC_REACTIONWHEEL_VOLTAGE_FACTOR;

    // Limit the input percentage to the dynamic output limit band if outside of it
    if (voltagePercentage > flybackVoltagePercentage + DYNAMIC_OUTPUT_LIMIT_BAND){
        return flybackVoltagePercentage + DYNAMIC_OUTPUT_LIMIT_BAND;
    } else if (voltagePercentage < flybackVoltagePercentage - DYNAMIC_OUTPUT_LIMIT_BAND){
        return flybackVoltagePercentage - DYNAMIC_OUTPUT_LIMIT_BAND;
    }

    // Return the input percentage if inside the dynamic output limit band
    return voltagePercentage;

}



/**
 * @param PWM_IDX01 Timer 1 CH2 - PE11 (168 MHz)
 * @param PWM_IDX02 Timer 1 CH3 - PE13 (168 MHz)
*/
HBridge hbridge(PWM_IDX01, PWM_IDX02);