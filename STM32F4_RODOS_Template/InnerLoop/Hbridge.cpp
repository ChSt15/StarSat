#include "rodos.h"	

#include "hbridge.hpp"
#include "math.h"


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


void HBridge::setVoltage(float voltagePercentage)
{      
    // Check if parameter exceeds limits
    float voltagePercentageDesired = checkVoltagePercentage(voltagePercentage);
    
    // Set increments accordingly
    uint16_t increments = pwmIncrements * voltagePercentageDesired;
    if (increments >= 0) {
        pwm1.write(increments);
        pwm2.write(0);
    } else {
        pwm1.write(0);
        pwm2.write(-increments);
    }
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



/**
 * @param PWM_IDX01 Timer 1 CH2 - PE11 (168 MHz)
 * @param PWM_IDX02 Timer 1 CH3 - PE13 (168 MHz)
*/
HBridge hbridge(PWM_IDX01, PWM_IDX02);