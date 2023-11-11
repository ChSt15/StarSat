#include "rodos.h"	

#include "hbridge.hpp"


HBridge::HBridge(RODOS::PWM_IDX pwm1, RODOS::PWM_IDX pwm2): 
    pwm1(pwm1), pwm2(pwm2)
{
    // ############# CTOR TO BE PROPERLY IMPLEMENTED!!!!! #############
}


void HBridge::setPower(float power)
{

}


HBridge hbridge;