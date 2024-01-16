#include "rodos.h"	
#include "adc.hpp"


ADC::ADC(RODOS::ADC_IDX adc_idx, RODOS::ADC_CHANNEL adc_channel): 
    adc(adc_idx)
{
    this->adc_channel = adc_channel;
}


void ADC::initialization(){
    adc.config(ADC_PARAMETER_RESOLUTION, 12);
    adc.init(this->adc_channel);
}


float ADC::getVoltage(){
    float current_voltage = adc.read(this->adc_channel);
    current_voltage = (current_voltage / resolution) * referenceVoltage;

    return current_voltage;
}


ADC adc(ADC_IDX0, ADC_CH_011);