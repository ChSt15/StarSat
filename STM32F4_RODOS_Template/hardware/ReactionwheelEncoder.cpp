#include "ReactionwheelEncoder.hpp"

#include "rodos.h"
#include "math.h"
#include "stdint.h"
#include "stm32f4xx_conf.h"


Topic<TimestampedData<float>> EncoderDataTopic(-1, "EncoderData");

ReactionwheelEncoder::ReactionwheelEncoder()
{

}

/* Private variables ---------------------------------------------------------*/
__IO uint32_t IC4ReadValue1 = 0, IC4ReadValue2 = 0, Capture = 0;
__IO uint8_t CaptureNumber = 0;
__IO uint32_t TIM2Freq = 0;
__IO uint8_t EncoderB;
__IO double CaptureTime;

void ReactionwheelEncoder::Init()
{
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    TIM_ICInitTypeDef  TIM_ICInitStructure;

    /* TIM2 clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    /* GPIOA clock enable */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

    /* TIM2 channel 4 pin (PA3) configuration for Encoder A (Yellow)*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* Connect TIM pins to AF2 */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_TIM2);

    /* Configure (PA5) pin as input floating for Encoder B (White)*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* -----------------------------------------------------------------------
       TIM2 Configuration:

       In this example TIM2 input clock (TIM2CLK) is set to 2 * APB1 clock (PCLK1):
           TIM2CLK = SystemCoreClock / 2 = 84000000 Hz

       To get TIM2 counter clock at X Hz, the prescaler is computed as follows:
           Prescaler = (TIM3CLK / TIM3 counter clock) - 1
           Prescaler = ((SystemCoreClock /2) / X Hz) - 1

       Note:
       SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f4xx.c file.
      ----------------------------------------------------------------------- */
      //TIM_PrescalerConfig(TIM2, (uint16_t) (((SystemCoreClock/2) / X) - 1), TIM_PSCReloadMode_Immediate);

      /* Enable the TIM2 global Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* TIM2 configuration: Input Capture mode ---------------------
       The external signal is connected to TIM2 CH4 pin (PA3)
       The Rising edge is used as active edge,
       The TIM2 CCR4 is used to compute the frequency value
    ------------------------------------------------------------ */
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV8;		/*!< Capture performed once every 8 events. */
    TIM_ICInitStructure.TIM_ICFilter = 0x0;

    TIM_ICInit(TIM2, &TIM_ICInitStructure);

    /* TIM enable counter */
    TIM_Cmd(TIM2, ENABLE);

    /* Enable the CC2 Interrupt Request */
    TIM_ITConfig(TIM2, TIM_IT_CC4, ENABLE);
}

extern "C" 
{
    /**
      * @brief  This function handles TIM2 global interrupt request.
      * @param  None
      * @retval None
      */
    void TIM2_IRQHandler(void)
    {
        if (TIM_GetITStatus(TIM2, TIM_IT_CC4) == SET)
        {
            /* Clear TIM2 Capture compare interrupt pending bit */
            TIM_ClearITPendingBit(TIM2, TIM_IT_CC4);
            CaptureTime = NOW();
            if (CaptureNumber == 0)
            {
                /* Get the Input Capture value */
                IC4ReadValue1 = TIM_GetCapture4(TIM2);
                CaptureNumber = 1;
            }
            else if (CaptureNumber == 1)
            {
                EncoderB = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_5);
                /* Get the Input Capture value */
                IC4ReadValue2 = TIM_GetCapture4(TIM2);

                /* Capture computation */
                if (IC4ReadValue2 > IC4ReadValue1)
                {
                    Capture = (IC4ReadValue2 - IC4ReadValue1);
                }
                else if (IC4ReadValue2 < IC4ReadValue1)
                {
                    Capture = ((0xFFFFFFFF - IC4ReadValue1) + IC4ReadValue2);
                }
                /* Frequency computation */
                TIM2Freq = (uint32_t)((SystemCoreClock / 2)) * 8 / Capture;
                CaptureNumber = 0;
            }
        }
    }
}


TimestampedData<float>& ReactionwheelEncoder::getSpeed()
{
    double SensorTime = ((NOW() - CaptureTime) / (double)MILLISECONDS);
    if (SensorTime > 250) //minimum measured speed is 2 RPS(120 RPM). This can give us 250ms of minimum interval between interrupts (2 interrupts every one revolution).
    {
        TIM2Freq = 0;
    }

    Speed.timestamp = SECONDS_NOW();
    if (EncoderB)
    {
        Speed.data = -1 * ((float)TIM2Freq / 16) * 2 * 3.1415;  //CCW
    }
    else { Speed.data = ((float)TIM2Freq / 16) * 2 * 3.1415; }  //CW

    return Speed;
}


ReactionwheelEncoder encoder;