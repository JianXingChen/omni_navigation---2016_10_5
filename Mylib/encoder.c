#include "encoder.h"
#include "stm32f4xx.h"

//TIM3 connect to encoder,a-b
//TIM3_CH1 ----- PB4
//TIM3_CH2 ----- PB5

void Encoder_Configuration(void)
{
    GPIO_InitTypeDef gpio;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure; 
		TIM_ICInitTypeDef TIM_ICInitStructure; 
	
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
    
    gpio.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOB,&gpio);
    
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource4,  GPIO_AF_TIM3);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource5,  GPIO_AF_TIM3);
	
		TIM_TimeBaseStructure.TIM_Period = 0xFFFF;   //2.5ms
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);
	
    TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
		
		TIM_ICStructInit(&TIM_ICInitStructure);
		TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
		TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
		TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
		TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
		TIM_ICInitStructure.TIM_ICFilter = 0x0;
		TIM_ICInit(TIM3, &TIM_ICInitStructure);
	
		TIM_SetCounter(TIM3,0);
    TIM_Cmd(TIM3, ENABLE);
}
