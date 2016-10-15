#include "main.h"

/*-PWM1---(PA0---TIM5_CH1)--*/   //Ä¦²ÁÂÖ
/*-PWM2---(PA1---TIM5_CH2)--*/   //Ä¦²ÁÂÖ
/*-PWM3---(PA1---TIM5_CH3)--*/   //²¦µ¯µç»ú
void PWM_Configuration(void)
{
		GPIO_InitTypeDef          GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef   TIM_TimeBaseStructure;
    TIM_OCInitTypeDef         TIM_OCInitStructure;
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

	  GPIO_PinAFConfig(GPIOA,GPIO_PinSource0, GPIO_AF_TIM5);
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource1, GPIO_AF_TIM5);    
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource2, GPIO_AF_TIM5);  
	
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOA,&GPIO_InitStructure);

    TIM_TimeBaseStructure.TIM_Prescaler = 84-1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period = 2500;   //2.5ms
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM5,&TIM_TimeBaseStructure);
    
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputState_Disable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Set;
    TIM_OC1Init(TIM5,&TIM_OCInitStructure);
    TIM_OC2Init(TIM5,&TIM_OCInitStructure);
    TIM_OC3Init(TIM5,&TIM_OCInitStructure);
		
    TIM_OC1PreloadConfig(TIM5,TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM5,TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM5,TIM_OCPreload_Enable);
		
    TIM_ARRPreloadConfig(TIM5,ENABLE);
    
    TIM_Cmd(TIM5,ENABLE);
}
