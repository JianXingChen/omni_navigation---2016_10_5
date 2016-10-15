#include "clock.h"

int32_t clock_cnt = 0;
extern int Remote_flag;
int get_yaw_flag=0;
extern char odm[8];
float enable_getyaw;
void TIM4_Int_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);  
	
  TIM_TimeBaseInitStructure.TIM_Period = 20-1; 	
	TIM_TimeBaseInitStructure.TIM_Prescaler=8400-1;  
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStructure);
	
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE); 
	TIM_Cmd(TIM4,ENABLE); 
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0); 
	NVIC_InitStructure.NVIC_IRQChannel=TIM4_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=2; 
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void TIM4_IRQHandler(void)
{
  int i;

	if(TIM_GetITStatus(TIM4,TIM_IT_Update)==SET) 
	{
			clock_cnt++;	
	  enable_getyaw=Get_Time_Micros();
    enable_getyaw=	enable_getyaw*1.0e-6;//换算成秒
		if(enable_getyaw>5)  get_yaw_flag=1;//等待yaw角度上升到稳定
		if( clock_cnt >= 500)
	  	{
				clock_cnt=0;
				PCout(1)=~PCout(1);
		  }
		
		if(clock_cnt >=200) send_odm_msg();//向上位机发送odm的频率
		if((get_yaw_flag)&&(Remote_flag))//接受到遥控器以及初始化完成在让底盘运动
			{
				if((RC_Ctl.rc.s1==1)&&(RC_Ctl.rc.s2==1))		Usart_Chassis_Control(0,liner.x,liner.y,angular.z); //判断上位机/遥控器控制模式
				else	Remote_Chassis_Control(1);
				Wheel_Speed_control(1);
			}
		else Wheel_Speed_control(0);
}
	TIM_ClearITPendingBit(TIM4,TIM_IT_Update);  
}



void TIM2_Configuration(void)
{
    TIM_TimeBaseInitTypeDef tim;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
    tim.TIM_Period = 0xFFFFFFFF;
    tim.TIM_Prescaler = 84 - 1;	 //1M 的时钟  
    tim.TIM_ClockDivision = TIM_CKD_DIV1;	
    tim.TIM_CounterMode = TIM_CounterMode_Up;  
    TIM_ARRPreloadConfig(TIM2, ENABLE);	
    TIM_TimeBaseInit(TIM2, &tim);

    TIM_Cmd(TIM2,ENABLE);	
}
   

int32_t ms_count = 0;
uint32_t Get_Time_Micros(void)
{
	return TIM2->CNT;
}