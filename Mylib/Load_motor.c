#include "Load_motor.h"
#include "Holder.h"
#include "Shoot.h"
#include "Remote.h"
struct PID_PARA Load_motor_para;


	
void Load_Motor_PWM_Configuration(void)//PA2  TIM2 CH3
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);  	//TIM2时钟使能    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_TIM2); //GPIOA2复用为定时器2
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;           //GPIOA2
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
	GPIO_Init(GPIOA,&GPIO_InitStructure);    
	
	TIM_TimeBaseStructure.TIM_Prescaler=84-1;  //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=40000-1;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);
	
		//初始化TIM2 Channel3 PWM模式	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //输出极性:TIM输出比较极性低
	TIM_OC3Init(TIM2, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM2 OC3

	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);  //使能TIM14在CCR3上的预装载寄存器
 
  TIM_ARRPreloadConfig(TIM2,ENABLE);//ARPE使能 
	
	TIM_Cmd(TIM2, ENABLE);  //使能TIM2
	
	
	
}

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
	
		TIM_TimeBaseStructure.TIM_Period = 0xFFFF;  
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


void Load_motor_control(uint8_t flag)
{
	float delta, p_part , out, sum;
	int8_t i;
	

	
	Shoot_Info.Pulse_CNT = -(int16_t)(TIM_GetCounter(TIM3));
	TIM_SetCounter(TIM3,0);
	
	Shoot_Info.speed_buff[Shoot_Info.speed_filter_cnt]= Shoot_Info.Pulse_CNT * 250.0f / 5200.0f;//转换到速度  r/s  5200为一转5200个脉冲
	
	sum = 0;
	for( i=0;i<5;i++ )
	{
		sum += Shoot_Info.speed_buff[i];
	}
	Shoot_Info.Load_Motor_Speed = sum / 5.0f;
	Shoot_Info.speed_filter_cnt =  ( Shoot_Info.speed_filter_cnt + 1 ) % 5;
	
	
	
	delta = 0.7f - Shoot_Info.Load_Motor_Speed;
	
	p_part = delta * 100.0f * Load_motor_para.shell_P;
	
	if( p_part < 500  )
	{
		p_part = 500;
	}
	if( p_part > 5000 )
	{
		p_part = 5000;
	}

	
	Shoot_Info.Load_Motor_Out = p_part;
	
	if( (flag == 1)&&( (Shoot_Info.load_command  == 1)&&(RC_Ctl.rc.s2 == 1) ) )
	{
		TIM_SetCompare3(TIM2,(40000 - Shoot_Info.Load_Motor_Out));
	}
	else
	{
		TIM_SetCompare3(TIM2,40000);
	}
	
}

