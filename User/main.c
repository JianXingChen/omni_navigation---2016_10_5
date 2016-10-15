#include "stm32f4xx.h"
#include "sys.h" 
#include "delay.h"
#include "led.h" 
#include "Remote.h"
#include "CAN2.h"
#include "Wheel_Speed.h"
#include "clock.h"
#include "USART3.h"
#include "mpu6050_interrupt.h"
#include "mpu6050_driver.h"
uint32_t system_micrsecond;   //系统时间 单位ms
int main(void)
{     
	SystemInit();

	delay_init(168);
	LED_Init();
	Remote_Config();
	

	CAN2_Configuration();
	USART3_Configuration(115200);
	Wheel_Speed_control(0);
	TIM4_Int_Init();
	TIM2_Configuration();
	MPU6050_Initialize();  
	MPU6050_IntConfiguration();     
	MPU6050_EnableInt(); 
	delay_ms(1000);
	system_micrsecond = Get_Time_Micros();				
	while(1)
	{

	}
}







