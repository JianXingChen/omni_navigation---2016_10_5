#ifndef __SHOOT_H_
#define __SHOOT_H_
#include<stdint.h>
#include "stm32f4xx.h" 

struct SHOOT_INFO
{
	uint8_t load_command;
	
	uint16_t speed_filter_cnt;
	
	uint16_t Call_cnt;
	
	short Pulse_CNT;
	
	float Load_Motor_Speed;
	
	float speed_buff[5];
	
	float Load_Motor_Speed_Target;
	
	float Load_Motor_Out;
	
};

extern struct SHOOT_INFO Shoot_Info;




void Shoot_PWM_Configuration(void);
	


void Shoot_Fire(uint16_t speed);







#endif