
//2016.4.17
#ifndef __HOLDER_H_
#define __HOLDER_H_
#include<stdint.h>
#include "stm32f4xx.h" 

#define GYRO_FILTER_NUM 10

struct Hold_Info
{
	
	uint8_t angle_flag ;
	
	int16_t angle_speed_16;
	int16_t angle_speed_buff[GYRO_FILTER_NUM];
		
	int16_t can_angle_raw;
	
	float can_angle;
	
	float angle;
	float angle_temp;
	float angle_old;
	
	float angle_delta_interal;
	
	float angle_speed;
	float angle_speed_delta_interal;
	
	float angle_target;
	float angle_target_old;
	
	
	float shell_out;
	float out;
	
};
extern struct Hold_Info Pitch_Hold_Info;
extern struct Hold_Info yaw_Hold_Info;	



	
extern struct PID_PARA Pitch_para;
extern struct PID_PARA Yaw_para;


int8_t Holder_Pitch_Control(uint8_t flag);
int8_t Holder_Yaw_Control(uint8_t flag);




#endif

