#ifndef __CHASSIS_CONTROL_H__
#define __CHASSIS_CONTROL_H__


#include "stm32f4xx.h"
#include <stdio.h>

struct Chassis_Info
{
	float Chassis_angle;
	int16_t Chassis_angle_raw;
	float Chassis_out;

};

extern struct Chassis_Info Chassis_Control_Info;



int Usart_Chassis_Control(uint8_t flag,float liner_x,float liner_y,float angular_z  );
int8_t Remote_Chassis_Control(uint8_t flag);
float AmplitudeCheck(float Angle , float limit);
float yaw_back(void );


#endif



