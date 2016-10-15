#ifndef __WHEEL_SPEED_H__
#define __WHEEL_SPEED_H__

#include "stm32f4xx.h"
#include <stdio.h>
#include "Prepare_data.h"

#define SPEED_FILTER_NUM 10
#define SPEED_OUT_MAX 4500

#define NO_OUT 0
#define ALL_OUT 1
#define SIGNAL_OUT 2


#define ENABLE_MOTOR_OUT 1
#define DISABLE_MOTOR_OUT 0

struct Wheel_Info
{
	int16_t start_flag;
	
	float position[4];
	float position_old[4];
	float speed[4];
	int16_t speed_raw[4];

	float Target_speed[4];
	float speed_filter_buff[4][SPEED_FILTER_NUM];
	float out[4];
	
};
struct PID_Info
{
	float Ek;
	float Ek1;
	float P_Part;
	float I_Part;//I_Part+=Ek
	float D_Part;//D_Part=Ek-Ek1;
	float Target;
	float Out;
};

struct PID_PARA
{
	float core_P;
	float core_I;
	float core_D;
	float shell_P;
	float shell_I;
	float shell_D;
	
};
extern struct Wheel_Info Four_Wheel_Info;
extern struct PID_PARA Chassis_para;
extern struct PID_Info Chasis_Angular;
uint8_t Wheel_Speed_control(uint8_t flag);
void Wheel_out(uint8_t out_mode , float * speed_list );

#endif

