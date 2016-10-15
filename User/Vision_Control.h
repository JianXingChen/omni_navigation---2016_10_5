#ifndef __VISION_CONTROL_H__
#define __VISION_CONTROL_H__


#include "stm32f4xx.h"
#include <stdio.h>

#define CAMERA_REV_NUM_MAX 12

struct VISION_INFO
{
	uint8_t rev_buff[CAMERA_REV_NUM_MAX];
	
	
	int16_t Pitch_angle_s16;
	int16_t Yaw_angle_s16;
	
	int32_t Vision_cnt;
	int32_t Vision_cnt_dis;
	
	float Pitch_angle_f;
	float Yaw_angle_f;
	
	float Yaw_inval;
	float Yaw_inval_step;
	
		
};

extern struct VISION_INFO Vision_info;
#endif