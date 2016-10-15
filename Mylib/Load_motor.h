#ifndef __LOAD_MOTOR_H_
#define __LOAD_MOTOR_H_
#include<stdint.h>
#include "stm32f4xx.h" 



extern struct PID_PARA Load_motor_para;



void Load_Motor_PWM_Configuration(void);

void Encoder_Configuration(void);

void Load_motor_control(uint8_t flag);


#endif