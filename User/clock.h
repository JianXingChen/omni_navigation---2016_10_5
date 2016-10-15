#ifndef _CLOCK_H
#define _CLOCK_H
#include "sys.h"
#include "6050.h"
#include "Prepare_data.h"
#include "USART3.h"

#include "Holder.h"
#include "Motor_out.h"
#include "Wheel_Speed.h"
#include "Chassis_Control.h"
#include "Remote.h"
#include "Shoot.h"
#include "Load_motor.h"
void TIM4_Int_Init(void);
void TIM2_Configuration(void);
uint32_t Get_Time_Micros(void);

extern int32_t clock_cnt;
#endif
