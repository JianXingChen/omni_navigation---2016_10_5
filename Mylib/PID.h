#include "stm32f4xx.h" 
#include <stdio.h>

struct PID_InitTypeDef      
{
	float Kp;               //Kp参数
	float Ki;               //Ki参数
	float Kd;
	float Bias;             //当次误差
	float Last_Bias;  
	
	float Target;       //目标
	int16_t Out;          //输出
};

struct PID_WheelTypeDef      
{
	float Kp;               //Kp参数
	float Ki;               //Ki参数
	float Bias;             //当次误差
	float Last_Bias;  
	int Enc_New;
	int Enc_Old;
	int Increment;
	
	int Target;
	int16_t Out;          //输出
};

void  PID_PosControl(float FeedBack,float Target,struct PID_InitTypeDef *PID);
void  PID_SpeControl(int FeedBack,int Target,struct PID_WheelTypeDef *PID);

