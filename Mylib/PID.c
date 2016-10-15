
#include "PID.h"
#include "USART3.h"

/***************************************************************************************
  * @函数描述：  限幅.
  * @入口参数：  无.
  * @返回值  :   无.
****************************************************************************************/
float AmplitudeCheck(float Angle)
{
	if(Angle>4900)  
		Angle=4900;
	if(Angle<-4900)  
		Angle=-4900;
	return Angle;
}

/***************************************************************************************
  * @函数描述：  PID处理函数.
  * @入口参数：  FeedBack----反馈值，Target----给定值，PID----待运算PID结构体，
                 Inner----1,内环；0，外环.
  * @返回值  :   无.
****************************************************************************************/
void PID_PosControl(float FeedBack,float Target,struct PID_InitTypeDef *PID)
{
//	if(Inner==1)
//	{
//		PID->Kp = Pitch_para.core_P/1000; //   0.52;
//		PID->Ki = Pitch_para.core_I/1000;//  0;
//		PID->Kd = Pitch_para.core_D/1000;//  0;
//	}
//	else if(Inner==0)
//	{
//		PID->Kp=-Pitch_para.shell_P/10;//    120;
//		PID->Ki=-Pitch_para.shell_I/1000;//  0;
//		PID->Kd=-Pitch_para.shell_D/1000;//  0;
//	}
//	else ;
	
	PID->Bias = Target - FeedBack;
/*--------------------------  增量式PID  --------------------*/
//	PID->Out += PID->Kp * (PID->Bias - PID->Last_Bias) +	PID->Ki * PID->Bias + PID->Kd *(PID->Bias - 2*PID->Last_Bias + PID->Pre_Bias);
//	PID->Pre_Bias = PID->Last_Bias;
/*-------------------------  绝对式PD  ---------------------*/
	PID->Out = PID->Kp * PID->Bias;
	
	PID->Out = AmplitudeCheck(PID->Out);
}

/***************************************************************************************
  * @函数描述：  PID速度处理函数.
  * @入口参数：  FeBack----反馈值，SET----给定值，PID----待运算PID结构体，
  * @返回值  :   无.
****************************************************************************************/
void  PID_SpeControl(int FeedBack,int Target,struct PID_WheelTypeDef *PID)
{
	
	PID->Bias = Target - FeedBack;
	PID->Out += PID->Kp * (PID->Bias - PID->Last_Bias) +	PID->Ki * PID->Bias;
	PID->Last_Bias = PID->Bias;
	
	AmplitudeCheck(PID->Out);
	
}




