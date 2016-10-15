#include "Holder.h"
#include "Motor_out.h"



//2016.4.17

struct Hold_Info Pitch_Hold_Info;
struct Hold_Info yaw_Hold_Info;	


struct PID_PARA Pitch_para = 
{
	250 , 0 , 60,
	110 , 0 , 0
}	;

//struct PID_PARA Pitch_para = 
//{
//	0 , 0 , 0,
//	0 , 0 , 0
//}	;






struct PID_PARA Yaw_para = 
{
	250,0,50,
	160,0,0
};


int8_t Holder_Pitch_Control(uint8_t flag)
{
	
	float core_delta , core_p_part;
	float shell_delta , shell_p_part , shell_d_part;
	
	
	//1. shell control
	shell_delta = Pitch_Hold_Info.angle_target- Pitch_Hold_Info.angle;
	
	shell_p_part = shell_delta * Pitch_para.shell_P;
	
	shell_d_part = Pitch_Hold_Info.angle_speed * Pitch_para.shell_D * 0.01f;//D SHELL
	
	Pitch_Hold_Info.shell_out = shell_p_part + shell_d_part;
	
	
	//2. core control
	
	core_delta = Pitch_Hold_Info.shell_out - Pitch_Hold_Info.angle_speed;
	
	core_p_part = core_delta * 0.01f * Pitch_para.core_P;
	
	
	
	if( flag == 1 )
	{
		Pitch_Hold_Info.out = core_p_part + 1000;
	}
	else
	{
		Pitch_Hold_Info.out = 0;
	}
	
	
	
	return 1;
	
}

int8_t Holder_Yaw_Control(uint8_t flag)
{
	
	float core_delta , core_p_part;
	float shell_delta , shell_p_part , shell_d_part;
	
	
	//1. shell control
	shell_delta = yaw_Hold_Info.angle_target - yaw_Hold_Info.angle;
	
	shell_p_part = shell_delta * Yaw_para.shell_P;
	
	shell_d_part = yaw_Hold_Info.angle_speed * Yaw_para.shell_D * 0.01f;//D SHELL
	
	yaw_Hold_Info.shell_out = shell_p_part + shell_d_part;
	
	
	//2. core control
	
	core_delta = yaw_Hold_Info.shell_out - yaw_Hold_Info.angle_speed;
	
	core_p_part = core_delta * 0.01f * Yaw_para.core_P;
	
	
	
	if( flag == 1 )
	{
		yaw_Hold_Info.out = core_p_part;
	}
	else
	{
		yaw_Hold_Info.out = 0;
	}

	return 1;
	
}






