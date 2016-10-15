#include "Wheel_Speed.h"
#include "Chassis_Control.h"
#define pi 3.1415926;

CanTxMsg TxMessage;

float delta[4]={0} ;
extern	int get_yaw_flag;	
struct Wheel_Info Four_Wheel_Info;
struct PID_PARA Chassis_para = //从这里面更改外环内环参数 
{
	5000,20,0,//内环速度
	4,0,0     //外环角度
};



uint8_t Wheel_Speed_control(uint8_t flag)
{
struct PID_Info Wheel_Speed;

	static int8_t num = 0;
	int8_t i,wheel_cnt;
	float sum[4];

	/***********NUM1 计算四个轮子的电机输出*************/
	for( wheel_cnt = 0;wheel_cnt<4;wheel_cnt++ )
	{		
		/*(2)计算PID*/
	  Wheel_Speed.Ek = Four_Wheel_Info.Target_speed[wheel_cnt] - Four_Wheel_Info.speed_raw[wheel_cnt];
		delta[wheel_cnt]=Wheel_Speed.Ek;//用来观测差值
		
		Wheel_Speed.P_Part  = Wheel_Speed.Ek /100;//P PART 缩小10倍
	  Wheel_Speed.I_Part += Wheel_Speed.Ek ;
		Wheel_Speed.D_Part  = Wheel_Speed.Ek-Wheel_Speed.Ek1;
		
		Four_Wheel_Info.out[wheel_cnt] = Chassis_para.core_P * Wheel_Speed.P_Part 
		                               + Chassis_para.core_I * Wheel_Speed.I_Part
																	 + Chassis_para.core_D * Wheel_Speed.D_Part;
		
		Wheel_Speed.Ek1=Wheel_Speed.Ek;
		
		Four_Wheel_Info.out[wheel_cnt]=AmplitudeCheck(Four_Wheel_Info.out[wheel_cnt],4000 );
	}
		//输出到电机
		if( flag == DISABLE_MOTOR_OUT)	Wheel_out( NO_OUT , Four_Wheel_Info.out);
		else 	Wheel_out( ALL_OUT , Four_Wheel_Info.out);
		return 1;
}

/*****************************************************
 * *函数名：Wheel_out
 * *描述:      控制四个底盘电机的输出
 * *输入:      out_mode
		可选参数：NO_OUT  : 无电机输出模式
**************ALL_OUT  :所有电机输出模式
**************SIGNAL_OUT	:单个电机输出模式
**输出:无

 ****************************************************/	 
void Wheel_out(uint8_t out_mode , float * speed_list )
{
	int16_t i16_speed_list[4];
	int8_t i;
	uint8_t TransmitMailbox;
	uint16_t t;

	TxMessage.StdId = 0x200;	
	TxMessage.IDE=CAN_ID_STD;					 //STANDARD MODE
  TxMessage.RTR=CAN_RTR_DATA;				 //发送的是数据
  TxMessage.DLC=8;							 //数据长度为2字节

	if( out_mode == ALL_OUT )//所有电机输出模式
	{
		for( i=0;i<4;i++ )
		{
			i16_speed_list[i] = (int16_t)(*(speed_list+i));
			TxMessage.Data[i*2+0] = (uint8_t)(i16_speed_list[i]>>8);
			TxMessage.Data[i*2+1] = (uint8_t)(i16_speed_list[i]);
		}
	}
	else if(out_mode==SIGNAL_OUT) 
	{
		TxMessage.Data[0]=0>>8;
		TxMessage.Data[1]=0;
		TxMessage.Data[2]=0>>8;
		TxMessage.Data[3]=0;
		TxMessage.Data[4]=0>>8;
		TxMessage.Data[5]=0;
		TxMessage.Data[6]=0;
		TxMessage.Data[7]=0;
	}
	else if(out_mode==NO_OUT)	
	{
		TxMessage.Data[0]=0>>8;
		TxMessage.Data[1]=0;
		TxMessage.Data[2]=0>>8;
		TxMessage.Data[3]=0;
		TxMessage.Data[4]=0>>8;
		TxMessage.Data[5]=0;
		TxMessage.Data[6]=0;
		TxMessage.Data[7]=0;
	}
	TransmitMailbox=CAN_Transmit(CAN2,&TxMessage);
	t=0;
	while((CAN_TransmitStatus(CAN2,TransmitMailbox)!=CANTXOK)&&(t!= 0xFF))
	{
		t++;
	}
}

