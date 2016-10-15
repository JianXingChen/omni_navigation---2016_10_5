#include "Prepare_data.h"
#include "6050.h"
#include "Holder.h"
#include "USART3.h"


uint8_t mpu6050_error_flag = 0;

void Prepare_data(void)
{
	static int16_t gyro_filter_cnt = 0;
	
	uint8_t i ;
	uint8_t Addr1,Addr2;
	
	float sum_pitch , sum_yaw;
	
	Addr1 = Single_Read(MPU6050_ADDRESS,WHO_AM_I); //count++;
	if( Addr1 != 0x68 )
	{
		mpu6050_error_flag = 1;
		
		return;
	}
	
//	Addr2 = Single_Read(MPU6050_ADDRESS2,WHO_AM_I); //count++;
//	if( Addr2 != 0x68 )
//	{
//		mpu6050_error_flag = 1;
//		
//		return;
//	}
	
	//Send_data[3] = Addr1 + Addr2 ;
	
	
	
	
	MPU6050_Read();
	
	
	//1. get angle speed
	Pitch_Hold_Info.angle_speed_16 =  ((((int16_t)mpu6050_buffer[10]) << 8) | mpu6050_buffer[11]) - sensor.gyro.quiet.y;// pitch angle speed
	Pitch_Hold_Info.angle_speed_buff[gyro_filter_cnt] = -Pitch_Hold_Info.angle_speed_16 ;
	
	
	yaw_Hold_Info.angle_speed_16 =  - (((((int16_t)mpu6050_buffer[12]) << 8) | mpu6050_buffer[13]) - sensor.gyro.quiet.z);// pitch angle speed
	yaw_Hold_Info.angle_speed_buff[gyro_filter_cnt] = yaw_Hold_Info.angle_speed_16 ;
	
	
	
	//slide filter
	sum_pitch = 0;
	sum_yaw = 0;
	
	for( i = 0;i < GYRO_FILTER_NUM;i++ )
	{
		sum_pitch += Pitch_Hold_Info.angle_speed_buff[i];
		sum_yaw += yaw_Hold_Info.angle_speed_buff[i];
		
		
	}
	Pitch_Hold_Info.angle_speed = sum_pitch / GYRO_FILTER_NUM;
	yaw_Hold_Info.angle_speed = sum_yaw / GYRO_FILTER_NUM;
	
	if(  abs( Pitch_Hold_Info.angle_speed ) < GYRO_GAP)
	{
		Pitch_Hold_Info.angle_speed = 0;
	}
	
	if(  abs( yaw_Hold_Info.angle_speed ) < GYRO_GAP)
	{
		yaw_Hold_Info.angle_speed = 0;
	}
	
	
	
	gyro_filter_cnt = ( gyro_filter_cnt + 1 ) % GYRO_FILTER_NUM;
	
	//2. get angle
	//2.1 PITCH ANGLE

//	if( Pitch_Hold_Info.can_angle_raw > 4000 )
//	{
//		Pitch_Hold_Info.can_angle_raw = Pitch_Hold_Info.can_angle_raw - 8192;
//	}
	
	Pitch_Hold_Info.can_angle = 0.04132f* (Pitch_Hold_Info.can_angle_raw  - 640) ;
	Pitch_Hold_Info.angle = Pitch_Hold_Info.can_angle;
	
	//2.2 YAW ANGLE
	yaw_Hold_Info.angle += ( yaw_Hold_Info.angle_speed * 0.00003272f  );
	
	
}

