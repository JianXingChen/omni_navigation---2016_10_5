
#include "mpu6050.h"
#include "delay.h"

struct _sensor sensor;	
	
u8 mpu6050_buffer[14];					

/***************************************************************************************
  * @函数描述：  MPU6050初始化
  * @入口参数：  无.
  * @返回值  :   初始化完成标志，0----完成，!0----未完成.
****************************************************************************************/
u8 InitMPU6050(void)
{
	unsigned char Addr;
	u8 date,count = 0;
	do
	{
		Addr=Single_Read(MPU6050_ADDRESS,WHO_AM_I); count++;
		if(count>8) return 0;                                      //MPU6050连接失败
	}while(Addr!=0x68);
	delay_ms(2);
	
	date = Single_Write(MPU6050_ADDRESS, PWR_MGMT_1, 0x00);  	   //解除休眠状态0x00
	delay_ms(50);
	date += Single_Write(MPU6050_ADDRESS, SMPLRT_DIV, 0x00);     //采样频率（1KHz）
	delay_ms(50);
	date += Single_Write(MPU6050_ADDRESS, CONFIG, 0x01);         //低通滤波0x00
	delay_ms(50);
	date += Single_Write(MPU6050_ADDRESS, GYRO_CONFIG, 0x10);    //陀螺仪量程 
	delay_ms(50);
	date += Single_Write(MPU6050_ADDRESS, ACCEL_CONFIG, 0x09);   //加速度量程 
	delay_ms(50);
	
//	count = 0;
//	do
//	{
//		Addr=Single_Read(MPU6050_ADDRESS2,WHO_AM_I); count++;
//		if(count>8) return 0;                                      //MPU6050连接失败
//	}while(Addr!=0x68);
//	date += Single_Write(MPU6050_ADDRESS, PWR_MGMT_1, 0x01);  	   //解除休眠状态
//	date += Single_Write(MPU6050_ADDRESS, SMPLRT_DIV, 0x00);     //采样频率（1KHz）
//	date += Single_Write(MPU6050_ADDRESS, CONFIG, 0x01);         //低通滤波
//	date += Single_Write(MPU6050_ADDRESS, GYRO_CONFIG, 0x08);    //陀螺仪量程 
//	date += Single_Write(MPU6050_ADDRESS, ACCEL_CONFIG, 0x08);   //加速度量程 
	
	return date;
}
/***************************************************************************************
  * @函数描述：  读取所有加速度计、陀螺仪
  * @入口参数：  无.
  * @返回值  :   无.
****************************************************************************************/
void MPU6050_Read(void)
{
	
	mpu6050_buffer[8]=Single_Read(MPU6050_ADDRESS, GYRO_XOUT_H);
	mpu6050_buffer[9]=Single_Read(MPU6050_ADDRESS, GYRO_XOUT_L);
	
	
	mpu6050_buffer[10]=Single_Read(MPU6050_ADDRESS, GYRO_YOUT_H);
	mpu6050_buffer[11]=Single_Read(MPU6050_ADDRESS, GYRO_YOUT_L);
	
	mpu6050_buffer[12]=Single_Read(MPU6050_ADDRESS, GYRO_ZOUT_H);
	mpu6050_buffer[13]=Single_Read(MPU6050_ADDRESS, GYRO_ZOUT_L);
}


/***************************************************************************************
  * @函数描述：  陀螺仪零点校准
  * @入口参数：  无.
  * @返回值  :   无.
****************************************************************************************/

void Gyro_OFFEST(void)
{
	
	int cnt_g=1000;
	 int cnt = cnt_g;
	 float  tempgx=0,tempgy=0,tempgz=0;
	 sensor.gyro.averag.x=0;    //零点偏移清零
	 sensor.gyro.averag.y=0;  
	 sensor.gyro.averag.z=0;
	 while(cnt_g--)       //循环采集1000次   求平均
	 {
		  MPU6050_Read();
		 
		  sensor.gyro.origin.x = ((((int16_t)mpu6050_buffer[8]) << 8) | mpu6050_buffer[9]);
	    sensor.gyro.origin.y = ((((int16_t)mpu6050_buffer[10]) << 8)| mpu6050_buffer[11]);
	    sensor.gyro.origin.z = ((((int16_t)mpu6050_buffer[12]) << 8)| mpu6050_buffer[13]);
      tempgx+= sensor.gyro.origin.x;
			tempgy+= sensor.gyro.origin.y;
			tempgz+= sensor.gyro.origin.z;
   }
	 sensor.gyro.quiet.x=tempgx/cnt;
	 sensor.gyro.quiet.y=tempgy/cnt;
	 sensor.gyro.quiet.z=tempgz/cnt;
	
}
/***************************************************************************************
  * @函数描述：  读取陀螺仪原始数据
  * @入口参数：  待获取轴.
  * @返回值  :   当前角速度.
****************************************************************************************/
void Get_Gyro_data(float *G_Y,float *G_Z)
{
		int i;  float sum=0;
		static float GyroY_data[4]={0,0,0,0},GyroZ_data[4]={0,0,0,0};
	
		MPU6050_Read();
		sensor.gyro.origin.x  = ((((int16_t)mpu6050_buffer[8]) << 8) | mpu6050_buffer[9]) - sensor.gyro.quiet.x;
		sensor.gyro.origin.y  = ((((int16_t)mpu6050_buffer[10]) << 8)| mpu6050_buffer[11]) - 6;
		sensor.gyro.origin.z  = ((((int16_t)mpu6050_buffer[12]) << 8)| mpu6050_buffer[13]) - 62;
		
		if((sensor.gyro.origin.y>-30)&(sensor.gyro.origin.y<30)) sensor.gyro.origin.y=0;
		if((sensor.gyro.origin.z>-30)&(sensor.gyro.origin.z<30)) sensor.gyro.origin.z=0;
		
		for(i=3;i>=1;i--)
		{
			GyroZ_data[i] = GyroZ_data[i-1];
			sum+=GyroZ_data[i];
		}
		GyroZ_data[0] = sensor.gyro.origin.z;
		*G_Z = (sum + GyroZ_data[0]) / 4;
		
		for(i=3;i>=1;i--)
		{
			GyroY_data[i] = GyroY_data[i-1];
			sum+=GyroY_data[i];
		}
		GyroY_data[0] = sensor.gyro.origin.y;
		*G_Y = (sum + GyroY_data[0]) / 4;
		
}
