
#include "USART3.h"
#include "Wheel_Speed.h"

/*-----USART3_TX-----PB10-----*/
/*-----USART3_RX-----PB11-----*/
/*****************************************************************
*** 接收cmd_vel话题，并将其转化为左右轮转速
*** 发布odm话题，提供粗略的里程计信息
*** 2d的机器人移动平台只会用到3个值linear.x,liner.y与angular.z
    分别代表水平分速度，垂直分速度，与角速度
	   |____|
		||    ||      ^
		 |    |       |                                          <
	  ||____||      | +x方向       <————  liner.y方向            ）  angular.z
		
****	接收上位机程序:共8位，第0位是数据大小，第7位是检验。
****	第1位是liner.x高8,第2位是低8
****  第3位是liner.y高8，第4位是低8
****	第5位是angular.z高8，第6位是低8		
*******************************************************************/


u8 com_data;
int head_flag=0,end_flag=0,n=0,put_flag,k=1;
static	int count=0;
int para_get[8];
int para_rev_tem[8];
geometry_msgs_twist liner;
geometry_msgs_twist  angular;



void USART3_Configuration(int32_t baud)
{
    USART_InitTypeDef usart3;
		GPIO_InitTypeDef  gpio;
    NVIC_InitTypeDef  nvic;

		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_DMA1,ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
		
		GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3);
		GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3); 
		
		gpio.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
		gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOB,&gpio);
		
		usart3.USART_BaudRate = baud;
		usart3.USART_WordLength = USART_WordLength_8b;
		usart3.USART_StopBits = USART_StopBits_1;
		usart3.USART_Parity = USART_Parity_No;
		usart3.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;
    usart3.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_Init(USART3,&usart3);

		USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
		USART_Cmd(USART3,ENABLE);
    
    nvic.NVIC_IRQChannel = USART3_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 3;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
}

void USART3_Sends(char *buf1)		  //字符串发送函数
{
		u8 i=0;
		for(i=0;i<8;i++)
		{	USART_SendData(USART3,buf1[i]);  //发送一个字节
			while(USART_GetFlagStatus(USART3,USART_FLAG_TXE)==RESET){};//等待发送结束
			}
	}
void USART3_Sendb(u8 k)		         //字节发送函数
{		 
			USART_SendData(USART3,k);  //发送一个字节
			while(USART_GetFlagStatus(USART3,USART_FLAG_TXE)==RESET){};//等待发送结束	
} 



void USART3_IRQHandler(void)//接收中断
{    
			if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  //接收中断 (接收寄存器非空) 
			{
			 com_data =USART_ReceiveData(USART3);
				if(com_data==0xA0)   head_flag=1;//判断帧头
				if((com_data==0x06)&&(head_flag==1))
					{
					head_flag=0;
					put_flag=1 ;	
				}
				if(put_flag==1)
					{
						para_rev_tem[count]=com_data;	 //从0x06开始存，共存8个
						count++;
						if(count>=8)	
						{count=0;put_flag=0;}//第七个数据存完关闭存数据标志位
					}		
  		}
			 if((para_rev_tem[7]==0xA1)&&(count==0))  
			 { int i;
				 for(i=1;i<7;i++) para_get[i]=para_rev_tem[i];//校验第8位数			 
       }
		   liner.x   =	para_get[1]*256+para_get[2];
   		 liner.y   =	para_get[3]*256+para_get[4];
			 angular.z =  para_get[5]*256+para_get[6];
			
		  USART_ClearITPendingBit(USART3,USART_IT_RXNE);//清除中断标志位
	}
extern char odm[8];
void send_odm_msg()
{

		USART3_Sendb(0xb0);
		USART3_Sendb(0x08);
		USART3_Sends(odm);
		USART3_Sendb(0xb1);	
}
























































