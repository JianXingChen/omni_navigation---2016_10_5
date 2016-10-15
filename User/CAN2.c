
#include "CAN2.h"
#include "Holder.h"
#include "Chassis_Control.h"
#include "Wheel_Speed.h"
#define abs(x) ((x)>0? (x):(-(x)))


CanRxMsg RxMessage205;
CanRxMsg RxMessage206;

/***********************************************************************************
  * @函数描述：  CAN1通信初始化，波特率1MHz. CAN1_TX----PA12,CAN1_RX-----PA11
  * @入口参数：  无.
  * @返回值  :   无.
***********************************************************************************/
void CAN2_Configuration(void)
{
   CAN_InitTypeDef        can;
    CAN_FilterInitTypeDef  can_filter;
    GPIO_InitTypeDef       gpio;
    NVIC_InitTypeDef       nvic;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_CAN2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_CAN2); 

    gpio.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_12 ;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    GPIO_Init(GPIOB, &gpio);

    nvic.NVIC_IRQChannel = CAN2_RX0_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 0;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    CAN_DeInit(CAN2);
    CAN_StructInit(&can);

    can.CAN_TTCM = DISABLE;
    can.CAN_ABOM = DISABLE;    
    can.CAN_AWUM = DISABLE;    
    can.CAN_NART = DISABLE;    
    can.CAN_RFLM = DISABLE;    
    can.CAN_TXFP = ENABLE;     
    can.CAN_Mode = CAN_Mode_Normal; 
    can.CAN_SJW  = CAN_SJW_1tq;
    can.CAN_BS1 = CAN_BS1_9tq;
    can.CAN_BS2 = CAN_BS2_4tq;
    can.CAN_Prescaler = 3;   //CAN BaudRate 42/(1+9+4)/3=1Mbps
    CAN_Init(CAN2, &can);
    
    can_filter.CAN_FilterNumber=14;
    can_filter.CAN_FilterMode=CAN_FilterMode_IdMask;
    can_filter.CAN_FilterScale=CAN_FilterScale_32bit;
    can_filter.CAN_FilterIdHigh=0x0000;
    can_filter.CAN_FilterIdLow=0x0000;
    can_filter.CAN_FilterMaskIdHigh=0x0000;
    can_filter.CAN_FilterMaskIdLow=0x0000;
    can_filter.CAN_FilterFIFOAssignment=0;//the message which pass the filter save in fifo0
    can_filter.CAN_FilterActivation=ENABLE;
    CAN_FilterInit(&can_filter);
    
    CAN_ITConfig(CAN2,CAN_IT_FMP0,ENABLE);

}

/***************************************************************************************
  * @函数描述：  CAN1接收中断函数,依据ID将数据送入不同全局变量
  * @入口参数：  无.
  * @返回值  :   无.
****************************************************************************************/
char odm[8];

void CAN2_RX0_IRQHandler(void)
{
  CanRxMsg RxMessage;
		int i;
  if (CAN_GetITStatus(CAN2,CAN_IT_FMP0)!= RESET)
	{
	
		CAN_Receive(CAN2,CAN_FIFO0,&RxMessage);

	if ((RxMessage.StdId >= 0x201)&&( RxMessage.StdId <= 0x204 ))
		{
	 Four_Wheel_Info.position[RxMessage.StdId-0x201] = RxMessage.Data[0]*256+RxMessage.Data[1];
   Four_Wheel_Info.speed_raw[RxMessage.StdId-0x201] = RxMessage.Data[2]*256+RxMessage.Data[3];
	 Four_Wheel_Info.speed_raw[RxMessage.StdId-0x201] = Four_Wheel_Info.speed_raw[RxMessage.StdId-0x201] /19;
			
	 odm[(RxMessage.StdId-0x201)*2]= RxMessage.Data[2];
   odm[(RxMessage.StdId-0x201)*2+1]  = RxMessage.Data[3];			

}

   }
	 		CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);
}
