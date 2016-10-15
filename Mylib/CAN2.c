
#include "main.h"

CanRxMsg RxMessage_205;
CanRxMsg RxMessage_206;
CanRxMsg RxMessage_201;
CanRxMsg RxMessage_202;
CanRxMsg RxMessage_203;
CanRxMsg RxMessage_204;
int Turn = 0;

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
  * @函数描述：  CAN1发送函数.
  * @入口参数：  ID----本地ID，Data----待发送数组地址.
  * @返回值  :   无.
****************************************************************************************/
void CAN2_Send(uint32_t ID,u8 *Data)         
{
	u8 TransmitMailbox=0,i=0;
	CanTxMsg TxMessage;
	
	TxMessage.StdId = ID;            //本地ID
	TxMessage.ExtId = 0x0000;
	TxMessage.IDE = CAN_ID_STD; 
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = 8; 
	
	for(i=0;i<8;i++)
	{
		TxMessage.Data[i] = *Data++; 
	}
	
	TransmitMailbox=CAN_Transmit(CAN2,&TxMessage);
	
	while((CAN_TransmitStatus(CAN2,TransmitMailbox)!=CANTXOK)&&(i!=0xFF))
	{
		i++;
	}
	i=0;
	while((CAN_MessagePending(CAN2,CAN_FIFO0)<1)&&(i!=0xFF))
	{
		i++;
	}

}


/***************************************************************************************
  * @函数描述：  CAN1接收中断函数,依据ID将数据送入不同全局变量
  * @入口参数：  无.
  * @返回值  :   无.
****************************************************************************************/
void CAN2_RX0_IRQHandler(void)
{
  CanRxMsg RxMessage;
  int Cur_Pos_Yaw;
	
  if (CAN_GetITStatus(CAN2,CAN_IT_FMP0)!= RESET)
	{
		CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);
  
		/*------------------------Rx接收设置-------------------------*/
		RxMessage.StdId=0x00;
		RxMessage.IDE =CAN_ID_STD;
		RxMessage.RTR=CAN_RTR_DATA;
		RxMessage.DLC = 8; 
	
		CAN_Receive(CAN2,CAN_FIFO0,&RxMessage);
		
		switch (RxMessage.StdId)
		{
			case 0x205: RxMessage_205=RxMessage; break;
			case 0x206: RxMessage_206=RxMessage; break;
			case 0x201: RxMessage_201=RxMessage; break;
			case 0x202: RxMessage_202=RxMessage; break;
			case 0x203: RxMessage_203=RxMessage; break;
			case 0x204: RxMessage_204=RxMessage; break;
			default: break;
		}
		
		Cur_Pos_Yaw = (short)((RxMessage_205.Data[0]<<8) | RxMessage_205.Data[1]);
		Turn = (2788.0 - Cur_Pos_Yaw)*1300.0/6333.0;
		
		PID_Wheel_201.Enc_New = (short)((RxMessage_201.Data[0] << 8) | RxMessage_201.Data[1]);
		PID_Wheel_202.Enc_New = (short)((RxMessage_202.Data[0] << 8) | RxMessage_202.Data[1]);
		PID_Wheel_203.Enc_New = (short)((RxMessage_203.Data[0] << 8) | RxMessage_203.Data[1]);
		PID_Wheel_204.Enc_New = (short)((RxMessage_204.Data[0] << 8) | RxMessage_204.Data[1]);
   }
}


