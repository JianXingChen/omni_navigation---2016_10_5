#ifndef __USART3_H__
#define __USART3_H__


#include "stm32f4xx.h"
#include <stdio.h>
#include "Holder.h"

#define PARA_REV_NUM 17    //?????
#define PRAR_REV_DMA_CNT (DMA1_Stream1->MA_CNDTR)   //DMA?????

#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))
	





typedef struct
{
	uint8_t  rev_state;
	uint16_t data_num_now;
	uint16_t data_num_last;
	uint16_t rev_overtime_cnt;
	
}_PARA_REV;

typedef struct 
{
  float x;
  float y;
  float z;
}geometry_msgs_twist;

extern	geometry_msgs_twist liner;
extern	geometry_msgs_twist  angular;
extern _PARA_REV Para_s_rev;
extern uint8_t para_rev[PARA_REV_NUM];

extern float Send_data[4];

/*********????*******/

void USART3_Configuration(int32_t baud);


uint8_t Detect_referee_rev_dma(uint16_t cnt );//检测裁判系统发送数据
void USART3_Sends(char* buf1);
void USART3_Sendb(u8 k);
void send_odm_msg();


#endif
