
//2016.4.17
#ifndef __REMOTE_H_
#define __REMOTE_H_
#include<stdint.h>
#include "stm32f4xx.h" 
//
//extern int Remote_flag;
#define BSP_USART1_DMA_RX_BUF_LEN	30u

typedef struct
{
	struct
	{
		uint16_t ch0;
		uint16_t ch1;
		uint16_t ch2;
		uint16_t ch3;
		uint8_t s1;
		uint8_t s2;
	}rc;
	struct
	{
		int16_t x;
		int16_t y;
		int16_t z;
		uint8_t press_l;
		uint8_t press_r;
	}mouse;
	struct
	{
		uint16_t v;
	}key;
}RC_Ctl_t;

extern RC_Ctl_t RC_Ctl;


extern unsigned char sbus_rx_buffer[18];

void Remote_Config(void);



#endif

