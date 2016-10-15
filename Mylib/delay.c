
#include "delay.h"

static u8  fac_us=0;							//usÑÓÊ±±¶³ËÊý			   
static u16 fac_ms=0;							//msÑÓÊ±±¶³ËÊý,ÔÚucosÏÂ,´ú±íÃ¿¸ö½ÚÅÄµÄmsÊ
#define SYSTICK_COUNTFLAG           16
	

void delay_init()
{
	SysTick->CTRL  &=		(~2);				//0xfffffffb;		//bit2Çå¿Õ,Ñ¡ÔñÍâ²¿Ê±ÖÓ  HCLK/8
	
	fac_us=(72000000/1000000)/8;	 //1usÐè¼ÆËãµÄÂö³å¸öÊý,ÆäÖÐ(SystemFrequency/1000000)´ú±í1USµÄÂö³å¸öÊý,8ÊÇÐèÒª8·ÖÆµ		 
	fac_ms=(u16)fac_us*1000;           //1msÐè¼ÆËãµÄÂö³å¸öÊý
}								    

	    								   
void delay_us(u32 nus)
{		
	u32 temp;	    	 
	SysTick->LOAD=nus*fac_us; //Ê±¼ä¼ÓÔØ	  		 
	SysTick->VAL   =  (0x00);                     /* Load the SysTick Counter Value */         //Çå¿Õ¼ÆÊýÆ÷
	SysTick->CTRL = (1<<0);		/* Enable SysTick and SysTick Timer */  //¿ªÊ¼µ¹Êý 	 
	do
		{
		temp=SysTick->CTRL;
		}
	while(temp&0x01&&!(temp&(1<<SYSTICK_COUNTFLAG)));//µÈ´ýÊ±¼äµ½´ï   
	//SysTick->CTRL	&=  (~(1<<SYSTICK_ENABLE));    //¹Ø±Õ¼ÆÊýÆ÷
	SysTick->CTRL = 0x00;
	SysTick->VAL   =  (0x00);                     /* Load the SysTick Counter Value */         //Çå¿Õ¼ÆÊýÆ÷
}
 
void delay_ms(u16 nms)
{	 		  	  
	u32 temp;	
	SysTick->LOAD=(u32)nms*fac_ms;//Ê±¼ä¼ÓÔØ(SysTick->LOADÎª24bit)
	SysTick->VAL   =  (0x00);                     /* Load the SysTick Counter Value */         //Çå¿Õ¼ÆÊýÆ÷
	SysTick->CTRL =  (1<<0);   /* Enable SysTick and SysTick Timer */      //¿ªÊ¼µ¹Êý  
	do
		{
		temp=SysTick->CTRL;
		}
	while((temp&0x01)&&(!(temp&(1<<SYSTICK_COUNTFLAG))));//µÈ´ýÊ±¼äµ½´ï   
	SysTick->CTRL	&=  (~(1<<0));    //¹Ø±Õ¼ÆÊýÆ÷
	SysTick->VAL   =  (0x00);                     /* Load the SysTick Counter Value */         //Çå¿Õ¼ÆÊýÆ÷  
} 








































