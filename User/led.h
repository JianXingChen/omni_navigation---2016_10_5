#ifndef __LED_H
#define __LED_H
#include "sys.h"



#define LED0 PCout(1)	// DS0
#define LED1 PCout(2)	// DS1	 

void LED_Init(void); 				    
#endif
