#include "referee.h"

#include "USART3.h"


UsartRevData URData;



int8_t Referee_judge(void)
{
	
	if( para_rev[0] != 0xA5 )//
	{
		return 0;
	}
	
	switch( para_rev[4] )
	{
		 case 0x01:
			URData.CPData.GameInfo.remainTime = (URData.buffer[6])|(URData.buffer[7]<<8)|(URData.buffer[8]<<16)|(URData.buffer[9]<<24); 
			URData.CPData.GameInfo.remainLifeValue = (URData.buffer[10])|(URData.buffer[11]<<8);
			BYTE0(URData.CPData.GameInfo.realChassisOutV)	= 	URData.buffer[12];
			BYTE1(URData.CPData.GameInfo.realChassisOutV)	=	URData.buffer[13];
			BYTE2(URData.CPData.GameInfo.realChassisOutV)	=	URData.buffer[14];
			BYTE3(URData.CPData.GameInfo.realChassisOutV)	=	URData.buffer[15];
			BYTE0(URData.CPData.GameInfo.realChassisOutA)	= 	URData.buffer[16];
			BYTE1(URData.CPData.GameInfo.realChassisOutA)	=	URData.buffer[17];
			BYTE2(URData.CPData.GameInfo.realChassisOutA)	=	URData.buffer[18];
			BYTE3(URData.CPData.GameInfo.realChassisOutA)	=	URData.buffer[19];
			URData.CPData.GameInfo.runeStatus[0] = URData.buffer[20];
			URData.CPData.GameInfo.runeStatus[1] = URData.buffer[21];
			URData.CPData.GameInfo.runeStatus[2] = URData.buffer[22];
			URData.CPData.GameInfo.runeStatus[3] = URData.buffer[23];
			URData.CPData.GameInfo.bigRune0Status = URData.buffer[24];
			URData.CPData.GameInfo.bigRune1status = URData.buffer[25];
			URData.CPData.GameInfo.conveyorBelts0 = URData.buffer[26]&0x03;
			URData.CPData.GameInfo.conveyorBelts0 = URData.buffer[26]&0x0C;
			URData.CPData.GameInfo.parkingApron0 = URData.buffer[26]&0x10;
			URData.CPData.GameInfo.parkingApron1 = URData.buffer[26]&0x20;
			URData.CPData.GameInfo.parkingApron2 = URData.buffer[26]&0x40;
			URData.CPData.GameInfo.parkingApron3 = URData.buffer[26]&0x80;
			break;
		
		case 0x02:
			URData.BData.weakId = URData.buffer[6]&0x0f;
			URData.BData.weakId = (URData.buffer[6]&0xf0) >>4;
			URData.BData.value	= (URData.buffer[7])|(URData.buffer[8]<<8);
			break;
		
		case 0x03:
			BYTE0(URData.SData.realBulletShootSpeed)= URData.buffer[6];
			BYTE1(URData.SData.realBulletShootSpeed)=URData.buffer[7];
			BYTE2(URData.SData.realBulletShootSpeed)=URData.buffer[8];
			BYTE3(URData.SData.realBulletShootSpeed)=URData.buffer[9];
			BYTE0(URData.SData.realBulletShootFreq)=URData.buffer[10];
			BYTE1(URData.SData.realBulletShootFreq)=URData.buffer[11];
			BYTE2(URData.SData.realBulletShootFreq)=URData.buffer[12];
			BYTE3(URData.SData.realBulletShootFreq)=URData.buffer[13];
			BYTE0(URData.SData.realGolfShootSpeed)=URData.buffer[14];
			BYTE1(URData.SData.realGolfShootSpeed)=URData.buffer[15];
			BYTE2(URData.SData.realGolfShootSpeed)=URData.buffer[16];
			BYTE3(URData.SData.realGolfShootSpeed)=URData.buffer[17];
			BYTE0(URData.SData.realGolfShootFreq)=URData.buffer[18];
			BYTE1(URData.SData.realGolfShootFreq)=URData.buffer[19];
			BYTE2(URData.SData.realGolfShootFreq)=URData.buffer[20];
			BYTE3(URData.SData.realGolfShootFreq)=URData.buffer[21];
			break;
		
		
	}
	
	
	
	return 1;
}