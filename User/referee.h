#ifndef __REFEREE_H__
#define __REFEREE_H__


#include "stm32f4xx.h"
#include <stdio.h>


typedef enum{
		BUFF_TYPE_NONE, 
		BUFF_TYPE_ARMOR = 0x01, 
		BUFF_TYPE_SUPPLY = 0x04, 
		BUFF_TYPE_BULLFTS= 0x08, 
}eBuffType;

typedef __packed struct
{
	uint8_t flag; 
	uint32_t x;
	uint32_t y;
	uint32_t z;
	uint32_t compass;
}tGpsData;

typedef __packed struct
{
	uint32_t remainTime;
	uint16_t remainLifeValue;
	float realChassisOutV;
	float realChassisOutA;
	uint8_t runeStatus[4];
	uint8_t bigRune0Status;
	uint8_t bigRune1status;
	uint8_t conveyorBelts0:2;
	uint8_t conveyorBelts1:2;
	uint8_t parkingApron0:1;
	uint8_t parkingApron1:1;
	uint8_t parkingApron2:1;
	uint8_t parkingApron3:1;
	tGpsData gpsData;
}tGameInfo;

typedef struct CompetitionProcessData{
	eBuffType BuffType;
	tGpsData  GpsData;
	tGameInfo GameInfo;
}CompetitionProcessData;

typedef __packed struct
{
	uint8_t weakId:4;
	uint8_t way:4;
	uint16_t value;
}tRealBloodChangedData;

typedef __packed struct
{
	float realBulletShootSpeed;
	float realBulletShootFreq;
	float realGolfShootSpeed;
	float realGolfShootFreq;
}tRealShootData;




typedef struct UsartRevData{
	uint8_t SOF;
	uint16_t DataLength;
	uint8_t CRC8;
	uint16_t CmdID;
	CompetitionProcessData 	CPData;
  tRealBloodChangedData 	BData;
	tRealShootData 					SData;
	uint8_t buffer[50];
}UsartRevData;

extern UsartRevData URData;


int8_t Referee_judge(void);//处理信息，裁判系统

#endif


