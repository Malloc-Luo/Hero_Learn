#ifndef __TIMER_H__
#define __TIMER_H__ 
#include "stdint.h"
#include "can2.h"
#include "board.h"
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"

#define NOW 0
#define OLD 1
#define NEW 2

#define GET_TIME_NUM 10

float Get_Cycle_T(u8);
void TDT_Cycle_Time_Init(void);

#ifdef OLD_TIMER_MODE
#define USE_OLD_MODE

typedef struct
{
	uint8_t for_1000Hz;
	uint8_t for_500Hz;
	uint8_t for_333Hz;
	uint8_t for_250Hz;
	uint8_t for_200Hz;
	uint8_t for_100Hz;
	
}TIMERS;

extern TIMERS MyTimer;

extern u8 PowerFlag;

void Timer_Init(TIMERS *);
void Timer(void);
void Timer_1000Hz(void);
void Timer_500Hz(void);
void Timer_333Hz(void);
void Timer_250Hz(void);
void Timer_200Hz(void);
void Timer_100Hz(void);
#endif

#ifdef NEW_TIMER_MODE
#define USE_NEW_MODE

extern uint16_t Timer200Hz;

void Timer_0(void);
void Timer_1(void);
void Timer_2(void);
void Timer_3(void);
void Timer_4(void);

#endif


#endif 












