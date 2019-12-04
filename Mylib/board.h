#ifndef __BOARD_H__
#define __BOARD_H__

//掉电
#define LOST_POWER 1
//
#define ENABLE_POWER 2

/*
 * 使用新的系统定时器模式或者旧的定时器模式
 * 		0  旧的定时器模式
 * 		1	 新的定时器模式
 */
#define TIMER_MODE 0

#if TIMER_MODE
#define NEW_TIMER_MODE
#else
#define OLD_TIMER_MODE
#endif

#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "stm32f4xx_gpio.h"
#include <math.h>
#include <arm_math.h>
#include <stdio.h>
#include <stdlib.h.>

#include "pid.h"
#include "can2.h"
#include "can1.h"
#include "control.h"
#include "TDT_hero.h"
#include "dbus.h"
#include "mymath.h"
#include "timer.h"
#include "fdbus.h"
#include "myiic.h"
#include "mpu6050.h"
#include "imu.h"
#include "yaw_control.h"
#include "pitch_control.h"

//吸引火力
test_Empty(void);



void TDT_SysTick_Configuration(void);
void Init_All(void);
uint32_t GetSysTime_us(void);
void DelayUs(uint16_t us);
void DelayMs(uint32_t ms);

#endif



