#ifndef __BOARD_H__
#define __BOARD_H__

//����
#define LOST_POWER 1
//
#define ENABLE_POWER 2

/*
 * ʹ���µ�ϵͳ��ʱ��ģʽ���߾ɵĶ�ʱ��ģʽ
 * 		0  �ɵĶ�ʱ��ģʽ
 * 		1	 �µĶ�ʱ��ģʽ
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

//��������
test_Empty(void);



void TDT_SysTick_Configuration(void);
void Init_All(void);
uint32_t GetSysTime_us(void);
void DelayUs(uint16_t us);
void DelayMs(uint32_t ms);

#endif



