#ifndef __BOARD_H__
#define __BOARD_H__

#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "stm32f4xx_gpio.h"
#include <math.h>

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

void TDT_SysTick_Configuration(void);
void Init_All(void);
uint32_t GetSysTime_us(void) ;
void DelayUs(uint16_t us);
void DelayMs(uint32_t ms);

#endif

