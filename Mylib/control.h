#ifndef __CONTROL_H__
#define __CONTROL_H__
//#include "board.h"
#include "pid.h"
#include "stm32f4xx.h"
#include "stdio.h" 
#include "can2.h"
#include "dbus.h"
#include "stdlib.h"
#include "mymath.h"

#define MOTOR_NUMBER 				 4
#define MAX_PITCH_ASPPED     300	///��̨pitch�������ת���ٶ�
#define MAX_YAW_ASPPED       3000	///��̨yaw�������ת����
#define MID_YAW_ANGLE        2720
#define MID_YAW_BEHIND_ANGLE 6850
#define MID_PITCH_ANGLE      4060

typedef struct PID_Controller PID_Config;
typedef struct PID_Controller * PID_Def;

extern PID_Config Chassis_Motor[4];

void Chassis_Init(PID_Config * );
void Chassis_Ctrl(void);

#endif



