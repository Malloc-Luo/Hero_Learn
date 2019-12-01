#ifndef __PID_H__
#define __PID_H__
//#include "board.h"
#include "stm32f4xx.h"
#include "stdlib.h"
#include "control.h"

#define OUT 0
#define INT 1

struct PID_Controller
{
	float PID_SetValue;
	float PID_ActuallValue;
	float PID_Error;
	float PID_LastError;
	float PID_Integral;
	float Kp;
	float Ki;
	float Kd;
	float PID_Ctrl;
	float IntegralMax;
	float OutMax;
	int16_t out;
	
};

typedef struct PID_Controller PID_Config;
typedef struct PID_Controller * PID_Def;

typedef float KI;
typedef float KP;
typedef float KD;
typedef int16_t MODE;

void PID_Init(KP, KI, KD, PID_Config * pid);

void PID_Ctrl(float ,float ,PID_Config * pid);

extern float LIMIT(float, float, float);


#endif





