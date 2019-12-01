#ifndef __MYMATH_H__
#define __MYMATH_H__
#include "stdint.h"
#include "stm32f4xx.h"

#define PI 3.141593f

#define TAN_MAP_RES     0.003921569f     /* (smallest non-zero value in table) */
#define RAD_PER_DEG     0.017453293f
#define TAN_MAP_SIZE    256

float f_sin(float rad);
float f_cos(float rad);
float power(float value, int times);
float f_sqrt(float );
float f_atan2(float ,float);


int ABS_int(int value);
float ABS_float(float value);

#endif
