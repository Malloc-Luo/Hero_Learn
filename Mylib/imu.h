#ifndef _IMU_H
#define	_IMU_H
#include "board.h"
#include "mpu6050.h"

#define IMU_INTEGRAL_LIM (2.0f *ANGLE_TO_RAD )

#define RtA 	  57.324841f

#define x 0
#define y 1
#define z 2

typedef struct _eulerAngle
{
	float pitch;
	float roll;
	float yaw;
  float lastyaw;
  float yawsum;
  long YAW;
  long Angle_error;
  long R__Angle;
	long last_Angle;
	int yaw_cnt;		
}eulerAngle;


extern eulerAngle gimbalTopAngle;
extern eulerAngle gimbalBotAngle;
extern float Special_pitch;//互补滤波后的陀螺仪数值
void TDT_IMUTopupdate(float half_T, vec3f* gyro, vec3f* acc);
void TDT_IMUBotupdate(float half_T, vec3f* gyro, vec3f* acc);

#endif
