#include "control.h"

void Chassis_Motor_Get_Speed(int16_t * Input, int16_t * Output)
{
	u8 i = 0;
	for(i = 0;i<MOTOR_NUMBER; i++)
	{
		Output[i] = Input[i];
	}
}


/**
 * 底盘电机控制
 */
static float forward_back_data;
static float left_right_data;
double yaw_rad;
float sin_rad;
float cos_rad;
PID_Config Chassis_Motor[4];

void Chassis_Ctrl(void)
{
//	static PID_Config Chassis_Motor[4];
	static uint16_t counter = 0;
	float Motor_Setspeed[4];
	float Motor_Actualspeed[4];
	u8 i = 0;
	
	yaw_rad = (float)(can2feedback.positionYaw - MID_ANGLE_YAW) * 0.000767084f;
	
	if(yaw_rad <0)
		yaw_rad = yaw_rad + 2*PI;
	
	sin_rad = f_sin(yaw_rad);
	cos_rad = f_cos(yaw_rad);
//	sin_rad = 0;
//	cos_rad = 1;

	if(counter == 0)
	{
		for(i = 0;i <MOTOR_NUMBER;i++)
			PID_Init(4.8, 0.1, 0, &Chassis_Motor[i]);
	}

	/*处理遥控器信号值*/
	if(NDJ6.ch[3]>20 || NDJ6.ch[3]< -20)
		forward_back_data = NDJ6.ch[3] * FB_MAXSPEED;
	else
		forward_back_data = 0;
	
	if(NDJ6.ch[2]>20 || NDJ6.ch[2]<-20)
		left_right_data = -NDJ6.ch[2] * LR_MAXSPEED;
	else 
		left_right_data = 0;
	
	Motor_Setspeed[0] = +((forward_back_data*cos_rad+left_right_data*(-sin_rad))+ (left_right_data*cos_rad+forward_back_data*sin_rad) - Chassis_Follow_value * (4.2f));
	Motor_Setspeed[1] = -((forward_back_data*cos_rad+left_right_data*(-sin_rad))+ (left_right_data*cos_rad+forward_back_data*sin_rad) - Chassis_Follow_value * (4.2f));
	Motor_Setspeed[2] = +((forward_back_data*cos_rad+left_right_data*(-sin_rad))- (left_right_data*cos_rad+forward_back_data*sin_rad) - Chassis_Follow_value * (4.2f));
	Motor_Setspeed[3] = -((forward_back_data*cos_rad+left_right_data*(-sin_rad))- (left_right_data*cos_rad+forward_back_data*sin_rad) - Chassis_Follow_value * (4.2f));
	
	/*传入电机实际转速*/
//	Chassis_Motor_Get_Speed(can2feedback.motor3508, Motor_Actualspeed);
	for(i = 0;i<MOTOR_NUMBER;i++)
		Motor_Actualspeed[i] = (float)can2feedback.motor3508[i];
	
	for(i = 0;i<MOTOR_NUMBER;i++)
	{
		if(Motor_Actualspeed[i]<=10 && Motor_Actualspeed[i]>=-10)
			PID_Init(4.8, 0.1, 0, &Chassis_Motor[i]);
		
		PID_Ctrl(Motor_Setspeed[i], Motor_Actualspeed[i], &Chassis_Motor[i]);
		can2senddata.motor3508out[i] = (int16_t)Chassis_Motor[i].out;
	}
	
	if(counter>=10000)
		counter = 1;
	
	counter ++;
	Can2_Send_Data_to_Chassis(&can2senddata);
	
}