#include "pid.h"

/**
 * @discription :
 *		mode = INT 积分限幅
 *		mode = OUT 输出限幅
 */
float LIMIT(float a, float min, float max)
{
	if(a<=min) return min;
	if(a>=max) return max;
}


void PID_Init(KP p, KI i, KD d, PID_Def pid)
{
	pid->Kp = p;
	pid->Ki = i;
	pid->Kd = d;
	
	pid->PID_ActuallValue = 0.0;
	pid->PID_Ctrl = 0.0;
	pid->PID_Error = 0.0;
	pid->PID_Integral = 0.0;
	pid->PID_LastError = 0.0;
	pid->PID_SetValue = 0.0;
	
	pid->IntegralMax = 8000.0;
	pid->OutMax = 1500.0;
	
}

/**
 * 设置输出最大限幅<=2500
 * default 1500
 */
void PID_Outmax_Set(uint16_t outmax,PID_Def pid)
{
	pid->OutMax = (float)outmax;
	
	if(pid->OutMax>=2500.0f)
		pid->OutMax = 2500.0f;
	
}

/*
 * 设置积分限幅<=10000
 * default 8000
 */
void PID_Integralmax_Set(uint16_t intmax, PID_Def pid)
{
	pid->IntegralMax = (float)intmax;
	
	if(pid->IntegralMax>=10000.0f)
		pid->IntegralMax = 10000.0f;
}


/**
 * @funcation PID_Ctrl PID控制器
 * 
 * @brief 参数分别为 +设定值：(int16_t)Setvalue
 *									 +实际值：(int16_t)Actualvalue
 *                   +PID控制结构体：(PID_Config *)pid
 *	使用时首先针对控制对象声明一个PID结构体 PID_Config
 *  传入参数，调出结构体成员 out 为输出值
 */
void PID_Ctrl(float Setvalue, float Actualvalue, PID_Def pid)
{
	
	pid->PID_SetValue = Setvalue;
	pid->PID_ActuallValue = Actualvalue;
	
	pid->PID_Error = pid->PID_SetValue - pid->PID_ActuallValue;
	
	pid->PID_Integral +=pid->PID_Error;
	
	pid->PID_Integral = LIMIT(pid->PID_Integral, -pid->IntegralMax, pid->IntegralMax);
	
	pid->out = (pid->Kp *pid->PID_Error + pid->Ki * pid->PID_Integral + pid->Kd * (pid->PID_LastError - pid->PID_Error));
	
	pid->PID_LastError = pid->PID_Error;
	
	pid->out = LIMIT(pid->out, -pid->OutMax, pid->OutMax);
	
}

