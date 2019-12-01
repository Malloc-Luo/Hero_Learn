#include "pid.h"

/**
 * @discription :
 *		mode = INT �����޷�
 *		mode = OUT ����޷�
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
	
	pid->IntegralMax = 6000.0;
	pid->OutMax = 7000.0;
	
}


/**
 * @funcation PID_Ctrl PID������
 * 
 * @brief �����ֱ�Ϊ +�趨ֵ��(int16_t)Setvalue
 *									 +ʵ��ֵ��(int16_t)Actualvalue
 *                   +PID���ƽṹ�壺(PID_Config *)pid
 *	ʹ��ʱ������Կ��ƶ�������һ��PID�ṹ�� PID_Config
 *  ��������������ṹ���Ա out Ϊ���ֵ
 */
void PID_Ctrl(float Setvalue, float Actualvalue, PID_Def pid)
{
//	int16_t PID_out;
	
	pid->PID_SetValue = Setvalue;
	pid->PID_ActuallValue = Actualvalue;
	
	pid->PID_Error = pid->PID_SetValue - pid->PID_ActuallValue;
	
	pid->PID_Integral +=pid->PID_Error;
	
	pid->PID_Integral = LIMIT(pid->PID_Integral, -pid->IntegralMax, pid->IntegralMax);
	
	pid->out = (int16_t)(pid->Kp *pid->PID_Error + pid->Ki * pid->PID_Integral + pid->Kd * (pid->PID_LastError - pid->PID_Error));
	
	pid->PID_LastError = pid->PID_Error;
	
	pid->out = LIMIT(pid->out, -pid->OutMax, pid->OutMax);
	
//	pid->out = *Outvalue;
	
}

