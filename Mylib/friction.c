#include "friction.h"

u8 left_right_frict_power_flag = 0;
u8 up_frict_power_flag = 0;
int16_t left_right_setvalue = 4700;
int16_t top_setvalue = 7500;

static PID_Config Frict_right;
static PID_Config Frict_left;
static PID_Config Frict_up;

void Friction_control(void)
{
	static u8 counter = 0;
	static u8 pidinitflag = 1;
	float right_setvalue = -left_right_setvalue, left_setvalue = left_right_setvalue, up_setvalue = top_setvalue;
	float right_actualvalue = 0.0, left_actualvalue = 0.0, up_actualvalue = 0.0;
	
	left_right_frict_power_flag = NDJ6.ch[4];
	up_frict_power_flag = NDJ6.ch[5];
	
	if(left_right_frict_power_flag == BOTTOM || left_right_frict_power_flag == NDJLOST)
		left_right_setvalue = 0;
	else
		left_right_setvalue = 4700;
	
//	if(up_frict_power_flag == BOTTOM)
//		top_setvalue = 0;
//	else
//		top_setvalue = 7200;
	
	if(pidinitflag)
	{
		PID_Init(2.2, 0.1, 0, &Frict_right);
		PID_Outmax_Set(2000, &Frict_right);
		PID_Integralmax_Set(5000, &Frict_right);
		
		PID_Init(2.2, 0.1, 0, &Frict_left);
		PID_Outmax_Set(2000, &Frict_left);
		PID_Integralmax_Set(5000, &Frict_left);
		
		PID_Init(1.5, 0.1, 0, &Frict_up);
		PID_Outmax_Set(1100, &Frict_up);
		pidinitflag = 0;
	}
	
	right_actualvalue = can1Feedback.frition3508[0];
	left_actualvalue = can1Feedback.frition3508[1];
	up_actualvalue = can1Feedback.TriggerSpeed;
	
	PID_Ctrl(right_setvalue, right_actualvalue, &Frict_right);
	PID_Ctrl(left_setvalue, left_actualvalue, &Frict_left);
	PID_Ctrl(up_setvalue, up_actualvalue, &Frict_up);
	
	can1Senddata_frict.motor3508right_out = (int16_t)Frict_right.out;
	can1Senddata_frict.motor3508left_out = (int16_t)Frict_left.out;
	can1Senddata_frict.motorup_out = (int16_t)Frict_up.out;
	
	Can1_Send_Data_to_Frict(&can1Senddata_frict);
}


