#include "friction.h"

u8 left_right_frict_power_flag = 0;
u8 up_frict_power_flag = 0;

static PID_Config Frict_right;
static PID_Config Frict_left;
static PID_Config Frict_up;

void Friction_control(void)
{
	static u8 counter = 0;
	static u8 pidinitflag = 1;
	float right_setvalue = -4500.0, left_setvalue = 4500.0, up_setvalue = 7200.0;
	float right_actualvalue = 0.0, left_actualvalue = 0.0, up_actualvalue = 0.0;
	
	left_right_frict_power_flag = NDJ6.ch[4];
	up_frict_power_flag = NDJ6.ch[5];
	
	if(pidinitflag)
	{
		PID_Init(1.5, 0.1, 0, &Frict_right);
		PID_Init(1.5, 0.1, 0, &Frict_left);
		PID_Init(1.5, 0.1, 0, &Frict_up);
		pidinitflag = 0;
	}
	
	right_actualvalue = can1Feedback.frition3508[0];
	left_actualvalue = can1Feedback.frition3508[1];
	up_actualvalue = can1Feedback.TriggerSpeed;
	
	PID_Ctrl(right_setvalue, right_actualvalue, &Frict_right);
	PID_Ctrl(left_setvalue, left_actualvalue, &Frict_left);
	PID_Ctrl(up_setvalue, up_actualvalue, &Frict_up);
	
	can1Senddata.motor3508right_out = (int16_t)Frict_right.out;
	can1Senddata.motor3508left_out = (int16_t)Frict_left.out;
	can1Senddata.motorup_out = (int16_t)Frict_up.out;
	
	Can1_Send_Data_to_Frict(&can1Senddata);
}


