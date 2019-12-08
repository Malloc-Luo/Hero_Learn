#include "pitch_control.h"

PID_Config Pitch_Inner;
PID_Config Pitch_Outer;

/*
 * pitch轴开机初始化
 */
void Pitch_Init(void)
{
	float innersetvalue = 0.0, outersetvalue = 0.0;
	float inneractualvalue = 0.0, outeractualvalue = 0.0;
	static u8 pidinitflag = 1;
	static u8 counter = 0;
	
	outersetvalue = PITCHCELIB;
	innersetvalue = Pitch_Outer.out;
	inneractualvalue = can1Feedback.positionSpeed;
	
	if(pidinitflag)
	{
		PID_Init(1.2, 0.1, 0, &Pitch_Inner);
		PID_Init(1.2, 0.1, 0, &Pitch_Outer);
		pidinitflag = 0;
	}
	
	PID_Ctrl(innersetvalue, inneractualvalue, &Pitch_Inner);
	can1Senddata_pitch.pitchout_left = (int16_t)Pitch_Inner.out;
	can1Senddata_pitch.pitchout_right = -can1Senddata_pitch.pitchout_left;
	
	if(counter>=1)
	{
		outeractualvalue = can1Feedback.positionPitch;
		
		PID_Ctrl(outersetvalue, outeractualvalue, &Pitch_Outer);
	}
	
	counter ++;
	counter = ((counter>=2)? 0: counter);
	
	Can1_Send_Data_to_Pitch(&can1Senddata_pitch);
}

/**
 * pitch轴校准检查
 */
u8 Pitch_Celib_Flag(void)
{
	int16_t pitch_angle = 0;
	static u8 begin_flag = 0;
	static u8 counter = 0;
	u8 flag = 0;
	int16_t error;
	pitch_angle = can1Feedback.positionPitch;
	error = pitch_angle - PITCHCELIB;
	
	if(ABS_int(error)<50)
	{
		begin_flag = 1;
		counter = 0;
	}
	else
	{
		begin_flag = 0;
		counter = 0;
	}
	
	if(begin_flag)
	{
		counter++;
		if(counter>=4)
			flag = SUCCESS;
	}
	else
	{
		flag = UNSUCCESS;
	}
	
	return flag;
}


/**
 * pitch轴控制
 */
void Pitch_Control(void)
{
	float innersetvalue = 0.0, outersetvalue = 0.0;
	float inneractualvalue = 0.0, outeractualvalue = 0.0;
	int16_t pitch_angle;
	static u8 pidinitflag = 1;
	static u8 counter = 0;
//	static u8 pitch_init_flag = 1;
//	static u8 pitch_flag = 1;
//	
//	pitch_init_flag = Pitch_Celib_Flag();
//	
//	if(pitch_init_flag == UNSUCCESS)
//	{
//		Pitch_Init();
//		
//	}
	
	if(pidinitflag)
	{
		PID_Init(1.2, 0.1, 0, &Pitch_Inner);
		PID_Init(1.2, 0.1, 0, &Pitch_Outer);
		pidinitflag = 0;
	}
	pitch_angle = can1Feedback.positionPitch;
	
	innersetvalue = Pitch_Outer.out;
	inneractualvalue = mpu6050Top.gyro.dps.data[x];
	
	PID_Ctrl(innersetvalue, inneractualvalue, &Pitch_Inner);
	can1Senddata_pitch.pitchout_left = (int16_t)Pitch_Inner.out;
	can1Senddata_pitch.pitchout_right = -can1Senddata_pitch.pitchout_left;
	
	if(counter>=1)
	{
		if(pitch_angle>PITCH_MAX)
			outersetvalue = PITCH_MIDDLE;
		else
			outersetvalue -= (NDJ6.ch[1]*0.7+1.5*NDJ6.ch[7]*4)*0.02;
		
		outeractualvalue = gimbalTopAngle.roll;
		PID_Ctrl(outersetvalue, outeractualvalue, &Pitch_Outer);
	}
	
	counter ++;
	counter = ((counter>1)? 0: counter);
	
//	Can1_Send_Data_to_Pitch(&can1Senddata_pitch);
}


