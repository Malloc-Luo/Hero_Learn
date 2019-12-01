#include "timer.h"

TIMERS MyTimer;

volatile float Cycle_T[GET_TIME_NUM][3];

float Get_Cycle_T(u8 item)	
{
	Cycle_T[item][OLD] = Cycle_T[item][NOW];	//上一次的时间
	Cycle_T[item][NOW] = GetSysTime_us()/1000000.0f; //本次的时间
	Cycle_T[item][NEW] = ( ( Cycle_T[item][NOW] - Cycle_T[item][OLD] ) );//间隔的时间（周期）
	return Cycle_T[item][NEW];
}

void TDT_Cycle_Time_Init(void)
{
	u8 i;
	for(i=0;i<GET_TIME_NUM;i++)
	{
		Get_Cycle_T(i);
	}
}

void Timer_Init(TIMERS *Timer)
{
	Timer->for_1000Hz = 0;
	Timer->for_100Hz = 0;
	Timer->for_200Hz = 0;
	Timer->for_250Hz = 0;
	Timer->for_333Hz = 0;
	Timer->for_500Hz = 0;
}

void Timer(void)
{
	if(MyTimer.for_100Hz>=10)
	{
		Timer_100Hz();
		MyTimer.for_100Hz = 0;
	}
	
	if(MyTimer.for_200Hz>=5)
	{
		Timer_200Hz();
		MyTimer.for_200Hz = 0;
	}
	
	if(MyTimer.for_250Hz>=4)
	{
		Timer_250Hz();
		MyTimer.for_250Hz = 0;
	}
	
	if(MyTimer.for_333Hz>=3)
	{
		Timer_333Hz();
		MyTimer.for_333Hz = 0;
	}
	
	if(MyTimer.for_500Hz>=2)
	{
		Timer_500Hz();
		MyTimer.for_500Hz = 0;
	}
	
	if(MyTimer.for_1000Hz>=1)
	{
		Timer_1000Hz();
		MyTimer.for_1000Hz = 0;
	}

}

u8 PowerFlag = 0;
void Timer_1000Hz(void)
{
	
	Mpu6050Top_Read();                       	/*读取mpu6050Top数据*/
	Mpu6050Top_Data_Prepare();              	/*mpu6050Top准备数据*/
	
}

void Timer_500Hz(void)
{
	float loop_time_500hz;
	loop_time_500hz = Get_Cycle_T(0); 
	TDT_IMUTopupdate(0.5f *loop_time_500hz, &mpu6050Top.gyro.radps, &mpu6050Top.acc.filter ); /*Top四元数姿态解算*/
	PowerFlag = NDJ6.ch[4];
	
	if(PowerFlag != 1)
		Chassis_Ctrl();
}

void Timer_333Hz(void)
{
	
}

void Timer_250Hz(void)
{
	
}

void Timer_200Hz(void)
{
	
}

void Timer_100Hz(void)
{
	
}



