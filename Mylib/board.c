#include "board.h"

#define TICK_PER_SECOND 1000 
#define TICK_US	(1000000/TICK_PER_SECOND)

volatile uint32_t sysTickUptime=0;

/*ÆµÂÊ1000Hz*/
void TDT_SysTick_Configuration(void)
{
	RCC_ClocksTypeDef  rcc_clocks;
	uint32_t cnts;

	RCC_GetClocksFreq(&rcc_clocks);

	cnts = (uint32_t)rcc_clocks.HCLK_Frequency / TICK_PER_SECOND / 8;

	SysTick_Config(cnts);
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);
	
}



uint32_t GetSysTime_us(void) 
{
	register uint32_t ms;
	u32 value;
	ms = sysTickUptime;
	value = ms * TICK_US + (SysTick->LOAD - SysTick->VAL) * TICK_US / SysTick->LOAD;
	return value;
}

void DelayUs(uint16_t us)
{
	uint32_t now = GetSysTime_us();
    
	while (GetSysTime_us() - now < us);
}

void DelayMs(uint32_t ms)
{
    while (ms--)
      DelayUs(1000);
}

void Init_All(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);
	
#ifdef OLD_TIMER_MODE
	Timer_Init(&MyTimer);
#endif
	
	TDT_Dbus_Configuration();
	TDT_SysTick_Configuration();
	TDT_Cycle_Time_Init();
	Switch_Init();	
	Can2_Init();
	CAN1_Init();
//	I2C1_Soft_Init();  
//	DelayMs(100);                       
//	Mpu6050Top_Init(1000, 40);          	
//	DelayMs(100);                       
//	Mpu6050Top_CalOffset_Gyro();   	
	
}