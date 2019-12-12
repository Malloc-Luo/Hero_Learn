#include "friction.h"

u8 left_right_frict_power_flag = 0;
u8 up_frict_power_flag = 0;
u8 shootflag = FIRE;

static PID_Config Frict_right;
static PID_Config Frict_left;
static PID_Config Frict_up;

void Friction_control(void)
{
	static u8 counter = 0;
	static u8 pidinitflag = 1;
	float right_setvalue = -4500.0, left_setvalue = 4500.0 ;
	float right_actualvalue = 0.0, left_actualvalue = 0.0;
	

	left_right_frict_power_flag = NDJ6.ch[5];
	
	if(pidinitflag)
	{
		PID_Init(1.5, 0.1, 0, &Frict_right);
		PID_Init(1.5, 0.1, 0, &Frict_left);
		pidinitflag = 0;
	}
	
	right_actualvalue = can1Feedback.frition3508[0];
	left_actualvalue = can1Feedback.frition3508[1];

	
	PID_Ctrl(right_setvalue, right_actualvalue, &Frict_right);
	PID_Ctrl(left_setvalue, left_actualvalue, &Frict_left);
	
	
	can1Senddata.motor3508right_out = (int16_t)Frict_right.out;
	can1Senddata.motor3508left_out = (int16_t)Frict_left.out;
	
	Can1_Send_Data_to_Frict(&can1Senddata);
}

//拨弹摩擦轮位置环控制
static PID_Config top_frict_inner;
static PID_Config top_frict_outer;

void Trigger_control(Top_Frict_Mode mode)
{
	float top_inner_setvalue = 8500.0, top_outer_setvalue = 0.0;
	float top_inner_actualvalue = 0.0, top_outer_actualvalue = 0.0;
	static u8 pidinitflag = 1 ;
	static u8 last_mode = 0;
	static u8 counter = 0;
	
	top_inner_actualvalue = can1Feedback.TriggerSpeed;
	
	last_mode = mode;
	
	//速度环模式
	if(mode == SPEED)
	{
		if(last_mode == POSITION)
			pidinitflag = 1;
		
		if(pidinitflag)
		{
			PID_Init(2.5, 0.1, 0, &top_frict_inner);
			pidinitflag = 0;
		}
		
		PID_Ctrl(top_inner_setvalue, top_inner_actualvalue, &top_frict_inner);
	}
	
	//位置环模式
	if(mode == POSITION)
	{
		if(last_mode == SPEED)
		{
			pidinitflag = 1;
			top_outer_setvalue = can1Feedback.TriggerPosition + TOP_MOVE;
		}
		
		if(pidinitflag)
		{
			PID_Init(2.5, 0.1, 0, &top_frict_inner);
			PID_Init(2.5, 0.1, 0, &top_frict_outer);
			pidinitflag = 0;
		}
		
		top_inner_setvalue = top_fric t_outer.out;
		
		PID_Ctrl(top_inner_setvalue, top_inner_actualvalue, &top_frict_inner);
		can1Senddata.motorup_out = (int16_t)Frict_up.out;
		
		counter ++;
		if(counter >= 2)
		{
			counter = 0;
			top_outer_actualvalue = can1Feedback.TriggerPosition;
			
			PID_Ctrl(top_outer_setvalue, top_outer_actualvalue, &top_frict_outer);
		}
	}
}

void Switch_Init(void)
{
	GPIO_InitTypeDef gpio;
	NVIC_InitTypeDef nvic;
	EXTI_InitTypeDef exti;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//使能GPIOA时钟

	gpio.GPIO_Pin = GPIO_Pin_7;
	gpio.GPIO_Mode = GPIO_Mode_IN;			//普通输入模式  
	gpio.GPIO_Speed = GPIO_Speed_100MHz;//100M          
	gpio.GPIO_PuPd = GPIO_PuPd_DOWN;		//下拉          
	GPIO_Init(GPIOB, &gpio);						//初始化GPIOA0,1

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//使能SYSCFG时钟

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource7);

	exti.EXTI_Line = EXTI_Line7;
	exti.EXTI_Mode = EXTI_Mode_Interrupt;		 //中断事件  
	exti.EXTI_Trigger = EXTI_Trigger_Rising; //上升沿触发
	exti.EXTI_LineCmd = ENABLE;							 //中断线使能
	EXTI_Init(&exti);												 //配置      

	nvic.NVIC_IRQChannel = EXTI9_5_IRQn;						//外部中断        
	nvic.NVIC_IRQChannelPreemptionPriority = 0x00;	//抢占优先级0     
	nvic.NVIC_IRQChannelSubPriority = 0x02;					//子优先级2       
	nvic.NVIC_IRQChannelCmd = ENABLE;								//使能外部中断通道
	NVIC_Init(&nvic);
	
}

int shoot_count=0;
void EXTI9_5_IRQHandler(void)
{
	DelayUs(100);
	if(EXTI_GetITStatus(EXTI_Line7) != RESET )
	{
		if (GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_7))
		{
				shootflag=WAIT;
		}
	}
	EXTI_ClearITPendingBit(EXTI_Line7); //清除LINE5上的中断标志位
}

void Shoot(void)
{
	//if(NDJ6.ch[8] == 1 && NDJ6.ch_last[8] == 0/*|| NDJ6.ch[4] != BOTTOM*/)
	//if( NDJ6.ch[4] != BOTTOM)
	if(NDJ6.ch[4] == MIDDLE && NDJ6.ch_last[4] == BOTTOM)
		up_frict_power_flag = 1;
		
	//if(NDJ6.ch[9] == 1 && NDJ6.ch_last[9] == 0/*|| NDJ6.ch[4] == BOTTOM*/)
	//if(NDJ6.ch[4] == BOTTOM)
	if(NDJ6.ch[4] == BOTTOM && NDJ6.ch_last[4] == MIDDLE)
		up_frict_power_flag = 0;
	 
	if(shootflag == WAIT)
	{
		up_frict_power_flag = 0;
		Trigger_control(POSITION);
	}
	
	if(shootflag == FIRE)
		Trigger_control(SPEED);
	
	Friction_control();
		
}
