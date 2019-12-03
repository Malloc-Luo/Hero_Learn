#include "stm32f4xx_it.h"
#include "board.h"
/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}


extern u8 Init_OK;

void SysTick_Handler(void)
{
#ifdef OLD_TIMER_MODE
	MyTimer.for_1000Hz++;
	MyTimer.for_100Hz++;
	MyTimer.for_200Hz++;
	MyTimer.for_250Hz++;
	MyTimer.for_333Hz++;
	MyTimer.for_500Hz++;
	Timer();
#endif
	
#ifdef NEW_TIMER_MODE
	
	Mpu6050Top_Read();                       	/*��ȡmpu6050Top����*/
	Mpu6050Top_Data_Prepare();              	/*mpu6050Top׼������*/
	
	if(Timer200Hz>=5)
		Timer200Hz = 0;
	
	switch(Timer200Hz)
	{
		case 0:
			Timer_0();
			break;
		case 1:
			Timer_1();
			break;
		case 2:
			Timer_2();
			break;
		case 3:
			Timer_3();
			break;
		case 4:
			Timer_4();
			break;
		default :
			break;
	}
	Timer200Hz ++;
#endif
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
  
/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/
