#include "dbus.h"

rc NDJ6;

unsigned char sbus_rx_buffer[18];
static uint8_t Get_Keypress(uint16_t Key);
int shoot_change,NDJ6keybit_G,NDJ6keybit_B,NDJ6keybit_C;

void TDT_Dbus_Configuration(void)
{		
		USART_InitTypeDef USART_InitStructure;
	  GPIO_InitTypeDef  GPIO_InitStructure;
    NVIC_InitTypeDef  NVIC_InitStructure;
    DMA_InitTypeDef   DMA_InitStructure;
	
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_DMA1,ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
		
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource3 ,GPIO_AF_USART3);
	
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOA,&GPIO_InitStructure);
    
		USART_DeInit(USART2);
		USART_InitStructure.USART_BaudRate = 100000;
		USART_InitStructure.USART_WordLength = USART_WordLength_8b;
		USART_InitStructure.USART_StopBits = USART_StopBits_1;
		USART_InitStructure.USART_Parity = USART_Parity_No;
		USART_InitStructure.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_Init(USART2,&USART_InitStructure);			
		USART_Cmd(USART2,ENABLE);
				
		USART_ITConfig(USART2,USART_IT_IDLE,ENABLE);
		   
    USART_DMACmd(USART2,USART_DMAReq_Rx,ENABLE);
 
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
 
    DMA_DeInit(DMA1_Stream5);
    DMA_InitStructure.DMA_Channel= DMA_Channel_4;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(USART2->DR);
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)sbus_rx_buffer;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize = 18;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    DMA_InitStructure.DMA_MemoryBurst = DMA_Mode_Normal;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA1_Stream5,&DMA_InitStructure);

    DMA_Cmd(DMA1_Stream5,ENABLE);
}

u16 i_temp;
void USART2_IRQHandler(void)
{
	int16_t sbus_decode_buffer[4];
	if(USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)
	{
		DMA_Cmd(DMA1_Stream5,DISABLE);
	
		i_temp = USART2->SR;  
		i_temp = USART2->DR; 
		NDJ6.ch_last[4]=NDJ6.ch[4];
		NDJ6.ch_last[5]=NDJ6.ch[5];
		NDJ6.ch_last[8]=NDJ6.ch[8];	
		NDJ6.ch_last[9]=NDJ6.ch[9];	
		NDJ6.last_key.R=NDJ6.key.bit.R;
		NDJ6.last_key.F=NDJ6.key.bit.F;
		NDJ6.last_key.V=NDJ6.key.bit.V;
		NDJ6.last_key.B=NDJ6.key.bit.B;
		NDJ6.last_key.Z=NDJ6.key.bit.Z;
		NDJ6.last_key.Q=NDJ6.key.bit.Q;
		NDJ6.last_key.E=NDJ6.key.bit.E;
		NDJ6.last_key.X=NDJ6.key.bit.X;
		NDJ6.last_key.G=NDJ6.key.bit.G;
		NDJ6.last_key.C=NDJ6.key.bit.C;
		
		//”““°∏À∫·œÚ   ∑∂Œß+-660
		sbus_decode_buffer[0] = (sbus_rx_buffer[0]| (sbus_rx_buffer[1] << 8)) & 0x07ff; //!< Channel 0
		NDJ6.ch[0] = deathzoom(sbus_decode_buffer[0]-1024, 10);
		if(ABS_int(NDJ6.ch[0])<15)
			NDJ6.ch[0] = 0;
		//”““°∏À◊›œÚ   ∑∂Œß+-660
		 sbus_decode_buffer[1] = ((sbus_rx_buffer[1] >> 3) | (sbus_rx_buffer[2] << 5)) & 0x07ff; //!< Channel 1
		NDJ6.ch[1] = deathzoom(sbus_decode_buffer[1]-1024, 10);
		//◊Û“°∏À∫·œÚ   ∑∂Œß+-660
		sbus_decode_buffer[2]= ((sbus_rx_buffer[2] >> 6) | (sbus_rx_buffer[3] << 2) | (sbus_rx_buffer[4] << 10)) & 0x07ff; //!< Channel 2
		NDJ6.ch[2] = deathzoom(sbus_decode_buffer[2]-1024, 55);
		//◊Û“°∏À◊›œÚ   ∑∂Œß+-660
		sbus_decode_buffer[3] = ((sbus_rx_buffer[4] >> 1) | (sbus_rx_buffer[5] << 7)) & 0x07ff; //!< Channel 3
		NDJ6.ch[3] = deathzoom(sbus_decode_buffer[3]-1024, 10);
		
		if ((ABS_int(NDJ6.ch[0]) > 660) || \
				(ABS_int(NDJ6.ch[1]) > 660) || \
				(ABS_int(NDJ6.ch[2]) > 660) || \
				(ABS_int(NDJ6.ch[3]) > 660))
		{
			memset(&NDJ6, 0, sizeof(rc));
			return ;
		}
		NDJ6.ch[4] = ((sbus_rx_buffer[5] >> 4)& 0x000C) >> 2; //!< Switch left			 
		NDJ6.ch[5] = ((sbus_rx_buffer[5] >> 4)& 0x0003); 			//!< Switch right9 / 9 
    NDJ6.ch[6] = 	((sbus_rx_buffer[6]) | (sbus_rx_buffer[7] << 8));	 // Û±Í“∆∂Øx 
		NDJ6.ch[7] = 	-((sbus_rx_buffer[8]) | (sbus_rx_buffer[9] << 8)); // Û±Í“∆∂Øy 
		NDJ6.ch[8] = sbus_rx_buffer[12]; // Û±Í◊Ûº¸
		NDJ6.ch[9] = sbus_rx_buffer[13]; // Û±Í”“º¸	
		NDJ6.key.key_code = sbus_rx_buffer[14]| ((int16_t)sbus_rx_buffer[15] << 8); //W(1)S(2)A(4)D(8) QE shift(16) ctr(32) 

		DMA1_Stream5->NDTR = 18;
		USART_ClearITPendingBit(USART2, USART_IT_IDLE);
		DMA_Cmd(DMA1_Stream5,ENABLE);
		
		if(NDJ6.last_key.Z==0&&NDJ6.key.bit.Z==1)
		{
			shoot_change++;
			
			if(shoot_change>1)
				shoot_change=0;
		}
		
		if(NDJ6.last_key.G==0&&NDJ6.key.bit.G==1)
		{
			NDJ6keybit_G++;
		
			if(NDJ6keybit_G>1)
				NDJ6keybit_G = 0;
		}
		
		if(NDJ6.last_key.B==0&&NDJ6.key.bit.B==1)
		{
			NDJ6keybit_B++;
			
			if(NDJ6keybit_B>1)
				NDJ6keybit_B = 0;
		}
		
		if(NDJ6.last_key.C==0&&NDJ6.key.bit.C==1)
		{
			NDJ6keybit_C++;
			
			if(NDJ6keybit_C>1)
				NDJ6keybit_C = 0;
		}
	}
}

/**
  * @brief  Be sure if the key has pressed.
  * @param  Key_value
  * @retval True or False
  */
//static uint8_t Get_Keypress(uint16_t Key)
//{
//    if(NDJ6.ch[10] & Key)
//       return 1;
//    else
//			 return 0;
//}
