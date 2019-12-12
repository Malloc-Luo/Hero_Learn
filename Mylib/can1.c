#include "can1.h"

/*----CAN1_TX-----PA12----*/
/*----CAN1_RX-----PA11----*/
can1_feedback can1Feedback;
can1_senddata can1Senddata;

void CAN1_Init(void)
{
	CAN_InitTypeDef        can;
	CAN_FilterInitTypeDef  can_filter;
	GPIO_InitTypeDef       gpio;
	NVIC_InitTypeDef       nvic;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_CAN1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_CAN1);

	gpio.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_11;
	gpio.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init(GPIOA, &gpio);
	
	nvic.NVIC_IRQChannel = CAN1_RX0_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 1;
	nvic.NVIC_IRQChannelSubPriority = 1;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);
	
	nvic.NVIC_IRQChannel = CAN1_TX_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 0;
	nvic.NVIC_IRQChannelSubPriority = 1;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);    
	
	CAN_DeInit(CAN1);
	CAN_StructInit(&can);
	
	can.CAN_TTCM = DISABLE;
	can.CAN_ABOM = ENABLE;
	can.CAN_AWUM = DISABLE;
	can.CAN_NART = DISABLE;
	can.CAN_RFLM = DISABLE;
	can.CAN_TXFP = ENABLE;
	can.CAN_Mode = CAN_Mode_Normal;
	can.CAN_SJW  = CAN_SJW_1tq;
	can.CAN_BS1 = CAN_BS1_9tq;
	can.CAN_BS2 = CAN_BS2_4tq;
	can.CAN_Prescaler = 3;   //CAN BaudRate 42/(1+9+4)/3=1Mbps
	CAN_Init(CAN1, &can);

	can_filter.CAN_FilterNumber=0;
	can_filter.CAN_FilterMode=CAN_FilterMode_IdMask;
	can_filter.CAN_FilterScale=CAN_FilterScale_32bit;
	can_filter.CAN_FilterIdHigh=0x0000;
	can_filter.CAN_FilterIdLow=0x0000;
	can_filter.CAN_FilterMaskIdHigh=0x0000;
	can_filter.CAN_FilterMaskIdLow=0x0000;
	can_filter.CAN_FilterFIFOAssignment=0;
	can_filter.CAN_FilterActivation=ENABLE;
	CAN_FilterInit(&can_filter);
		
	CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);
	CAN_ITConfig(CAN1,CAN_IT_TME,ENABLE); 
}


int trigger_positionlast=4096,trigger_round=0;

void CAN1_RX0_IRQHandler(void)
{
	CanRxMsg Can1RxMsg;
	
	if (CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET)
	{
		CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
		CAN_Receive(CAN1, CAN_FIFO0, &Can1RxMsg);
	}
	
	switch(Can1RxMsg.StdId)
	{
		/*摩擦轮速度返回值*/
		case 0x201:
			can1Feedback.frition3508[0] = (int16_t)((Can1RxMsg.Data[2]<<8) | (Can1RxMsg.Data[3]));
			break;
		case 0x202:
			can1Feedback.frition3508[1] = (int16_t)((Can1RxMsg.Data[2]<<8) | (Can1RxMsg.Data[3]));
			break;
		
		//拨弹摩擦轮
		case 0x203:
			can1Feedback.TriggerPosition_now = (int16_t)((Can1RxMsg.Data[0]<<8) | (Can1RxMsg.Data[1]));
			can1Feedback.TriggerSpeed = (int16_t)((Can1RxMsg.Data[2]<<8) | (Can1RxMsg.Data[3]));
			
		/*过圈处理*/
			if(can1Feedback.TriggerPosition_now-trigger_positionlast>4096)
			{
				trigger_round--;
			}
			
			if(can1Feedback.TriggerPosition_now-trigger_positionlast<-4096)
			{
				trigger_round++;
			}
			
			trigger_positionlast = can1Feedback.TriggerPosition_now;
			can1Feedback.TriggerPosition = can1Feedback.TriggerPosition_now+trigger_round*8192;
			break;
			
		/*Pitch轴速度及角度*/	
		case 0x205:
			can1Feedback.positionPitch = (int16_t)((Can1RxMsg.Data[0]<<8) | (Can1RxMsg.Data[1]));
			can1Feedback.positionSpeed = (int16_t)((Can1RxMsg.Data[2]<<8) | (Can1RxMsg.Data[3]));
			break;
		
		case 0x604:
		default:
			break;
	}
}

void CAN1_TX_IRQHandler(void)
{
    if (CAN_GetITStatus(CAN1,CAN_IT_TME)!= RESET)
    {
        CAN_ClearITPendingBit(CAN1,CAN_IT_TME);
    }
}


void Can1_Send_Data_to_Frict(can1_senddata *can1data)
{
	CanTxMsg can1txmsg;
	
	can1txmsg.DLC = 8;
	can1txmsg.IDE = 0;
	can1txmsg.RTR = 0;
	can1txmsg.StdId = 0x200;	
	can1txmsg.Data[0] = (u8)((int16_t)can1data->motor3508right_out>>8);
	can1txmsg.Data[1] = (u8)((int16_t)can1data->motor3508right_out);
	can1txmsg.Data[2] = (u8)((int16_t)can1data->motor3508left_out>>8);
	can1txmsg.Data[3] = (u8)((int16_t)can1data->motor3508left_out);
	
	can1txmsg.Data[4] = (u8)((int16_t)can1data->motorup_out>>8);
	can1txmsg.Data[5] = (u8)((int16_t)can1data->motorup_out);
	
//	if(left_right_frict_power_flag == BOTTOM)
//	{
//		can1txmsg.Data[0] = 0;
//		can1txmsg.Data[1] = 0;
//		can1txmsg.Data[2] = 0;
//		can1txmsg.Data[3] = 0;
//	}
	
	if(up_frict_power_flag == 0 && shootflag == WAIT)
	{
		can1txmsg.Data[4] = 0;
		can1txmsg.Data[5] = 0;
	}
	
	can1txmsg.Data[6] = 0;
	can1txmsg.Data[7] = 0;
	
	CAN_Transmit(CAN1, &can1txmsg);

}


