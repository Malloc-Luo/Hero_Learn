#include "can2.h"

void Can2_Init(void)
{
  CAN_InitTypeDef        can;
  CAN_FilterInitTypeDef  can_filter;
  GPIO_InitTypeDef       gpio;
  NVIC_InitTypeDef       nvic;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);

  GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_CAN2);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_CAN2);

  gpio.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_12 ;
  gpio.GPIO_Mode = GPIO_Mode_AF;
  GPIO_Init(GPIOB, &gpio);

  nvic.NVIC_IRQChannel = CAN2_RX0_IRQn;
  nvic.NVIC_IRQChannelPreemptionPriority = 3;
  nvic.NVIC_IRQChannelSubPriority = 1;
  nvic.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvic);

  nvic.NVIC_IRQChannel = CAN2_TX_IRQn;
  nvic.NVIC_IRQChannelPreemptionPriority = 2;
  nvic.NVIC_IRQChannelSubPriority = 1;
  nvic.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvic);

  CAN_DeInit(CAN2);
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
  can.CAN_Prescaler = 3;               //CAN BaudRate 42/(1+9+4)/3=1Mbps
  CAN_Init(CAN2, &can);

  can_filter.CAN_FilterNumber=14;
  can_filter.CAN_FilterMode=CAN_FilterMode_IdMask;
  can_filter.CAN_FilterScale=CAN_FilterScale_32bit;
  can_filter.CAN_FilterIdHigh=0x0000;
  can_filter.CAN_FilterIdLow=0x0000;
  can_filter.CAN_FilterMaskIdHigh=0x0000;
  can_filter.CAN_FilterMaskIdLow=0x0000;
  can_filter.CAN_FilterFIFOAssignment=0;//the message which pass the filter save in fifo0
  can_filter.CAN_FilterActivation=ENABLE;
  CAN_FilterInit(&can_filter);

  CAN_ITConfig(CAN2,CAN_IT_FMP0,ENABLE);
  CAN_ITConfig(CAN2,CAN_IT_TME,ENABLE);

}

unsigned char can_tx_success_flag = 0;

void CAN2_TX_IRQHandler(void)
{
  if(CAN_GetITStatus(CAN2,CAN_IT_TME)!= RESET)
    {
      CAN_ClearITPendingBit(CAN2,CAN_IT_TME);
      can_tx_success_flag=1;
    }
}



/**
 * @discription CAN2接收中断
 *
 */

CanRxMsg CAN2_Rxmessage;     //can原始数据
can2_feedback can2feedback;  //can接收处理数据
can2_senddata can2senddata_chassis;
can2_senddata can2senddata_yaw;

int16_t real_yaw;
int yaw_offset=4091;

void CAN2_RX0_IRQHandler(void)
{
  if (CAN_GetITStatus(CAN2,CAN_IT_FMP0)!= RESET)
	{
		CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);
		CAN_Receive(CAN2, CAN_FIFO0, &CAN2_Rxmessage);

		switch(CAN2_Rxmessage.StdId)
		{
			/*底盘四个电机*/
			case 0x201:
				can2feedback.motor3508[0] = (int16_t)((CAN2_Rxmessage.Data[2]<<8) | CAN2_Rxmessage.Data[3]);
				break;
			case 0x202:
				can2feedback.motor3508[1] = (int16_t)((CAN2_Rxmessage.Data[2]<<8) | CAN2_Rxmessage.Data[3]);
				break;
			case 0x203:
				can2feedback.motor3508[2] = (int16_t)((CAN2_Rxmessage.Data[2]<<8) | CAN2_Rxmessage.Data[3]);
				break;
			case 0x204:
				can2feedback.motor3508[3] = (int16_t)((CAN2_Rxmessage.Data[2]<<8) | CAN2_Rxmessage.Data[3]);
				break;

			/*YAW轴机械角度及电机转速*/
			case 0x207:
				real_yaw = (int16_t)((CAN2_Rxmessage.Data[0]<<8) | (CAN2_Rxmessage.Data[1]));

				if(real_yaw - yaw_offset >0)
					can2feedback.positionYaw = real_yaw - yaw_offset;
				else
					can2feedback.positionYaw = 8192 + (real_yaw - yaw_offset);

				can2feedback.yawspeed = (int16_t)((CAN2_Rxmessage.Data[2]<<8) | CAN2_Rxmessage.Data[3]);
				break;
		}
	}
}

/*
 * 将控制数据发送至底盘电机
 */
void Can2_Send_Data_to_Chassis(can2_senddata *can2data)
{
  CanTxMsg Can2TxMsg;

  Can2TxMsg.RTR = 0;
  Can2TxMsg.IDE = 0;
  Can2TxMsg.DLC = 8;

  Can2TxMsg.StdId = 0x200;

  if(PowerFlag == LOST_POWER)
	{
		Can2TxMsg.Data[0] = 0;
		Can2TxMsg.Data[1] = 0;
		Can2TxMsg.Data[2] = 0;
		Can2TxMsg.Data[3] = 0;
		Can2TxMsg.Data[4] = 0;
		Can2TxMsg.Data[5] = 0;
		Can2TxMsg.Data[6] = 0;
		Can2TxMsg.Data[7] = 0;
	}
  else
	{
		Can2TxMsg.Data[0] = (u8)((int16_t)can2data->motor3508out[0]>>8);
		Can2TxMsg.Data[1] = (u8)((int16_t)can2data->motor3508out[0]);
		Can2TxMsg.Data[2] = (u8)((int16_t)can2data->motor3508out[1]>>8);
		Can2TxMsg.Data[3] = (u8)((int16_t)can2data->motor3508out[1]);
		Can2TxMsg.Data[4] = (u8)((int16_t)can2data->motor3508out[2]>>8);
		Can2TxMsg.Data[5] = (u8)((int16_t)can2data->motor3508out[2]);
		Can2TxMsg.Data[6] = (u8)((int16_t)can2data->motor3508out[3]>>8);
		Can2TxMsg.Data[7] = (u8)((int16_t)can2data->motor3508out[3]);
	}


  CAN_Transmit(CAN2, &Can2TxMsg);
}

void Can2_Send_Data_to_Yaw(can2_senddata *can2data)
{
  CanTxMsg Can2TxMsg;

  Can2TxMsg.RTR = 0;
  Can2TxMsg.IDE = 0;
  Can2TxMsg.DLC = 8;

  Can2TxMsg.StdId = 0x1ff;

  Can2TxMsg.Data[0] = 0;
  Can2TxMsg.Data[1] = 0;
  Can2TxMsg.Data[2] = 0;
  Can2TxMsg.Data[3] = 0;
  Can2TxMsg.Data[4] = (u8)(int16_t)(can2data->yaw_out>>8);
  Can2TxMsg.Data[5] = (u8)(int16_t)(can2data->yaw_out);
  Can2TxMsg.Data[6] = 0;
  Can2TxMsg.Data[7] = 0;

  CAN_Transmit(CAN2, &Can2TxMsg);

}


