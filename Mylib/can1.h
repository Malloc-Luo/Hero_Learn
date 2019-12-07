#ifndef __CAN1_H__
#define __CAN1_H__
#include "board.h"

typedef struct _can1_feedback
{
	int16_t frition3508[2];
	long TriggerPosition_now;
	long TriggerPosition;
	long TriggerSpeed;
	double positionYaw;
	double positionPitch;
	double positionSpeed;
	int yawspeed;
	int Friction_safe_flag;
}can1_feedback;

typedef struct _can1_senddata
{
	int16_t motor3508right_out;
	int16_t motor3508left_out;
	int16_t motorup_out;
	//int16_t yaw_out;
	//int16_t pitch_out[2];
	
}can1_senddata;

typedef struct _vec8f vec8f;

extern can1_feedback can1Feedback;
extern can1_senddata can1Senddata;
extern unsigned char can1_tx_success_flag;
extern int Friction_module_offlinecheck;
void CAN1_Init(void);
void CAN1_RX0_IRQHandler(void);
void CAN1_TX_IRQHandler(void);
void TDT_Can1_OutUpdate1(vec8f* controllerOut);
void Can1_Send_Data_to_Frict(can1_senddata *);


#endif

