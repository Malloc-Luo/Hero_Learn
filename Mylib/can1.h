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

typedef struct _vec8f vec8f;

extern can1_feedback can1Feedback;
extern unsigned char can1_tx_success_flag;
extern int Friction_module_offlinecheck;
void CAN1_Init(void);
void CAN1_RX0_IRQHandler(void);
void CAN1_TX_IRQHandler(void);
void TDT_Can1_OutUpdate1(vec8f* controllerOut);


#endif

