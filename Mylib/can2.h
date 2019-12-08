#ifndef __CAN2_H__
#define __CAN2_H__
#include "board.h"

typedef struct 
{
	int16_t motor3508[4];
//	int16_t motor6020[4];
  int16_t positionYaw;
	int16_t yawspeed;
	u8 pho_left;
	u8 pho_right;
	u8 Picker_Status;
	u8 Cover_Status;
	u8 robot_Level;
}can2_feedback;

typedef struct
{
	int16_t motor3508out[4];
	int16_t yaw_out;
	int16_t pitch_out[2];
	
}can2_senddata;

extern can2_feedback can2feedback;
extern can2_senddata can2senddata_chassis;
extern can2_senddata can2senddata_yaw;

void Can2_Init(void);
void Can2_Send_Data_to_Chassis(can2_senddata *);
void Can2_Send_Data_to_Yaw(can2_senddata *);


#endif  


