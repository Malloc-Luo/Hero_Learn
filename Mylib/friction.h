#ifndef __FRICTION_H__
#define __FRICTION_H__
#include "board.h"

typedef enum
{
	SPEED,
	POSITION
	
}Top_Frict_Mode;

extern u8 left_right_frict_power_flag;
extern u8 up_frict_power_flag;

void Friction_control(void);

void Trigger_control(u8 mode);


#endif
