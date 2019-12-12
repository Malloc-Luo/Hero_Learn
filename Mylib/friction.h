#ifndef __FRICTION_H__
#define __FRICTION_H__
#include "board.h"

#define TOP_MOVE 119570
#define FIRE     100
#define WAIT     0

typedef enum
{
	SPEED,
	POSITION
	
}Top_Frict_Mode;

extern u8 left_right_frict_power_flag;
extern u8 up_frict_power_flag;
extern u8 shootflag;

void Friction_control(void);

void Trigger_control(Top_Frict_Mode mode);

void Shoot(void);

void Switch_Init(void);


#endif
