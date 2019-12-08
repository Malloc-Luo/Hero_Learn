#ifndef __FRICTION_H__
#define __FRICTION_H__
#include "board.h"

extern u8 left_right_frict_power_flag;
extern u8 up_frict_power_flag;
extern int16_t left_right_setvalue;
extern int16_t top_setvalue;

void Friction_control(void);


#endif
