#ifndef __PITCH_CONTROL__
#define __PITCH_CONTROL__
#include "board.h"

#define PITCHCELIB 4060
#define PITCH_MAX 3800
#define PITCH_MIDDLE 0

u8 Pitch_Celib_Flag(void);

void Pitch_Init(void);

void Pitch_Control(void);

#endif

