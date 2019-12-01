#ifndef _Dbus_H
#define _Dbus_H

#include "board.h"

#define KEY_B			0x8000
#define KEY_V			0x4000
#define KEY_C			0x2000
#define KEY_X			0x1000
#define KEY_Z			0x0800
#define KEY_G			0x0400
#define KEY_F			0x0200
#define KEY_R			0x0100
#define KEY_E			0x0080
#define KEY_Q			0x0040
#define KEY_CTRL	0x0020
#define KEY_SHIFT	0x0010
#define KEY_D			0x0008
#define KEY_A			0x0004
#define KEY_S			0x0002
#define KEY_W			0x0001

#define deathzoom(x,zoom) (float)(x>0?(((x-zoom)<0)?0:x):((x+zoom)>0?0:x))

typedef __packed struct
{
	int16_t ch[10];
	int16_t ch_last[11];
  __packed union
  {
    uint16_t key_code;
    __packed struct 
    {
      uint16_t W:1;
      uint16_t S:1;
      uint16_t A:1;
      uint16_t D:1;
      uint16_t SHIFT:1;
      uint16_t CTRL:1;
      uint16_t Q:1;
      uint16_t E:1;
      uint16_t R:1;
      uint16_t F:1;
      uint16_t G:1;
      uint16_t Z:1;
      uint16_t X:1;
      uint16_t C:1;
      uint16_t V:1;
      uint16_t B:1;
    } bit;
  } key;
	__packed struct
	{
			int16_t value;
			uint8_t     B;			
			uint8_t     V;			
			uint8_t     C;			
			uint8_t     X;		
			uint8_t     Z;			
			uint8_t     G;		
			uint8_t     F;			
			uint8_t     R;			
			uint8_t     E;			
			uint8_t     Q;		
			uint8_t     CTRL;	
			uint8_t     SHIFT;	
			uint8_t     D;			
			uint8_t     A;			
			uint8_t     S;			
			uint8_t     W;			
	} last_key;
} rc;

extern int shoot_change,NDJ6keybit_G,NDJ6keybit_B,NDJ6keybit_C;

extern rc NDJ6;

void TDT_Dbus_Configuration(void);

#endif /*_Dbus_H*/
