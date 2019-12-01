#include "fdbus.h"
_Bool NDJ6_ch8,NDJ6_ch9,NDJ6_chR,NDJ6_chE,NDJ6_chQ,NDJ6_chF,NDJ6_chG,NDJ6_chV,NDJ6_chB,NDJ6_chC,NDJ6_chX,NDJ6_chZ=0;
void fdbus()
{
		
	if(NDJ6.ch_last[8] == 0 && NDJ6.ch[8] == 1)
	NDJ6_ch8 ++;
	if(NDJ6.ch_last[9] == 0&&NDJ6.ch[9] == 1)
	NDJ6_ch9 ++;
	if(NDJ6.last_key.R == 0&&NDJ6.key.bit.R == 1)
	NDJ6_chR ++;
	if(NDJ6.last_key.B == 0&&NDJ6.key.bit.B == 1)
	NDJ6_chB ++;
	if(NDJ6.last_key.Z == 0&&NDJ6.key.bit.Z == 1)
	NDJ6_chZ ++;
	if(NDJ6.last_key.Q == 0&&NDJ6.key.bit.Q == 1)
	NDJ6_chQ ++;
	if(NDJ6.last_key.E == 0&&NDJ6.key.bit.E == 1)
	NDJ6_chE ++;
	if(NDJ6.last_key.X == 0&&NDJ6.key.bit.X == 1)
	NDJ6_chX ++;
	if(NDJ6.last_key.G == 0&&NDJ6.key.bit.G == 1)
	NDJ6_chG ++;
	if(NDJ6.last_key.F == 0&&NDJ6.key.bit.F == 1)
	NDJ6_chF ++;

}