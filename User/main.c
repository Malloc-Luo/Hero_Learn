#include "board.h"

int shootflag=0;
int main(void)
{	
	Init_All();
	
 	while(1)
	{			
		//GPIOB7=GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_7);  
		fdbus();
  }
}
