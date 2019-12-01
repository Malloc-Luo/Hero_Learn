#ifndef __YAW_CONTROL_H__
#define __YAW_CONTROL_H__



#include "board.h"

/*���̸���ģʽ ���̸���ģʽ*/
#define FOLLOW 0x01
#define SEPARATION 0x02
#define YAW_MIDDLE 6850
#define SUCCESS 0x00
#define UNSUCCESS !SUCCESS

//Yaw���ڻ�(�ٶȻ�)���⻷(λ�û�)PID������
extern PID_Config Yaw_Inner;
extern PID_Config Yaw_Outer;

/**
 * @brief Yaw�Ὺ����λ��־
 *       return SUCCESS
 *			 return UNSUCCESS
 */
extern u8 Yaw_Celib_Flag(void);

void Yaw_InnerControl(void);
void Yaw_Init(void);
u8 Chassis_Mode_Checkout(void);

#endif

