#ifndef __YAW_CONTROL_H__
#define __YAW_CONTROL_H__

#include "board.h"

/*底盘跟随模式 底盘跟随模式*/
#define FOLLOW 0x01
#define SEPARATION 0x02
//#define YAW_MIDDLE 6850
#define YAW_MIDDLE 2700
#define SUCCESS 0x00
#define UNSUCCESS !SUCCESS

typedef struct PID_Controller PID_Config;
typedef struct PID_Controller * PID_Def;

//Yaw轴内环(速度环)和外环(位置环)PID控制器，控制云台
extern PID_Config Yaw_Inner;
extern PID_Config Yaw_Outer;

//控制底盘
extern PID_Config Yaw_Chassis_Inner;
extern PID_Config Yaw_Chassis_Outer;

//底盘跟随分量
extern float Chassis_Follow_value;

/**
 * @brief Yaw轴开机归位标志
 *       return SUCCESS
 *			 return UNSUCCESS
 */
extern u8 Yaw_Celib_Flag(void);
void Yaw_Control(void);
void Yaw_Init(void);
u8 Chassis_Mode_Checkout(void);
void Chassis_Follow_Mode(void);

#endif

