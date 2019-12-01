#ifndef __TDT_HERO__
#define __TDT_HERO__

#include "stm32f4xx.h"
#include "stdlib.h"
#include "stdint.h"


#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))

#define FB_MAXSPEED		10000.0f/660.0f	  /**< 底盘前后的最大速度*/
#define LR_MAXSPEED		8000.0f/660.0f	  /**< 底盘左右的最大速度*/
#define ROT_MAXASPEED	8000.0f/660.0f	  /**< 底盘旋转的最大角速度*/

#define FB_MAXSPEED_TRACK    7000.0f/660.0f  	/**< 履带前后的最大速度*/
#define ROT_MAXSPEED_TRACK   3000.0f/660.0f  	/**< 履带旋转的最大速度*/

#define MAXSET3508  12000  /**< 3508电机的最大给定值*/
#define MAXSET3510  10000  /**< 3510电机的最大给定值*/
#define MAXSET2006  10000 /**< 2006电机的最大给定值*/
#define MAXSET6623  5000  /**< 6623电机的最大给定值*/
#define MAXSET6020  30000  /**< 6020电机的最大给定值*/
#define MAXSETRM35  32767 /**< RM35电机的最大速度设定值*/

#define RM35MOTOR1 1
#define RM35MOTOR2 2
#define RM35MOTOR3 3
#define RM35MOTOR4 4

#define MID_ANGLE_YAW 2700
/**
* @struct  _vec2f
* @brief 二维float向量结构体
*/
typedef struct _vec2f
{
	float data[2];
}vec2f;
/**
* @struct  _vec2f
* @brief 二维float向量结构体
*/
typedef struct _vec2fint16_t
{
	short data[2];
}vec2fint16_t;

/**
* @struct  _vec3f
* @brief 三维float向量结构体
*/
typedef struct _vec3f
{
	float data[3];
}vec3f;

/**
* @struct  _vec3int16
* @brief 三维int16向量结构体
*/
typedef struct _vec3int16
{
	 short data[3];
}vec3int16;

/**
* @struct  _vec4f
* @brief 四维float向量结构体
*/
typedef struct _vec4f
{
	float data[4];
}vec4f;
/**
* @struct  _vec5f
* @brief 五维float向量结构体
*/
typedef struct _vec5f
{
	float data[5];
}vec5f;
/**
* @struct  _vec6f
* @brief 六维float向量结构体
*/
typedef struct _vec6f
{
	float data[6];
}vec6f;
/**
* @struct  _vec8f
* @brief 六维float向量结构体
*/ 
struct _vec8f
{
	float data[8];
	
};

/**
* @struct  _pid
* @brief pid结构体
*/
typedef struct _pid
{
	float setValue;
	float feedbackValue;
	float TfeedbackValue;	
	float last_feedbackValue;	
	float error;
	float lastError;
	float deltaError;
	float lastdeltaError;
	float integralError;
	float integralErrorMax;
	float kp;
	float ki;
	float kd;
	float pOut;
	float iOut;
	float dOut;
	float out;
}pid;
/**
* @struct  _PowerCtrl
* @brief 功率控制结构体
*/
typedef struct _PowerCtrl
{
	u8 UseBackup;					//使用保留备用功率
	u8 HardOnly;					//纯硬件限功率
	u8 AvoidMode;					//躲避模式
	u8 ShiftDown;					//操作手shift摁下
	u8 DisableLimit;				//禁用输出限幅，调试用
	u8 PowerMode;					//能量模式，1为超级电容，0为电池
	u8 SuperPowerOffline;			//模块离线
	u8 SuperPowerReady;				//超级电容状态
	u8 SuperPowerState;				//功率模块状态，1为开启，0关闭电容
	u8 OverPowerCase;				//捕获裁判系统单个
	u8 SuperPowerOfflineCheck;		//离线计数
	int16_t RemainPower;			//剩余能量
	float PowerOutLimt_P;			//输出限制系数
	float SuperPowerRemain_P;		//超级电容剩余百分比
	int16_t Jgmt_RealPower;			//擦破系统实时功率值 w
	float SuperPower_RealPower;		//超级电容实时功率值 w
	float SuperPower_V;				//超级电容实时电压
	float PowerSetValueLimt_P;		//速度设定限制系数
	float OverPowerExtLimt;			//超功率额外限制
	float RotateSpdReduce_P;		//衰减系数,转弯时减速

}PowerControl;

#endif
