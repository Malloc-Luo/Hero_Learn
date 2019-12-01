#ifndef __TDT_HERO__
#define __TDT_HERO__

#include "stm32f4xx.h"
#include "stdlib.h"
#include "stdint.h"


#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))

#define FB_MAXSPEED		10000.0f/660.0f	  /**< ����ǰ�������ٶ�*/
#define LR_MAXSPEED		8000.0f/660.0f	  /**< �������ҵ�����ٶ�*/
#define ROT_MAXASPEED	8000.0f/660.0f	  /**< ������ת�������ٶ�*/

#define FB_MAXSPEED_TRACK    7000.0f/660.0f  	/**< �Ĵ�ǰ�������ٶ�*/
#define ROT_MAXSPEED_TRACK   3000.0f/660.0f  	/**< �Ĵ���ת������ٶ�*/

#define MAXSET3508  12000  /**< 3508�����������ֵ*/
#define MAXSET3510  10000  /**< 3510�����������ֵ*/
#define MAXSET2006  10000 /**< 2006�����������ֵ*/
#define MAXSET6623  5000  /**< 6623�����������ֵ*/
#define MAXSET6020  30000  /**< 6020�����������ֵ*/
#define MAXSETRM35  32767 /**< RM35���������ٶ��趨ֵ*/

#define RM35MOTOR1 1
#define RM35MOTOR2 2
#define RM35MOTOR3 3
#define RM35MOTOR4 4

#define MID_ANGLE_YAW 2700
/**
* @struct  _vec2f
* @brief ��άfloat�����ṹ��
*/
typedef struct _vec2f
{
	float data[2];
}vec2f;
/**
* @struct  _vec2f
* @brief ��άfloat�����ṹ��
*/
typedef struct _vec2fint16_t
{
	short data[2];
}vec2fint16_t;

/**
* @struct  _vec3f
* @brief ��άfloat�����ṹ��
*/
typedef struct _vec3f
{
	float data[3];
}vec3f;

/**
* @struct  _vec3int16
* @brief ��άint16�����ṹ��
*/
typedef struct _vec3int16
{
	 short data[3];
}vec3int16;

/**
* @struct  _vec4f
* @brief ��άfloat�����ṹ��
*/
typedef struct _vec4f
{
	float data[4];
}vec4f;
/**
* @struct  _vec5f
* @brief ��άfloat�����ṹ��
*/
typedef struct _vec5f
{
	float data[5];
}vec5f;
/**
* @struct  _vec6f
* @brief ��άfloat�����ṹ��
*/
typedef struct _vec6f
{
	float data[6];
}vec6f;
/**
* @struct  _vec8f
* @brief ��άfloat�����ṹ��
*/ 
struct _vec8f
{
	float data[8];
	
};

/**
* @struct  _pid
* @brief pid�ṹ��
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
* @brief ���ʿ��ƽṹ��
*/
typedef struct _PowerCtrl
{
	u8 UseBackup;					//ʹ�ñ������ù���
	u8 HardOnly;					//��Ӳ���޹���
	u8 AvoidMode;					//���ģʽ
	u8 ShiftDown;					//������shift����
	u8 DisableLimit;				//��������޷���������
	u8 PowerMode;					//����ģʽ��1Ϊ�������ݣ�0Ϊ���
	u8 SuperPowerOffline;			//ģ������
	u8 SuperPowerReady;				//��������״̬
	u8 SuperPowerState;				//����ģ��״̬��1Ϊ������0�رյ���
	u8 OverPowerCase;				//�������ϵͳ����
	u8 SuperPowerOfflineCheck;		//���߼���
	int16_t RemainPower;			//ʣ������
	float PowerOutLimt_P;			//�������ϵ��
	float SuperPowerRemain_P;		//��������ʣ��ٷֱ�
	int16_t Jgmt_RealPower;			//����ϵͳʵʱ����ֵ w
	float SuperPower_RealPower;		//��������ʵʱ����ֵ w
	float SuperPower_V;				//��������ʵʱ��ѹ
	float PowerSetValueLimt_P;		//�ٶ��趨����ϵ��
	float OverPowerExtLimt;			//�����ʶ�������
	float RotateSpdReduce_P;		//˥��ϵ��,ת��ʱ����

}PowerControl;

#endif
