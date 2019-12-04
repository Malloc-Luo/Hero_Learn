#include "yaw_control.h"

//��̨
PID_Config Yaw_Inner;
PID_Config Yaw_Outer;

//Yaw����
PID_Config Yaw_Chassis_Inner;
PID_Config Yaw_Chassis_Outer;

/*����ģʽѡ��
* return:
 * FOLLOW ���̸���
 * SEPARATION ���̷���
 */
u8 Chassis_Mode_Checkout(void)
{
	u8 Mode;
	if(NDJ6.ch[5] == 2)
		Mode = FOLLOW;
	if(NDJ6.ch[5] == 3)
		Mode = SEPARATION;
	
	return Mode;
}


/**
 * @brief Yaw�Ὺ����λ��־
 *       return SUCCESS
 *			 return UNSUCCESS
 */
u8 Yaw_Celib_Flag(void)
{
	static u8 counter1 = 0;
	static u8 counter2 = 0;
	u8 flag = UNSUCCESS;
	u8 stopflag = 0;
	
	if(ABS_int(YAW_MIDDLE - can2feedback.positionYaw) <= 20)
	{
		stopflag = 1;
		counter1++;
	}
	
	if(stopflag)
		counter2++;
	
	if(counter1==counter2 && counter1>3)
		flag = SUCCESS;
		
	if(counter2>=5)
	{
		counter2 = 0;
		counter1 = 0;
		stopflag = 0;
	}
	
	return flag;
}


/**
 * @brief Yaw�Ὺ���Զ���λ���Ե���������Ϊ��׼ 
 * 				���̻�е�Ƕ� 6850 YAW_MIDDLE
 */
float innerKi = 0.1;
float innerKp = 1.4;
float outerKi = 0.1;
float outerKp = 1.4;

void Yaw_Init(void)
{
	u8 YawPidflag = 0;
	
	float outerSetvalue = YAW_MIDDLE;
	float outerActualvalue = 0;
	float innerSetvalue, innerActualvalue;
	u8 counter = 0;
	
	if(YawPidflag == 0)
	{
		PID_Init(innerKp, innerKi, 0, &Yaw_Inner);
		PID_Init(outerKp, outerKi, 0, &Yaw_Outer);
		YawPidflag = 1;
	}
	
	innerSetvalue = Yaw_Outer.out;
	innerActualvalue = (float)can2feedback.yawspeed;
	
	/*�ڻ�����*/
	PID_Ctrl(innerSetvalue, innerActualvalue, &Yaw_Inner);
	
	can2senddata.yaw_out = (int16_t)Yaw_Inner.out;
	
	if(++counter >= 2)
	{
		outerActualvalue = (float)can2feedback.positionYaw;
	
		/*�⻷����*/
		PID_Ctrl(outerSetvalue, outerActualvalue, &Yaw_Outer);
	}
}


static int16_t left_right_data = 0, left_right_data_last = 0;
u8 Yaw_Stopflag = 0;

void Yaw_Control(void)
{
	static uint16_t counter = 0;
	static u8 pidInitFlag = 0;
	u8 Chassis_Mode;
	
	float innerActualvalue = 0;
	float innerSetvalue = 0, innnerActualvalue = 0;
	static float outerSetvalue = 0, outerActualvalue = 0;
	float Yaw_positionSetvalue = 0;
	
	//��ʼ��PID������
	if(pidInitFlag == 0 )
	{
		PID_Init(1.8, 0.1, 0, &Yaw_Inner);
		PID_Init(1.8, 0.1, 0, &Yaw_Outer);
		pidInitFlag = 1;
	}
	
	innerSetvalue = Yaw_Outer.out;
	
	//�ڻ�����ֵΪ�����ǽ��ٶ�ֵ
	innerActualvalue = mpu6050Top.gyro.dps.data[z];
	
	//�ڻ�����
	PID_Ctrl(innerSetvalue, innerActualvalue, &Yaw_Inner);
	
	can2senddata.yaw_out = (int16_t)Yaw_Inner.out;
	if(counter>=2)
	{
		left_right_data = NDJ6.ch[0];
		
		//�ж�ң���ź��Ƿ�仯
		if(left_right_data==0 && left_right_data_last!=0)
		{
			pidInitFlag = 0;
			Yaw_Stopflag = 1;
		}
		if(left_right_data==0 && left_right_data_last==0)
			Yaw_Stopflag = 0;
		
		//ģʽ���
		Chassis_Mode = Chassis_Mode_Checkout();
		
		Yaw_positionSetvalue -= NDJ6.ch[0] * 0.6 * 0.05;
		
		outerSetvalue = Yaw_positionSetvalue;
		
		//Yaw��ŷ������Ϊ�⻷����ֵ
		outerActualvalue = gimbalTopAngle.yaw;
		
		//�⻷����
		PID_Ctrl(outerSetvalue, outerActualvalue, &Yaw_Outer);
		
	}
	counter ++;
	if(counter>2)
		counter = 0;
	
}

/**
 * @brief ���̸���ģʽ
 * 				* ����̨λ�û�����ȡMPU6050���ݣ�����Yaw����
 *				* ��Yaw����λ�û�����ȡYaw������е�Ƕȣ����Ƶ��̵����ת
 */
PID_Config Chassis_Yaw;
float Chassis_Follow_value = 0;

void Chassis_Follow_Mode(void)
{
	float innerActualvalue = 0;
	float innerSetvalue = 0, innnerActualvalue = 0;
	static float outerSetvalue = 0, outerActualvalue = 0;
	static u8 counter = 0;
	static u8 pidInitFlag = 0;
	
	outerSetvalue = YAW_MIDDLE;
	
	if(pidInitFlag == 0)
	{
		PID_Init(1.4, 0.0, 0.0, &Yaw_Chassis_Inner);
		PID_Init(1.4, 0.0, 0.0, &Yaw_Chassis_Outer);
		pidInitFlag = 1;
	}
	
	innerSetvalue = Yaw_Chassis_Outer.out;
	innerActualvalue = can2feedback.yawspeed;
	
	PID_Ctrl(innerSetvalue, innerActualvalue, &Yaw_Chassis_Inner);
	
	Chassis_Follow_value = Yaw_Chassis_Inner.out;
	
	if(counter>0)
	{
		outerActualvalue = can2feedback.positionYaw;
		
		
		PID_Ctrl(outerSetvalue, outerActualvalue, &Yaw_Chassis_Outer);
	}
	
	if(Yaw_Celib_Flag() == SUCCESS)
		pidInitFlag = 0;
	
	counter ++;
	
	if(counter>1)
		counter = 0;
}


