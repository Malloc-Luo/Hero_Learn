#include "imu.h"

eulerAngle gimbalTopAngle;
eulerAngle gimbalBotAngle;

float Kp=0.6f;			//0.6f                           // proportional gain governs rate of convergence to accelerometer/magnetometer
float Ki=0.1f;			//0.1f                           // integral gain governs rate of convergence of gyroscope biases
float Kpp=0.1f,p_up;//0.6f                           // 互补滤波的pid参数
float Kii=0.1f,i_up;//0.1f
float Special_pitch,Special_error,MPU_pitch_angle;//互补滤波之后的pitch值
volatile float yaw_temp;
volatile float last_yaw_temp;
int yaw_count;
void TDT_IMUTopupdate(float half_T, vec3f* gyro, vec3f* acc)
{
	float vx, vy, vz;//(r系到b系的第三列)
	
  float norm;
  float ex, ey, ez;
	
	float gx = gyro->data[x];
	float gy = gyro->data[y];
	float gz = gyro->data[z];
	float ax = acc->data[x];
	float ay = acc->data[y];
	float az = acc->data[z];
	
	static float q0 = 1, q1 = 0, q2 = 0, q3 = 0;     // quaternion elements representing the estimated orientation
	static float exInt = 0, eyInt = 0, ezInt = 0;    // scaled integral error

		
	//acc数据归一化
  norm = sqrt(ax*ax + ay*ay + az*az);       
  ax = ax / norm;
  ay = ay / norm;
  az = az / norm;

  // estimated direction of gravity and flux (v and w)              估计重力方向和流量/变迁
  vx = 2*(q1*q3 - q0*q2);												//四元素中xyz的表示
  vy = 2*(q0*q1 + q2*q3);
  vz = 1 - 2*(q1*q1 + q2*q2);
	
  // error is sum of cross product between reference direction of fields and direction measured by sensors
  ex = (ay*vz - az*vy) ;                           					 //向量外积在相减得到差分就是误差
  ey = (az*vx - ax*vz) ;
  ez = (ax*vy - ay*vx) ;

//	if(NDJ6.ch[3] != 0 || NDJ6.key.bit.W != 0 || NDJ6.key.bit.S != 0)
//	{
//		Ki = 0.01f;
//		Kp = 0.06f;
//		exInt = 0;
//		eyInt = 0;
//		ezInt = 0;
//	}
//	else
//	{
//	  Ki = 0.6f;
//		Kp = 0.2f;
//	}

  exInt = exInt + ex *Ki *2 *half_T;								  //对误差进行积分
  eyInt = eyInt + ey *Ki *2 *half_T;
  ezInt = ezInt + ez *Ki *2 *half_T;
	
  // 积分限幅
	exInt = LIMIT(exInt, - IMU_INTEGRAL_LIM ,IMU_INTEGRAL_LIM );
	eyInt = LIMIT(eyInt, - IMU_INTEGRAL_LIM ,IMU_INTEGRAL_LIM );
	ezInt = LIMIT(ezInt, - IMU_INTEGRAL_LIM ,IMU_INTEGRAL_LIM );
	
  // adjusted gyroscope measurements
  gx = gx + Kp *(ex + exInt);					   						
  gy = gy + Kp *(ey + eyInt);				   							
  gz = gz + Kp *(ez + ezInt);					   					  							
	
  // integrate quaternion rate and normalise						   //四元素的微分方程
  q0 = q0 + (-q1*gx - q2*gy - q3*gz) *half_T;
  q1 = q1 + ( q0*gx + q2*gz - q3*gy) *half_T;
  q2 = q2 + ( q0*gy - q1*gz + q3*gx) *half_T;
  q3 = q3 + ( q0*gz + q1*gy - q2*gx) *half_T;

  // normalise quaternion
  norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 = q0 / norm;
  q1 = q1 / norm;
  q2 = q2 / norm;
  q3 = q3 / norm;

	gimbalTopAngle.yaw = f_atan2(2*q1*q2+2*q0*q3, -2*q2*q2-2*q3*q3+1) *57.3f;
  gimbalTopAngle.roll = f_atan2(2*q2*q3 + 2*q0*q1, -2*q1*q1 - 2*q2*q2 + 1) *57.3f;
	gimbalTopAngle.pitch = asin(-2*q1*q3 + 2*q0*q2) *57.3f;
	
	Special_pitch-= (gyro->data[x] / ANGLE_TO_RAD)* 2 * half_T;
	Special_error=((can1Feedback.positionPitch-MID_PITCH_ANGLE)/8192*360)-Special_pitch;
  p_up=Kpp*Special_error;
	i_up+= Special_error * Kii * 2 * half_T;
	i_up=LIMIT(i_up,-3.6,3.6);
	Special_pitch=Special_pitch+p_up+i_up;//互补滤波内容
	
	last_yaw_temp = yaw_temp;
	
	yaw_temp=atan2(2*q1*q2+2*q0*q3, -2*q2*q2-2*q3*q3+1) *57.3f;
	
	if(yaw_temp-last_yaw_temp>=200)  //yaw轴角度经过处理后变成连续的
	{
		yaw_count--;
	}
	else if (yaw_temp-last_yaw_temp<=-200)
	{
		yaw_count++;
	}
	gimbalTopAngle.yaw  = yaw_temp + yaw_count*360;  //yaw轴角度
		
}

void TDT_IMUBotupdate(float half_T, vec3f* gyro, vec3f* acc)
{
	float vx, vy, vz;//(r系到b系的第三列)
	
  float norm;
  float ex, ey, ez;
	
	float gx = gyro->data[x];
	float gy = gyro->data[y];
	float gz = gyro->data[z];
	float ax = acc->data[x];
	float ay = acc->data[y];
	float az = acc->data[z];
	
	static float q0 = 1, q1 = 0, q2 = 0, q3 = 0;     // quaternion elements representing the estimated orientation
	static float exInt = 0, eyInt = 0, ezInt = 0;    // scaled integral error

		
	//acc数据归一化
  norm = f_sqrt(ax*ax + ay*ay + az*az);       
  ax = ax / norm;
  ay = ay / norm;
  az = az / norm;

  // estimated direction of gravity and flux (v and w)              估计重力方向和流量/变迁
  vx = 2*(q1*q3 - q0*q2);												//四元素中xyz的表示
  vy = 2*(q0*q1 + q2*q3);
  vz = 1 - 2*(q1*q1 + q2*q2);
	
  // error is sum of cross product between reference direction of fields and direction measured by sensors
  ex = (ay*vz - az*vy) ;                           					 //向量外积在相减得到差分就是误差
  ey = (az*vx - ax*vz) ;
  ez = (ax*vy - ay*vx) ;

  exInt = exInt + ex *Ki *2 *half_T;								  //对误差进行积分
  eyInt = eyInt + ey *Ki *2 *half_T;
  ezInt = ezInt + ez *Ki *2 *half_T;
	
  // 积分限幅
	exInt = LIMIT(exInt, - IMU_INTEGRAL_LIM ,IMU_INTEGRAL_LIM );
	exInt = LIMIT(exInt, - IMU_INTEGRAL_LIM ,IMU_INTEGRAL_LIM );
	exInt = LIMIT(exInt, - IMU_INTEGRAL_LIM ,IMU_INTEGRAL_LIM );
	
  // adjusted gyroscope measurements
  gx = gx + Kp *(ex + exInt);					   						
  gy = gy + Kp *(ey + eyInt);				   							
  gz = gz + Kp *(ez + ezInt);					   					  							
	
  // integrate quaternion rate and normalise						   //四元素的微分方程
  q0 = q0 + (-q1*gx - q2*gy - q3*gz) *half_T;
  q1 = q1 + ( q0*gx + q2*gz - q3*gy) *half_T;
  q2 = q2 + ( q0*gy - q1*gz + q3*gx) *half_T;
  q3 = q3 + ( q0*gz + q1*gy - q2*gx) *half_T;

  // normalise quaternion
  norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 = q0 / norm;
  q1 = q1 / norm;
  q2 = q2 / norm;
  q3 = q3 / norm;

  gimbalBotAngle.yaw = f_atan2(2*q1*q2+2*q0*q3, -2*q2*q2-2*q3*q3+1) *57.3f;
  gimbalBotAngle.roll = f_atan2(2*q2*q3 + 2*q0*q1, -2*q1*q1 - 2*q2*q2 + 1) *57.3f;
  gimbalBotAngle.pitch = asin(-2*q1*q3 + 2*q0*q2) *57.3f; 	
}


