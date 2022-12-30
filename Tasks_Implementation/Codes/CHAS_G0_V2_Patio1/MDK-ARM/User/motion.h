/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MOTION_H__
#define __MOTION_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
	
#define int_thres 10
	
typedef struct
{
	
	short L_speed;
	short A_speed;
	
	uint16_t PWM_LF;
	uint16_t PWM_RF;
	uint16_t PWM_LB;
	uint16_t PWM_RB;
	
	short speedLF;
	short speedRF;
	short speedLB;
	short speedRB;
	
} ESC_COM;

typedef struct
{
	float SetAngle;           //定义设定值
  float ActualAngle;        //定义实际值
  float err;                	//定义偏差值
  float err_last;            	//定义上一个偏差值
  float Kp,Ki,Kd;            	//定义比例、积分、微分系数
  short angular;            	//定义电压值（控制执行器的变量）
  float integral;            	//定义积分值
  short umax;					//正饱和值 
  short umin;					//负饱和值
	
} PID_Para;

void Angle_init(void);
short Angle_PID(float angle, float current);

void Drive(short linear, short angular);
void SpeedToPWM(short speedLF, short speedRF, short speedLB, short speedRB);
	
#endif /* __MOTION_H__ */
