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
	float SetAngle;           //�����趨ֵ
  float ActualAngle;        //����ʵ��ֵ
  float err;                	//����ƫ��ֵ
  float err_last;            	//������һ��ƫ��ֵ
  float Kp,Ki,Kd;            	//������������֡�΢��ϵ��
  short angular;            	//�����ѹֵ������ִ�����ı�����
  float integral;            	//�������ֵ
  short umax;					//������ֵ 
  short umin;					//������ֵ
	
} PID_Para;

void Angle_init(void);
short Angle_PID(float angle, float current);

void Drive(short linear, short angular);
void SpeedToPWM(short speedLF, short speedRF, short speedLB, short speedRB);
	
#endif /* __MOTION_H__ */
