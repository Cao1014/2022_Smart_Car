#ifndef __CONTROL_H
#define __CONTROL_H

#include "main.h"
#include "adc.h"
#include "tim.h"

#define Vref 3.334 /* ADC �ο���ѹ */
#define ADC_SAMPLE_SIZE 32
#define ENC_SAMPLE_SIZE 4
#define PWM_SAMPLE_SIZE 4

#define BEEP_SIZE 200
#define int_thres 16

typedef struct
{	
	float Vin;				//�����ѹֵ
	
	uint8_t M_Direction;	//���ת������
	uint8_t M_CNT;
	uint8_t M_ENC;				//��ֵ�˲���ı�����ֵ
	
	uint16_t PWM_CNT;
	short PWM_Length;	//����PWM�źų���
	short speed;
	
	uint8_t PID_Direction;
	short SetSpeed;           //�����趨ֵ
  short ActualSpeed;        //����ʵ��ֵ
  float err;                	//����ƫ��ֵ
  float err_last;            	//������һ��ƫ��ֵ
  float Kp1,Ki1,Kd1;            	//������������֡�΢��ϵ��
	float Kp2,Ki2,Kd2;            	//������������֡�΢��ϵ��
  uint16_t duty;            	//�����ѹֵ������ִ�����ı�����
  float integral;            	//�������ֵ
  short umax;					//������ֵ 
  short umin;					//������ֵ
	
} ESC_Para;


void TIM14_Sample(void);
void Mean_Filter(void);

void PWM_Detection(void);
void PWM_TO_SPEED(void);

void PID_init(void);
void PID_realize(short speed);

#endif
