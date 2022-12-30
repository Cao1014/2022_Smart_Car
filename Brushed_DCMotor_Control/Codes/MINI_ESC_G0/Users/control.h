#ifndef __CONTROL_H
#define __CONTROL_H

#include "main.h"
#include "adc.h"
#include "tim.h"

#define Vref 3.334 /* ADC 参考电压 */
#define ADC_SAMPLE_SIZE 32
#define ENC_SAMPLE_SIZE 4
#define PWM_SAMPLE_SIZE 4

#define BEEP_SIZE 200
#define int_thres 16

typedef struct
{	
	float Vin;				//输入电压值
	
	uint8_t M_Direction;	//电机转动方向
	uint8_t M_CNT;
	uint8_t M_ENC;				//均值滤波后的编码器值
	
	uint16_t PWM_CNT;
	short PWM_Length;	//输入PWM信号长度
	short speed;
	
	uint8_t PID_Direction;
	short SetSpeed;           //定义设定值
  short ActualSpeed;        //定义实际值
  float err;                	//定义偏差值
  float err_last;            	//定义上一个偏差值
  float Kp1,Ki1,Kd1;            	//定义比例、积分、微分系数
	float Kp2,Ki2,Kd2;            	//定义比例、积分、微分系数
  uint16_t duty;            	//定义电压值（控制执行器的变量）
  float integral;            	//定义积分值
  short umax;					//正饱和值 
  short umin;					//负饱和值
	
} ESC_Para;


void TIM14_Sample(void);
void Mean_Filter(void);

void PWM_Detection(void);
void PWM_TO_SPEED(void);

void PID_init(void);
void PID_realize(short speed);

#endif
