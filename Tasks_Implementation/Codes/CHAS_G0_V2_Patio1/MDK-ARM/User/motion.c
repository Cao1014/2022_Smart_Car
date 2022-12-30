#include "motion.h"
#include "tim.h"
#include "math.h"
#include "string.h"

ESC_COM G0;
PID_Para Angle;

volatile short LF, RF, LB, RB;

/**
  * @brief  PID initialization
  * @param  None
  * @retval None
  */
void Angle_init(void){
	
    Angle.SetAngle=0;			//���õ��ٶ�ֵ
    Angle.ActualAngle=0;	//ʵ�ʵ��ٶ�ֵ
    Angle.err=0.0;				  //ʵ�����
    Angle.err_last=0.0;		  //��һ�ε����
    Angle.angular=0;		      //�����ռ�ձ�
    Angle.integral=0.0;		//����ֵ

    Angle.Kp = 0.5;  
    Angle.Ki = 0.03; 
    Angle.Kd = 0.0;
	
	
    Angle.umax = +180;		//���ֵ���ܴ���22
    Angle.umin = -180;
	
    LL_mDelay(20);
	
		LL_TIM_EnableIT_UPDATE(TIM7);//�����ж�
}

short Angle_PID(float angle, float current)
{
	static int index;//���ڻ��ַ���,��ͨ����ֵ��ȷ���Ƿ�ʹ�û��ֳ���
	
	/*�����ֵ�Ƚϴ�ʱ��ȡ���������ã��������ڻ�������ʹ��ϵͳ�ȶ��Խ��ͣ�����������
	  ���������ӽ�Ŀ��ֵʱ��������Сʱ����������ֿ��ƣ������������߿��ƾ��ȡ�*/
	
	
	memcpy(&Angle.SetAngle, &angle, sizeof (float));
	Angle.ActualAngle = current;
	
	if(Angle.SetAngle >= Angle.umax) Angle.SetAngle = Angle.umax;
	else if(Angle.SetAngle <= Angle.umin) Angle.SetAngle = Angle.umin;
	
	if(fabs(Angle.SetAngle - Angle.ActualAngle) > 1.0f)
	Angle.err=Angle.SetAngle-Angle.ActualAngle;
	else Angle.err = 0.0;

	if(Angle.ActualAngle>Angle.umax)  //��ʵ��ֵ�������ֵ
	{
	   if(fabs(Angle.err)>int_thres)      
		{
			index = 0;
		}
	   else
	   {
			index = 1;
			if(Angle.err < 0) Angle.integral += Angle.err; //���ַ������
	   }
	}
	else if(Angle.ActualAngle<Angle.umin)
	{
		if(fabs(Angle.err)>int_thres)      
		{
			index = 0;
		}
	   else
	   {
			index = 1;
			if(Angle.err > 0) Angle.integral += Angle.err; //���ַ������
	   }
	}
	else
	{
		if(fabs(Angle.err)>int_thres)                    //���ַ������
		{
			index = 0;
		}
		else
		{
			index = 1;
			Angle.integral += Angle.err;
		}
	}

	Angle.angular=Angle.Kp*Angle.err+index*Angle.Ki*Angle.integral+Angle.Kd*(Angle.err-Angle.err_last);
	
	Angle.err_last = Angle.err;
	
		if(Angle.angular > 15) 		
			Angle.angular = 15;
		else if(Angle.angular < -15)  
			Angle.angular = -15;
	
	return Angle.angular;
}

void Drive(short linear, short angular)
{
	if(linear > 15) linear = 15;
	if(linear < -15) linear = -15;
	
	if(angular > 15) angular = 15;
	if(angular < -15) angular = -15;
	
	if(angular == 0)
	{
			LF = LB = - linear;
			RF = RB = + linear;
	}
	else
	{
		LF = LB = RF = RB = angular;
	}
		
	SpeedToPWM(LF,RF,LB,RB);
}

void SpeedToPWM(short speedLF, short speedRF, short speedLB, short speedRB)
{
	if(speedLF != 0)
	G0.PWM_LF = (speedLF + 50) * 10;
	else G0.PWM_LF = 0;
	WRITE_REG(TIM3->CCR1, G0.PWM_LF);
	
	if(speedRF != 0)
	G0.PWM_RF = (speedRF + 50) * 10;
	else G0.PWM_RF = 0;
	WRITE_REG(TIM3->CCR2, G0.PWM_RF);
	
	if(speedLB != 0)
	G0.PWM_LB = (speedLB + 50) * 10;
	else G0.PWM_LB = 0;
	WRITE_REG(TIM3->CCR3, G0.PWM_LB);
	
	if(speedRB != 0)
	G0.PWM_RB = (speedRB + 50) * 10;
	else G0.PWM_RB = 0;
	WRITE_REG(TIM3->CCR4, G0.PWM_RB);

}
