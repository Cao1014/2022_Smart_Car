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
	
    Angle.SetAngle=0;			//设置的速度值
    Angle.ActualAngle=0;	//实际的速度值
    Angle.err=0.0;				  //实际误差
    Angle.err_last=0.0;		  //上一次的误差
    Angle.angular=0;		      //输出的占空比
    Angle.integral=0.0;		//积分值

    Angle.Kp = 0.5;  
    Angle.Ki = 0.03; 
    Angle.Kd = 0.0;
	
	
    Angle.umax = +180;		//最大值不能大于22
    Angle.umin = -180;
	
    LL_mDelay(20);
	
		LL_TIM_EnableIT_UPDATE(TIM7);//开启中断
}

short Angle_PID(float angle, float current)
{
	static int index;//用于积分分离,即通过该值来确定是否使用积分常数
	
	/*当误差值比较大时，取消积分作用，以免由于积分作用使得系统稳定性降低，超调量增大；
	  当被控量接近目标值时（即误差较小时），引入积分控制，以消除净差，提高控制精度。*/
	
	
	memcpy(&Angle.SetAngle, &angle, sizeof (float));
	Angle.ActualAngle = current;
	
	if(Angle.SetAngle >= Angle.umax) Angle.SetAngle = Angle.umax;
	else if(Angle.SetAngle <= Angle.umin) Angle.SetAngle = Angle.umin;
	
	if(fabs(Angle.SetAngle - Angle.ActualAngle) > 1.0f)
	Angle.err=Angle.SetAngle-Angle.ActualAngle;
	else Angle.err = 0.0;

	if(Angle.ActualAngle>Angle.umax)  //若实际值大于最大值
	{
	   if(fabs(Angle.err)>int_thres)      
		{
			index = 0;
		}
	   else
	   {
			index = 1;
			if(Angle.err < 0) Angle.integral += Angle.err; //积分分离过程
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
			if(Angle.err > 0) Angle.integral += Angle.err; //积分分离过程
	   }
	}
	else
	{
		if(fabs(Angle.err)>int_thres)                    //积分分离过程
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
