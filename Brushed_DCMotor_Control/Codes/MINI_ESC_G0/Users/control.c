#include "control.h"
#include "math.h"
#include "arm_math.h"
#include "string.h"

extern short aADCxConvertedData[ADC_CONVERTED_DATA_BUFFER_SIZE];

short adc_rawdata[ADC_SAMPLE_SIZE] = {0};
short adc_mean[1] = {0};

short enc_rawdata[ENC_SAMPLE_SIZE] = {0};
short enc_mean[1] = {0};

short pwm_rawdata[PWM_SAMPLE_SIZE] = {0};
short pwm_mean[1] = {0};

short V_pin;

uint16_t adc_counter;
uint16_t enc_counter;
uint16_t beep_counter;
uint16_t pwm_counter;

uint8_t PWM_edge = 0;

ESC_Para MyESC;

void TIM14_Sample(void)
{
	if(LL_TIM_IsActiveFlag_UPDATE(TIM14) == SET)
		{
			LL_TIM_ClearFlag_UPDATE(TIM14);//清除中断标志
			
			LL_GPIO_TogglePin(GPIOC,GPIO_PIN_15);
			
			
			adc_rawdata[adc_counter] = aADCxConvertedData[0];
			adc_counter++;
			
			//0x00 (Counter used as upcounter) or 0x10 (Counter used as downcounter)
			MyESC.M_Direction = LL_TIM_GetCounterMode(TIM3);
			MyESC.M_CNT = (uint8_t)(LL_TIM_GetCounter(TIM3));
			
			if(MyESC.M_Direction == 0x00 && MyESC.M_CNT !=0 && MyESC.PID_Direction == 0x00)
			{
				if(MyESC.M_CNT > 128) MyESC.M_CNT = 256 - MyESC.M_CNT;
				enc_rawdata[enc_counter] = MyESC.M_CNT;
			}
			else if(MyESC.M_Direction == 0x10 && MyESC.M_CNT !=0 && MyESC.PID_Direction == 0x10)
			{
				if(MyESC.M_CNT < 128) MyESC.M_CNT = 256 - MyESC.M_CNT;
				enc_rawdata[enc_counter] = 256 - MyESC.M_CNT;
			}
			else if(MyESC.M_CNT == 0)
				enc_rawdata[enc_counter] = 0;
			
			enc_counter++;
			WRITE_REG(TIM3->CNT, 0);
			
			pwm_rawdata[pwm_counter] = MyESC.PWM_CNT;
			pwm_counter++;
			
			if (adc_counter == ADC_SAMPLE_SIZE)
				{
					adc_counter = 0;
				}
				
			if (enc_counter == ENC_SAMPLE_SIZE)
				{
					enc_counter = 0;
				}
				
			if (pwm_counter == PWM_SAMPLE_SIZE)
				{
					pwm_counter = 0;
				}
//			if (MyESC.err !=0)
//			{
//				beep_counter++;
//				LL_GPIO_SetOutputPin(GPIOA,GPIO_PIN_4);
//			}
//			else if (MyESC.err ==0 || beep_counter == BEEP_SIZE)
//			{
//				beep_counter = 0;
//				LL_GPIO_ResetOutputPin(GPIOA,GPIO_PIN_4);
//			}
		}
}

void PWM_Detection(void)
{
	if(LL_TIM_IsActiveFlag_CC1(TIM16))
	{

			if(PWM_edge == 0)          //捕获上升沿
			{
				TIM16->CNT = 0;
				
				LL_TIM_ClearFlag_CC1(TIM16);	
				LL_TIM_DisableIT_CC1(TIM16); 	//关闭中断
				LL_TIM_IC_SetPolarity(TIM16,LL_TIM_CHANNEL_CH1,LL_TIM_IC_POLARITY_FALLING);//切换捕获极性
				LL_TIM_EnableIT_CC1(TIM16);   //切换捕获极性后需重新启动
 
				PWM_edge = 1;          	 //上升沿、下降沿捕获标志位
			}
			else if(PWM_edge == 1)     //捕获下降沿
			{
				MyESC.PWM_CNT = READ_REG(TIM16->CCR1);                       //获取下降沿时间点
				
				LL_TIM_ClearFlag_CC1(TIM16);
				LL_TIM_DisableIT_CC1(TIM16); 	//关闭中断
				LL_TIM_IC_SetPolarity(TIM16,LL_TIM_CHANNEL_CH1,LL_TIM_IC_POLARITY_RISING);//切换捕获极性
				LL_TIM_EnableIT_CC1(TIM16);   //切换捕获极性后需重新启动
				
				PWM_edge = 0;
			}
	}
}

void PWM_TO_SPEED(void)
{	
	if(V_pin > 1450) /*********限幅***********/
	{
		if( READ_REG(TIM16->CNT) > 0x2BC)
		{
			MyESC.speed = 0;
			MyESC.PWM_CNT = 0;	
		}
		else
		{
			if(MyESC.PWM_Length < 32 || MyESC.PWM_Length > 68)
			{
				MyESC.speed = 0;
			}
			else 
			{
				MyESC.speed = MyESC.PWM_Length - 50;
			}
		}
	}
	else 
	{
		MyESC.speed = 0;
//		LL_GPIO_TogglePin(GPIOA,GPIO_PIN_4);
		LL_GPIO_TogglePin(GPIOC,GPIO_PIN_14);
		LL_mDelay(1);
	}
}

void Mean_Filter(void)
{
	arm_mean_q15(adc_rawdata,ADC_SAMPLE_SIZE,&adc_mean[0]);
	arm_mean_q15(enc_rawdata,ENC_SAMPLE_SIZE,&enc_mean[0]);
	arm_mean_q15(pwm_rawdata,PWM_SAMPLE_SIZE,&pwm_mean[0]);
	
	//计算输入电压 24V值大概为1566 22.2V值大概为1449
	memcpy(&V_pin,&adc_mean[0],sizeof(short));
	memcpy(&MyESC.M_ENC,&enc_mean[0],sizeof(short));
	memcpy(&MyESC.PWM_Length,&pwm_mean[0],sizeof(short));
//	MyESC.Vin = (float)V_pin / 4096.0f * Vref * (39.0 + 2.2) / 2.2;
}

/**
  * @brief  PID initialization
  * @param  None
  * @retval None
  */
void PID_init(void){
	
    MyESC.SetSpeed=0;			//设置的速度值
    MyESC.ActualSpeed=0;	//实际的速度值
    MyESC.err=0.0;				  //实际误差
    MyESC.err_last=0.0;		  //上一次的误差
    MyESC.duty=0;		      //输出的占空比
    MyESC.integral=0.0;		//积分值

    MyESC.Kp1 = 36.0;  //RB 18 Other 36
    MyESC.Ki1 = 1.2;   //LF 1.0 Other 1.2
    MyESC.Kd1 = 1.0;   //RF 0 Other 1.0
		
		MyESC.Kp2 = 36.0;  //RB 18 Other 36
    MyESC.Ki2 = 1.2; 
    MyESC.Kd2 = 1.0;
	
    MyESC.umax = +15;		//最大值不能大于22
    MyESC.umin = -15;
	
    LL_mDelay(20);
}

/**
  * @brief  Motor 1 PI Speed closed loop
  * @param  spinning direction, programmed speed, kp, ki, kd
  * @note   采用积分分离PID
  * @retval PWM占空比
  */
void PID_realize(short speed)
{
	
	if(speed > 0) 			
		MyESC.PID_Direction = 0x00;
	else if(speed < 0) 	
		MyESC.PID_Direction = 0x10;
	else 								
		MyESC.PID_Direction = 0x20;
	
	static int index;//用于积分分离,即通过该值来确定是否使用积分常数
	
	/*当误差值比较大时，取消积分作用，以免由于积分作用使得系统稳定性降低，超调量增大；
	  当被控量接近目标值时（即误差较小时），引入积分控制，以消除净差，提高控制精度。*/
	
	
	
	MyESC.ActualSpeed = MyESC.M_ENC;
	
	if(MyESC.PID_Direction == 0x00)
		MyESC.SetSpeed = speed;
	else if(MyESC.PID_Direction == 0x10)
		arm_abs_q15(&speed,&MyESC.SetSpeed,1);	//采集出实际转速	
	else if(MyESC.PID_Direction == 0x20)
		MyESC.SetSpeed = 0;
	
	if(MyESC.SetSpeed >= MyESC.umax) MyESC.SetSpeed = MyESC.umax;
	else if(MyESC.SetSpeed <= MyESC.umin) MyESC.SetSpeed = MyESC.umin;
	
	MyESC.err=MyESC.SetSpeed-MyESC.ActualSpeed;
	
	if(MyESC.ActualSpeed>MyESC.umax)  //若实际值大于最大值
	{
	   if(fabs(MyESC.err)>int_thres)      
		{
			index = 0;
		}
	   else
	   {
			index = 1;
			if(MyESC.err < 0) MyESC.integral += MyESC.err; //积分分离过程
	   }
	}
	else if(MyESC.ActualSpeed<MyESC.umin)
	{
		if(fabs(MyESC.err)>int_thres)      
		{
			index = 0;
		}
	   else
	   {
			index = 1;
			if(MyESC.err > 0) MyESC.integral += MyESC.err; //积分分离过程
	   }
	}
	else
	{
		if(fabs(MyESC.err)>int_thres)                    //积分分离过程
		{
			index = 0;
		}
		else
		{
			index = 1;
			MyESC.integral += MyESC.err;
		}
	}
	
	if(MyESC.PID_Direction == 0x00)
		MyESC.duty=MyESC.Kp1*MyESC.err+index*MyESC.Ki1*MyESC.integral+MyESC.Kd1*(MyESC.err-MyESC.err_last);
	else if(MyESC.PID_Direction == 0x10)
		MyESC.duty=MyESC.Kp2*MyESC.err+index*MyESC.Ki2*MyESC.integral+MyESC.Kd2*(MyESC.err-MyESC.err_last);
	
	MyESC.err_last = MyESC.err;
	
	if(MyESC.duty > 980) 		
		MyESC.duty = 980;

//	if(MyESC.duty < -980) 		
//		MyESC.duty = -980;

//	if(MyESC.duty < 0)
//		MyESC.duty = - MyESC.duty;
	
	if(MyESC.PID_Direction == 0x00)
	{
		WRITE_REG(TIM1->CCR1, MyESC.duty);
		WRITE_REG(TIM1->CCR4, 0);
	}
	else if(MyESC.PID_Direction == 0x10)
	{
		WRITE_REG(TIM1->CCR4, MyESC.duty);
		WRITE_REG(TIM1->CCR1, 0);
	}
	else if(MyESC.PID_Direction == 0x20)
	{
		WRITE_REG(TIM1->CCR1, 999);
		WRITE_REG(TIM1->CCR4, 999);
	}

	
}


