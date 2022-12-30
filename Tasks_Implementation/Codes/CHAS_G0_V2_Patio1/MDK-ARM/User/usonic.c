#include "usonic.h"
#include "tim.h"
#include "arm_math.h"

uint8_t US1_Edge = 0;
uint8_t US2_Edge = 0;

uint16_t us_counter;

usonic Myusonic;

void USonic1(void)
{
	if(LL_TIM_IsActiveFlag_CC1(TIM16))
	{

			if(US1_Edge == 0)          //捕获上升沿
			{
				TIM16->CNT = 0;
				
				LL_TIM_ClearFlag_CC1(TIM16);	
				LL_TIM_DisableIT_CC1(TIM16); 	//关闭中断
				LL_TIM_IC_SetPolarity(TIM16,LL_TIM_CHANNEL_CH1,LL_TIM_IC_POLARITY_FALLING);//切换捕获极性
				LL_TIM_EnableIT_CC1(TIM16);   //切换捕获极性后需重新启动
 
				US1_Edge = 1;          	 //上升沿、下降沿捕获标志位
			}
			else if(US1_Edge == 1)     //捕获下降沿
			{
				Myusonic.CNT1 = READ_REG(TIM16->CCR1);                       //获取下降沿时间点
				
				LL_TIM_ClearFlag_CC1(TIM16);
				LL_TIM_DisableIT_CC1(TIM16); 	//关闭中断
				LL_TIM_IC_SetPolarity(TIM16,LL_TIM_CHANNEL_CH1,LL_TIM_IC_POLARITY_RISING);//切换捕获极性
				LL_TIM_EnableIT_CC1(TIM16);   //切换捕获极性后需重新启动
				
				US1_Edge = 0;
			}
	}
	//单位cm
//	Myusonic.Dis1 = 0.001 * Sound_Speed * (Myusonic.CNT1 / 2);
}

void USonic2(void)
{
	if(LL_TIM_IsActiveFlag_CC1(TIM17))
	{

			if(US2_Edge == 0)          //捕获上升沿
			{
				TIM17->CNT = 0;
				
				LL_TIM_ClearFlag_CC1(TIM17);	
				LL_TIM_DisableIT_CC1(TIM17); 	//关闭中断
				LL_TIM_IC_SetPolarity(TIM17,LL_TIM_CHANNEL_CH1,LL_TIM_IC_POLARITY_FALLING);//切换捕获极性
				LL_TIM_EnableIT_CC1(TIM17);   //切换捕获极性后需重新启动
 
				US2_Edge = 1;          	 //上升沿、下降沿捕获标志位
			}
			else if(US2_Edge == 1)     //捕获下降沿
			{
				Myusonic.CNT2 = READ_REG(TIM17->CCR1);                       //获取下降沿时间点
				
				LL_TIM_ClearFlag_CC1(TIM17);
				LL_TIM_DisableIT_CC1(TIM17); 	//关闭中断
				LL_TIM_IC_SetPolarity(TIM17,LL_TIM_CHANNEL_CH1,LL_TIM_IC_POLARITY_RISING);//切换捕获极性
				LL_TIM_EnableIT_CC1(TIM17);   //切换捕获极性后需重新启动
				
				US2_Edge = 0;
			}
	}
	//单位cm
//	Myusonic.Dis2 = 0.001 * Sound_Speed * (Myusonic.CNT2 / 2);
}

void US_Sample(void)
{

			Myusonic.US1_rawdata[us_counter] = Myusonic.CNT1;
			Myusonic.US2_rawdata[us_counter] = Myusonic.CNT2;
			us_counter++;
			
			if (us_counter == US_SAMPLE_SIZE)
				{
					us_counter = 0;
				}
}

void Mean_Filter(void)
{
	arm_mean_q15(Myusonic.US1_rawdata,US_SAMPLE_SIZE,&Myusonic.US_mean[0]);
	arm_mean_q15(Myusonic.US2_rawdata,US_SAMPLE_SIZE,&Myusonic.US_mean[1]);
	
}
