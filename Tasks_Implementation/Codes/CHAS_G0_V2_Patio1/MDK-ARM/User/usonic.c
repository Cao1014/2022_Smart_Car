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

			if(US1_Edge == 0)          //����������
			{
				TIM16->CNT = 0;
				
				LL_TIM_ClearFlag_CC1(TIM16);	
				LL_TIM_DisableIT_CC1(TIM16); 	//�ر��ж�
				LL_TIM_IC_SetPolarity(TIM16,LL_TIM_CHANNEL_CH1,LL_TIM_IC_POLARITY_FALLING);//�л�������
				LL_TIM_EnableIT_CC1(TIM16);   //�л������Ժ�����������
 
				US1_Edge = 1;          	 //�����ء��½��ز����־λ
			}
			else if(US1_Edge == 1)     //�����½���
			{
				Myusonic.CNT1 = READ_REG(TIM16->CCR1);                       //��ȡ�½���ʱ���
				
				LL_TIM_ClearFlag_CC1(TIM16);
				LL_TIM_DisableIT_CC1(TIM16); 	//�ر��ж�
				LL_TIM_IC_SetPolarity(TIM16,LL_TIM_CHANNEL_CH1,LL_TIM_IC_POLARITY_RISING);//�л�������
				LL_TIM_EnableIT_CC1(TIM16);   //�л������Ժ�����������
				
				US1_Edge = 0;
			}
	}
	//��λcm
//	Myusonic.Dis1 = 0.001 * Sound_Speed * (Myusonic.CNT1 / 2);
}

void USonic2(void)
{
	if(LL_TIM_IsActiveFlag_CC1(TIM17))
	{

			if(US2_Edge == 0)          //����������
			{
				TIM17->CNT = 0;
				
				LL_TIM_ClearFlag_CC1(TIM17);	
				LL_TIM_DisableIT_CC1(TIM17); 	//�ر��ж�
				LL_TIM_IC_SetPolarity(TIM17,LL_TIM_CHANNEL_CH1,LL_TIM_IC_POLARITY_FALLING);//�л�������
				LL_TIM_EnableIT_CC1(TIM17);   //�л������Ժ�����������
 
				US2_Edge = 1;          	 //�����ء��½��ز����־λ
			}
			else if(US2_Edge == 1)     //�����½���
			{
				Myusonic.CNT2 = READ_REG(TIM17->CCR1);                       //��ȡ�½���ʱ���
				
				LL_TIM_ClearFlag_CC1(TIM17);
				LL_TIM_DisableIT_CC1(TIM17); 	//�ر��ж�
				LL_TIM_IC_SetPolarity(TIM17,LL_TIM_CHANNEL_CH1,LL_TIM_IC_POLARITY_RISING);//�л�������
				LL_TIM_EnableIT_CC1(TIM17);   //�л������Ժ�����������
				
				US2_Edge = 0;
			}
	}
	//��λcm
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
