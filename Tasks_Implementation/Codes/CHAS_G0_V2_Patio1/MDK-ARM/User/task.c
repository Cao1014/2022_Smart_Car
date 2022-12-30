#include "task.h"
#include "usart.h"
#include "elapse.h"
#include "math.h"
#include "motion.h"
#include "usonic.h"
#include "string.h"

Patio TDPS;

extern ESC_COM G0;
extern IMU MyIMU;
extern float OpenMv_angle;
extern usonic Myusonic;
extern uint8_t CV_flag;
extern CV_Data MyCV;
extern uint8_t Rx2Buffer[CVBUFFERSIZE];
uint16_t pdis = 0;

void Bridge(short speed)
{
	switch (TDPS.P1_Stage)
	{
	
		case 1://ײ������
			if(Myusonic.US_mean[0] > 120)
			{
				Drive(5,0);
			}
			else 
			{
				Drive(8,0);
				LL_mDelay(1200);
				Drive(0,0);
				TDPS.P1_Stage = 2;
				LL_mDelay(200);
			}
		break;
			
		case 2:
			if(Run_time(10,6) == 1)
			{
				Drive(0,0);
				LL_mDelay(200);
				TDPS.P1_Stage = 3;
			}
		break;
			
		case 3://�����ǹ���
			if(Run_time(3,-3) == 1)
			{
				IMU_Command(ANGLE_INIT);
				LL_TIM_EnableIT_UPDATE(TIM6);		//�����ǶȻ�PID
				Angle_init(); //�������
				OpenMv_angle = -90.0f;
				Drive(speed,G0.A_speed);
				TDPS.P1_Stage = 4;
			}
		break;
		
		case 4://�ȴ�ת�����
			Drive(0,G0.A_speed);
			if(Wait(15) == 1)
			{
				Drive(0,0);
				LL_mDelay(200);
				Angle_init(); //�������
				TDPS.P1_Stage = 5;
			}
		break;
			
		case 5://���� ׼����ײ
			if(Run_time_PID(100,speed) == 1)
			{
				IMU_Command(ANGLE_INIT);
				OpenMv_angle = 30.0f;	
				LL_mDelay(200);
				TDPS.P1_Stage = 6;	
			}
		break;
		
		case 6://�ȴ�ת�����
			Drive(0,G0.A_speed);
			if(Wait(20) == 1)
			{
				LL_TIM_DisableIT_UPDATE(TIM6);		//�رսǶȻ�PID
				Angle_init(); //�������
				TDPS.P1_Stage = 7;
			}
		break;
			
		case 7://��ײ
			if(Run_time(45,15) == 1)
			{
				IMU_Command(ANGLE_INIT);
				Angle_init(); //�������
				OpenMv_angle = 0.0f;
				LL_mDelay(200);
				TDPS.P1_Stage = 8;	
			}
		break;
			
		case 8://����
			if(Run_time(39, -5) == 1)
			{
				OpenMv_angle = 89.0f;	
				LL_mDelay(200);
				LL_TIM_EnableIT_UPDATE(TIM6);		//�����ǶȻ�PID	
				Angle_init(); //�������
				TDPS.P1_Stage = 9;	
			}
		break;
		
		case 9://�ȴ�ת�����
			Drive(0,G0.A_speed);
			if(Wait(15) == 1)
			{
				TDPS.P1_Stage = 10;
				Angle_init(); //�������
				LL_mDelay(200);
			}
		break;
			
		case 10://����ͣ����
			
			if(Myusonic.US_mean[1] > 350)
		  {
				Drive(speed,G0.A_speed);
			}
			else
			{
				Drive(0,0);
				Angle_init(); //�������
				LL_mDelay(200);
				TDPS.P1_Stage = 11;
			}
		break;
			
		case 11://����
			if(Run_time(25, 5) == 1)
			{
				LL_mDelay(200);
				Angle_init(); //�������
				TDPS.P1_Stage = 12;	
			}
		break;
			
		default: break;
	}
	
}

uint8_t Receive_Shape(void)
{
	LL_TIM_EnableIT_UPDATE(TIM6);		//�����ǶȻ�PID
	if(CV_flag != 1) //δ���յ�����
		CV_Receive(Rx2Buffer);
	else //���յ�ת��Ƕȣ���ʼת��
		{
			CV_flag = 0;	//ת�����
			return 1;
		}
	return 0;
}

void Diamond(short speed, uint8_t shape)
{
	switch (TDPS.P2_DStage)
	{
		case 1: //�����ǶȻ�PID
			IMU_Command(ANGLE_INIT);
			LL_TIM_DisableIT_UPDATE(TIM6);		//�رսǶȻ�PID
			OpenMv_angle = 0.0f;
			TDPS.P2_DStage = 2;	
		break;
		
		case 2: //ǰ��
			if(Run_time(47,6) == 1)
			{
				LL_mDelay(500);
				OpenMv_angle = 90.0f;
				LL_TIM_EnableIT_UPDATE(TIM6);		//�����ǶȻ�PID
				TDPS.P2_DStage = 3;	
			}
		break;
		
		case 3://�ȴ��Ƕ�У׼
			Drive(0,G0.A_speed);
			if(Wait(10) == 1)
			{
				TDPS.P2_DStage = 4;
				LL_TIM_DisableIT_UPDATE(TIM6);		//�رսǶȻ�PID
				Angle_init(); //�������
			}
		break;
			
		case 4://ǰ��һ���
			if(Run_time(3,6) == 1)
			{
				LL_mDelay(500);
				LL_TIM_EnableIT_UPDATE(TIM6);		//�����ǶȻ�PID
				TDPS.P2_DStage = 5;	
			}
		break;
			
		case 5://�ȴ��Ƕ�У׼
			Drive(0,G0.A_speed);
			if(Wait(10) == 1)
			{
				TDPS.P2_DStage = 6;
				LL_TIM_DisableIT_UPDATE(TIM6);		//�رսǶȻ�PID
				Angle_init(); //�������
			}
		break;
		
		case 6: //������״����
			Drive(0,0);
			if(Receive_Shape() == 1 && MyCV.shape != 0) 
//			if(MyCV.shape != 0) 
			{
				IMU_Command(ANGLE_INIT);
				TDPS.P2_DStage = 7;	
//				TDPS.P2_DStage = 30;	
			}
		break;
		
		case 7://ת��
			OpenMv_angle = 90.0f;	
			LL_mDelay(500);
			LL_TIM_EnableIT_UPDATE(TIM6);		//�����ǶȻ�PID
			TDPS.P2_DStage = 8;	
		break;
			
		case 8://�ȴ�ת�����
			Drive(0,G0.A_speed);
			if(Wait(10) == 1)
			{
				LL_TIM_DisableIT_UPDATE(TIM6);		//�رսǶȻ�PID
				Angle_init(); //�������
				TDPS.P2_DStage = 9;
			}
		break;
			
		case 9://ǰ��һ���
			if(Run_time(7,6) == 1)
			{
				LL_mDelay(500);
				OpenMv_angle = 0.0f;	
				LL_TIM_EnableIT_UPDATE(TIM6);		//�����ǶȻ�PID
				TDPS.P2_DStage = 10;	
			}
		break;
			
		case 10://�ȴ��Ƕ�У׼
			Drive(0,G0.A_speed);
			if(Wait(10) == 1)
			{
				TDPS.P2_DStage = 11;
				LL_TIM_DisableIT_UPDATE(TIM6);		//�رսǶȻ�PID
				Angle_init(); //�������
			}
		break;
			
		case 11://ǰ����·����
			if(Run_time(49,6) == 1)
			{
				OpenMv_angle = 0.0f;	//�Ƕ�����
				LL_TIM_EnableIT_UPDATE(TIM6);		//�����ǶȻ�PID
				LL_mDelay(500);
				TDPS.P2_DStage = 12;	
			}	
		break;
			
		case 12://�ȴ��Ƕ�У׼
			Drive(0,G0.A_speed);
			if(Wait(20) == 1)
			{
				LL_TIM_DisableIT_UPDATE(TIM6);		//�رսǶȻ�PID
				Angle_init(); //�������
				TDPS.P2_DStage = 13;
				TDPS.P2_SStage = 1;
			}
		break;
			
		case 13: //ǰ������״
				switch (shape)
				{
				case Square:
						
						switch (TDPS.P2_SStage)
						{
							case 1: //ת���׼����
								OpenMv_angle = 90.0f;	
								LL_mDelay(100);
								LL_TIM_EnableIT_UPDATE(TIM6);		//�����ǶȻ�PID
								TDPS.P2_SStage = 2;	
							break;
							
							case 2: //�ȴ�ת�����
								Drive(0,G0.A_speed);
								if(Wait(20) == 1)
								{
									LL_TIM_DisableIT_UPDATE(TIM6);		//�رսǶȻ�PID
									Angle_init(); //�������
									TDPS.P2_SStage = 3;
								}
							break;
							
							case 3:	//��ͷ�����
								if(Run_time(8,15) == 1)
								{
									LL_mDelay(1000);
									LL_TIM_EnableIT_UPDATE(TIM6);	//�����ǶȻ�PID	
									TDPS.P2_SStage = 4;	
								}
							break;
								
							case 4://�ȴ��Ƕ�У׼
								Drive(0,G0.A_speed);
								if(Wait(20) == 1)
								{
									TDPS.P2_SStage = 5;
									LL_TIM_DisableIT_UPDATE(TIM6);		//�رսǶȻ�PID
									Angle_init(); //�������
								}
							break;
							
							case 5:	//��ͷ�����
								if(Run_time(8,15) == 1)
								{
									LL_mDelay(1000);
									TDPS.P2_SStage = 6;	
								}
							break;
								
							case 6: //�ص���·����
								if(Run_time(10,-15) == 1)
								{
									LL_mDelay(1000);
									TDPS.P2_SStage = 7;	
								}
							break;
								
							case 7: //ת���׼ǽ��
								OpenMv_angle = 0;	
								LL_mDelay(100);
								LL_TIM_EnableIT_UPDATE(TIM6);	//�����ǶȻ�PID	
								TDPS.P2_SStage = 8;	
							break;
							
							case 8: //�ȴ�ת�����
								Drive(0,G0.A_speed);
								if(Wait(20) == 1)
								{
									LL_TIM_DisableIT_UPDATE(TIM6);		//�رսǶȻ�PID
									Angle_init(); //�������
									TDPS.P2_SStage = 9;
								}
							break;
							
							case 9: //����ǽ��
								if(Run_time(20,15) == 1)
								{
									LL_mDelay(200);
									LL_TIM_EnableIT_UPDATE(TIM6);	//�����ǶȻ�PID	
									TDPS.P2_SStage = 10;	
									TDPS.P2_DStage = 14;
								}
							break;
								
							default: break;
						}
						
				break; //switch square�Ľ���
					
				case Triangle:
						
						switch (TDPS.P2_SStage)
						{
							case 1: //ǰ�������α���
								if(Run_time(36,6) == 1)
								{
									LL_mDelay(1000);
									TDPS.P2_SStage = 2;
								}	
							break;
								
							case 2: //ת���׼������
								OpenMv_angle = -45.0f;	
								LL_mDelay(100);
								LL_TIM_EnableIT_UPDATE(TIM6);	//�����ǶȻ�PID
								TDPS.P2_SStage = 3;	
							break;
							
							case 3: //�ȴ�ת�����
								Drive(0,G0.A_speed);
								if(Wait(20) == 1)
								{
									LL_TIM_DisableIT_UPDATE(TIM6);		//�رսǶȻ�PID
									Angle_init(); //�������
									TDPS.P2_SStage = 4;
								}
							break;
							
							case 4:	//��ͷ�����
								if(Run_time(5,15) == 1)
								{
									LL_mDelay(1000);
									LL_TIM_EnableIT_UPDATE(TIM6);	//�����ǶȻ�PID	
									TDPS.P2_SStage = 5;	
								}
							break;
								
							case 5://�ȴ��Ƕ�У׼
								Drive(0,G0.A_speed);
								if(Wait(20) == 1)
								{
									TDPS.P2_SStage = 6;
									LL_TIM_DisableIT_UPDATE(TIM6);		//�رսǶȻ�PID
									Angle_init(); //�������
								}
							break;
							
							case 6:	//��ͷ�����
								if(Run_time(5,15) == 1)
								{
									LL_mDelay(1000);
									TDPS.P2_SStage = 7;	
								}
							break;
							
							case 7: //����
								if(Run_time(10,-15) == 1)
								{
									LL_mDelay(1000);
									LL_TIM_EnableIT_UPDATE(TIM6);	//�����ǶȻ�PID	
									TDPS.P2_SStage = 8;	
								}
							break;
								
							case 8: //ת���׼ǽ��
								OpenMv_angle = 0;	
								LL_mDelay(1000);
								LL_TIM_EnableIT_UPDATE(TIM6);	//�����ǶȻ�PID
								TDPS.P2_SStage = 9;	
							break;
							
							case 9: //�ȴ�ת�����
								Drive(0,G0.A_speed);
								if(Wait(20) == 1)
								{
									LL_TIM_DisableIT_UPDATE(TIM6);		//�رսǶȻ�PID
									Angle_init(); //�������
									TDPS.P2_SStage = 10;
								}
							break;
							
							case 10: //����ǽ��
								if(Run_time(10,15) == 1)
								{
									LL_mDelay(200);
									LL_TIM_EnableIT_UPDATE(TIM6);	//�����ǶȻ�PID	
									TDPS.P2_SStage = 11;	
									TDPS.P2_DStage = 14;
								}
							break;
								
							default: break;
						}								
					break;
						
					case Circle:
						switch (TDPS.P2_SStage)
						{
							case 1: //��ǰ��һ��
								if(Run_time(4,1) == 1)
								{
									LL_mDelay(1000);
									TDPS.P2_SStage = 2;
								}	
							break;
								
							case 2: //ת���׼ԲȦ
								OpenMv_angle = -90.0f;	
								LL_mDelay(1000);
								LL_TIM_EnableIT_UPDATE(TIM6);	//�����ǶȻ�PID	
								TDPS.P2_SStage = 3;	
							break;
							
							case 3: //�ȴ�ת�����
								Drive(0,G0.A_speed);
								if(Wait(20) == 1)
								{
									LL_TIM_DisableIT_UPDATE(TIM6);		//�رսǶȻ�PID
									Angle_init(); //�������
									TDPS.P2_SStage = 4;
								}
							break;
							
							case 4:	//��ͷ�����
								if(Run_time(8,15) == 1)
								{
									LL_mDelay(1000);
									LL_TIM_EnableIT_UPDATE(TIM6);	//�����ǶȻ�PID	
									TDPS.P2_SStage = 5;	
								}
							break;
								
							case 5://�ȴ��Ƕ�У׼
								Drive(0,G0.A_speed);
								if(Wait(20) == 1)
								{
									LL_TIM_DisableIT_UPDATE(TIM6);		//�رսǶȻ�PID
									Angle_init(); //�������
									TDPS.P2_SStage = 6;
								}
							break;
							
							case 6:	//��ͷ�����
								if(Run_time(15,15) == 1)
								{
									LL_mDelay(1000);
									TDPS.P2_SStage = 7;	
								}
							break;
							
							case 7: //�ص���·����
								if(Run_time(20,-15) == 1)
								{
									LL_mDelay(1000);
									TDPS.P2_SStage = 8;	
								}
							break;
								
							case 8: //ת���׼ǽ��
								OpenMv_angle = 0.0f;	
								LL_mDelay(100);
								LL_TIM_EnableIT_UPDATE(TIM6);	//�����ǶȻ�PID	
								TDPS.P2_SStage = 9;	
							break;
							
							case 9: //�ȴ�ת�����
								Drive(0,G0.A_speed);
								if(Wait(20) == 1)
								{
									LL_TIM_DisableIT_UPDATE(TIM6);		//�رսǶȻ�PID
									Angle_init(); //�������
									TDPS.P2_SStage = 10;
								}
							break;
							
							case 10: //����ǽ��
								if(Run_time(20,15) == 1)
								{
									LL_mDelay(200);
									LL_TIM_EnableIT_UPDATE(TIM6);	//�����ǶȻ�PID	
									TDPS.P2_SStage = 11;	
									TDPS.P2_DStage = 14;
								}	
							break;
								
							default: break;
						}	
					break;
					
					default: break;
				}	
		break;
				
		case 14: //�ȴ��Ƕ�У׼
		Drive(0,G0.A_speed);
			if(Wait(20) == 1)
			{
				Angle_init(); //�������
				OpenMv_angle = -90.0f;	
				TDPS.P2_DStage = 15;
			}
		break;
		
		case 15: //�ȴ��Ƕ�У׼
		Drive(0,G0.A_speed);
			if(Wait(20) == 1)
			{
				LL_TIM_DisableIT_UPDATE(TIM6);		//�رսǶȻ�PID
				Angle_init(); //�������
				IMU_Command(ANGLE_INIT);
				OpenMv_angle = 0.0f;	
				TDPS.P2_DStage = 16;
				TDPS.P2_TStage = 1;
			}
		break;
			
		default: break;
		}
}

//�����״ʶ��󣬿�ʼ������+�ǶȻ�����ģʽ
void Trash(uint16_t d_fence, uint16_t d_turn, short speed)
{
	switch (TDPS.P2_TStage)
	{
		case 1: //ǰ������
			LL_TIM_EnableIT_UPDATE(TIM6);	//�����ǶȻ�PID	
			if(Myusonic.US_mean[0] > d_fence)
				Drive(speed,G0.A_speed);
			else
			{
				Drive(0,0);
				OpenMv_angle = 90.0f;	
				LL_mDelay(200);
				TDPS.P2_TStage = 2;
			}
		break;
			
		case 2: //�ȴ�ת�����
			Drive(0,G0.A_speed);
			if(Wait(20) == 1)
			{
				Angle_init(); //�������
				TDPS.P2_TStage = 3;
			}
		break;
			
		case 3: //ǰ��ֱ���ұ߳���������ͻ��
			if(Myusonic.US_mean[1] < d_turn)
				Drive(speed,G0.A_speed);
			else
			{
				Drive(0,0);
				LL_mDelay(200);
				TDPS.P2_TStage = 4;
			}
		break;
			
		case 4: //ǰ��һ���
			
			if(Run_time(8,6) == 1)
			{
				LL_mDelay(200);
				OpenMv_angle = 0.0f;	
				TDPS.P2_TStage = 5;	
			}
		break;	
			
		case 5: //�ȴ�ת�����
			Drive(0,G0.A_speed);
			if(Wait(20) == 1)
			{
				Angle_init(); //�������
				TDPS.P2_TStage = 6;
			}
		break;
			
		case 6: //ǰ������
			if(Myusonic.US_mean[0] > d_fence)
				Drive(speed,G0.A_speed);
			else
			{
				Drive(0,0);
				OpenMv_angle = 90.0f;	
				LL_mDelay(200);
				TDPS.P2_TStage = 7;
			}
		break;
			
		case 7: //�ȴ�ת�����
			Drive(0,G0.A_speed);
			if(Wait(20) == 1)
			{
				Angle_init(); //�������
				TDPS.P2_TStage = 8;
			}
		break;
			
		case 8: //ǰ��ֱ���ұ߳���������ͻ��
			if(Myusonic.US_mean[1] < d_turn)
				Drive(speed,G0.A_speed);
			else
			{
				Drive(0,0);
				OpenMv_angle = 0.0f;	
				LL_mDelay(200);
				TDPS.P2_TStage = 9;
			}
		break;
			
		case 9: //ǰ��һ���
			
			if(Run_time(14,6) == 1)
			{
				LL_mDelay(200);
				OpenMv_angle = 0.0f;	
				TDPS.P2_TStage = 10;	
			}
		break;
			
		case 10: //�ȴ�ת�����
			Drive(0,G0.A_speed);
			if(Wait(20) == 1)
			{
				Angle_init(); //�������
				LL_TIM_EnableIT_UPDATE(TIM6);	//�����ǶȻ�PID	
				TDPS.P2_TStage = 11;
			}
		break;
			
		case 11: //ǰ������
			if(Myusonic.US_mean[0] > d_fence-120)
				Drive(speed,G0.A_speed);
			else
			{
				Drive(6,0);
				LL_mDelay(650);
				OpenMv_angle = 90.0f;	
				Drive(0,0);
				LL_mDelay(200);
				TDPS.P2_TStage = 12;
			}
		break;
			
		case 12: //�ȴ�ת�����
			Drive(0,G0.A_speed);
			if(Wait(20) == 1)
			{
				Angle_init(); //�������
				TDPS.P2_TStage = 13;
			}
		break;
			
		case 13: //ǰ��������Ͱ
			if(Myusonic.US_mean[0] > 180)
				Drive(speed,G0.A_speed);
			else
			{
				Drive(0,0);
				IMU_Command(ANGLE_INIT);
				LL_TIM_DisableIT_UPDATE(TIM6);		//�رսǶȻ�PID
				TDPS.P2_TStage = 14;
			}
		break;
			
		case 14: //��ǰ�ƽ�һ��
			if(Run_time(5,5) == 1)
			{
				LL_mDelay(200);
				Angle_init(); //�������
				TDPS.P2_TStage = 15;
			}	
		break;
			
		case 15://����
			TIM15->CCR2 = 130;
		if(Wait(5) == 1)
			{
				TDPS.P2_TStage = 16;
			}
		break;
			
		case 16://���� ����
			TIM15->CCR2 = 55;
		if(Run_time(5,-5) == 1)
			{
				LL_mDelay(500);
				OpenMv_angle = 90.0f;	
				LL_TIM_EnableIT_UPDATE(TIM6);		//�����ǶȻ�PID
				TDPS.P2_TStage = 17;
			}	
		break;
		
		case 17://�ȴ�ת�����
			Drive(0,G0.A_speed);
			if(Wait(20) == 1)
			{
				Angle_init(); //�������
				LL_TIM_DisableIT_UPDATE(TIM6);		//�رսǶȻ�PID
				TDPS.P2_TStage = 18;
				TDPS.P2_PStage = 1; //������һ�׶�
			}
		break;
		
		default: break;
	}
}

void Planter(short speed)
{
	switch (TDPS.P2_PStage)
	{
		case 1://ǰ��һ���
			LL_TIM_EnableIT_UPDATE(TIM6);		//�����ǶȻ�PID
			if(Run_time(8,10) == 1)
			{
				LL_mDelay(1000);
				Angle_init(); //�������
				TDPS.P2_PStage = 2;
			}	
		break;
	
		case 2://�ȴ��Ƕ�У׼
			Drive(0,G0.A_speed);
			if(Wait(10) == 1)
			{
				TDPS.P2_PStage = 3;
				memcpy(&pdis,&Myusonic.US_mean[1],sizeof(short));//�õ�һ��λ��
				Angle_init(); //�������
			}
		break;
		
		case 3: //�����һ����̳
			if(Myusonic.US_mean[1] > pdis - 4000)
				Drive(speed,G0.A_speed);
			else
			{
				Drive(10,0);
				LL_mDelay(2500);
				Drive(0,0);
				LL_mDelay(200);
				TDPS.P2_PStage = 4;
			}
		break;
		
		case 4: //�뿪��һ����̳
		if(Myusonic.US_mean[1] < pdis - 4000)
			Drive(speed,G0.A_speed);
		else
		{
			Drive(8,0);
			LL_mDelay(5000);
			Drive(0,0);
			LL_mDelay(200);
			TDPS.P2_PStage = 5;
		}
		break;
		
		case 5: //����ڶ�����̳
		if(Myusonic.US_mean[1] > pdis - 4000)
			Drive(speed,G0.A_speed);
		else
		{
			Drive(0,0);
			LL_mDelay(200);
			TDPS.P2_PStage = 6;
		}
		break;
			
		case 6: //����ʱ��
			Send_Time();
			Angle_init(); //�������
			LL_mDelay(500);
			TDPS.P2_PStage = 7;
		break;
		
		case 7://ǰ��һ�ξ���ͣ��
			if(Myusonic.US_mean[0] > 900)
			Drive(speed,G0.A_speed);
			else
			{
				Drive(0,0);
				LL_mDelay(200);
				TDPS.P2_PStage = 8;
			}
		break;
		
		case 8://�ȴ��Ƕ�У׼
			Drive(0,G0.A_speed);
			if(Wait(10) == 1)
			{
				TDPS.P2_PStage = 9;
				LL_TIM_DisableIT_UPDATE(TIM6);		//�رսǶȻ�PID
				Angle_init(); //�������
			}
		break;			
		default: break;
	}
}


//void Planter(short speed)
//{
//	switch (TDPS.P2_PStage)
//	{
//		case 1://ǰ��һ���
//			LL_TIM_EnableIT_UPDATE(TIM6);		//�����ǶȻ�PID
//			if(Run_time(8,10) == 1)
//			{
//				LL_mDelay(100);
//				Angle_init(); //�������
//				TDPS.P2_PStage = 2;
//			}	
//		break;
//	
//		case 2://�ȴ��Ƕ�У׼
//			Drive(0,G0.A_speed);
//			if(Wait(10) == 1)
//			{
//				memcpy(&pdis,&Myusonic.US_mean[1],sizeof(short));//�õ�һ��λ��
//				IMU_Command(ANGLE_INIT);
//				Angle_init(); //�������
//				TDPS.P2_PStage = 3;
//			}
//		break;
//		
//		case 3:
//		if(Myusonic.US_mean[0] > 400)
//				Drive(speed,G0.A_speed);
//			else
//			{
//				OpenMv_angle = 90.0f;	
//				Drive(0,0);
//				LL_mDelay(200);
//				TDPS.P2_PStage = 4;
//			}
//		break;
//			
//		case 4: //�ȴ�ת�����
//			Drive(0,G0.A_speed);
//			if(Wait(20) == 1)
//			{
//				Angle_init(); //�������
//				TDPS.P2_PStage = 5;
//			}
//		break;
//			
//		case 5: //����ֱ���ұ߳���������ͻ��
//			if(Myusonic.US_mean[1] < 550)
//				Drive(-speed,G0.A_speed);
//			else
//			{
//				Drive(0,0);
//				LL_mDelay(200);
//				TDPS.P2_PStage = 6;
//			}
//		break;
//			
//		case 6: //����һ���
//			
//			if(Run_time(8,-6) == 1)
//			{
//				LL_mDelay(200);
//				OpenMv_angle = 0.0f;	
//				TDPS.P2_PStage = 7;	
//			}
//		break;	
//			
//		case 7: //�ȴ�ת�����
//			Drive(0,G0.A_speed);
//			if(Wait(20) == 1)
//			{
//				Angle_init(); //�������
//				TDPS.P2_PStage = 8;
//			}
//		break;
//			
//		case 8:
//			
//		if(Run_time(10,10) == 1)
//			{
//				LL_mDelay(200);
//				Send_Time();
//				Angle_init(); //�������
//				TDPS.P2_PStage = 9;
//			}	
//		break;
//			
//		case 9://ǰ��һ�ξ���ͣ��
//			if(Myusonic.US_mean[0] > 900)
//			Drive(speed,G0.A_speed);
//			else
//			{
//				Drive(0,0);
//				LL_mDelay(200);
//				TDPS.P2_PStage = 10;
//			}
//		break;
//		
//		case 10://�ȴ��Ƕ�У׼
//			Drive(0,G0.A_speed);
//			if(Wait(10) == 1)
//			{
//				TDPS.P2_PStage = 11;
//				LL_TIM_DisableIT_UPDATE(TIM6);		//�رսǶȻ�PID
//				Angle_init(); //�������
//			}
//		break;			
//			
//		default: break;
//	}
//}
