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
	
		case 1://撞击摆正
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
			
		case 3://陀螺仪归零
			if(Run_time(3,-3) == 1)
			{
				IMU_Command(ANGLE_INIT);
				LL_TIM_EnableIT_UPDATE(TIM6);		//开启角度环PID
				Angle_init(); //清除积分
				OpenMv_angle = -90.0f;
				Drive(speed,G0.A_speed);
				TDPS.P1_Stage = 4;
			}
		break;
		
		case 4://等待转向完毕
			Drive(0,G0.A_speed);
			if(Wait(15) == 1)
			{
				Drive(0,0);
				LL_mDelay(200);
				Angle_init(); //清除积分
				TDPS.P1_Stage = 5;
			}
		break;
			
		case 5://过桥 准备冲撞
			if(Run_time_PID(100,speed) == 1)
			{
				IMU_Command(ANGLE_INIT);
				OpenMv_angle = 30.0f;	
				LL_mDelay(200);
				TDPS.P1_Stage = 6;	
			}
		break;
		
		case 6://等待转向完毕
			Drive(0,G0.A_speed);
			if(Wait(20) == 1)
			{
				LL_TIM_DisableIT_UPDATE(TIM6);		//关闭角度环PID
				Angle_init(); //清除积分
				TDPS.P1_Stage = 7;
			}
		break;
			
		case 7://冲撞
			if(Run_time(45,15) == 1)
			{
				IMU_Command(ANGLE_INIT);
				Angle_init(); //清除积分
				OpenMv_angle = 0.0f;
				LL_mDelay(200);
				TDPS.P1_Stage = 8;	
			}
		break;
			
		case 8://后退
			if(Run_time(39, -5) == 1)
			{
				OpenMv_angle = 89.0f;	
				LL_mDelay(200);
				LL_TIM_EnableIT_UPDATE(TIM6);		//开启角度环PID	
				Angle_init(); //清除积分
				TDPS.P1_Stage = 9;	
			}
		break;
		
		case 9://等待转向完毕
			Drive(0,G0.A_speed);
			if(Wait(15) == 1)
			{
				TDPS.P1_Stage = 10;
				Angle_init(); //清除积分
				LL_mDelay(200);
			}
		break;
			
		case 10://到桥停下来
			
			if(Myusonic.US_mean[1] > 350)
		  {
				Drive(speed,G0.A_speed);
			}
			else
			{
				Drive(0,0);
				Angle_init(); //清除积分
				LL_mDelay(200);
				TDPS.P1_Stage = 11;
			}
		break;
			
		case 11://后退
			if(Run_time(25, 5) == 1)
			{
				LL_mDelay(200);
				Angle_init(); //清除积分
				TDPS.P1_Stage = 12;	
			}
		break;
			
		default: break;
	}
	
}

uint8_t Receive_Shape(void)
{
	LL_TIM_EnableIT_UPDATE(TIM6);		//开启角度环PID
	if(CV_flag != 1) //未接收到数据
		CV_Receive(Rx2Buffer);
	else //接收到转弯角度，开始转弯
		{
			CV_flag = 0;	//转弯结束
			return 1;
		}
	return 0;
}

void Diamond(short speed, uint8_t shape)
{
	switch (TDPS.P2_DStage)
	{
		case 1: //开启角度环PID
			IMU_Command(ANGLE_INIT);
			LL_TIM_DisableIT_UPDATE(TIM6);		//关闭角度环PID
			OpenMv_angle = 0.0f;
			TDPS.P2_DStage = 2;	
		break;
		
		case 2: //前进
			if(Run_time(47,6) == 1)
			{
				LL_mDelay(500);
				OpenMv_angle = 90.0f;
				LL_TIM_EnableIT_UPDATE(TIM6);		//开启角度环PID
				TDPS.P2_DStage = 3;	
			}
		break;
		
		case 3://等待角度校准
			Drive(0,G0.A_speed);
			if(Wait(10) == 1)
			{
				TDPS.P2_DStage = 4;
				LL_TIM_DisableIT_UPDATE(TIM6);		//关闭角度环PID
				Angle_init(); //清除积分
			}
		break;
			
		case 4://前进一点点
			if(Run_time(3,6) == 1)
			{
				LL_mDelay(500);
				LL_TIM_EnableIT_UPDATE(TIM6);		//开启角度环PID
				TDPS.P2_DStage = 5;	
			}
		break;
			
		case 5://等待角度校准
			Drive(0,G0.A_speed);
			if(Wait(10) == 1)
			{
				TDPS.P2_DStage = 6;
				LL_TIM_DisableIT_UPDATE(TIM6);		//关闭角度环PID
				Angle_init(); //清除积分
			}
		break;
		
		case 6: //接受形状数据
			Drive(0,0);
			if(Receive_Shape() == 1 && MyCV.shape != 0) 
//			if(MyCV.shape != 0) 
			{
				IMU_Command(ANGLE_INIT);
				TDPS.P2_DStage = 7;	
//				TDPS.P2_DStage = 30;	
			}
		break;
		
		case 7://转向
			OpenMv_angle = 90.0f;	
			LL_mDelay(500);
			LL_TIM_EnableIT_UPDATE(TIM6);		//开启角度环PID
			TDPS.P2_DStage = 8;	
		break;
			
		case 8://等待转向完毕
			Drive(0,G0.A_speed);
			if(Wait(10) == 1)
			{
				LL_TIM_DisableIT_UPDATE(TIM6);		//关闭角度环PID
				Angle_init(); //清除积分
				TDPS.P2_DStage = 9;
			}
		break;
			
		case 9://前进一点点
			if(Run_time(7,6) == 1)
			{
				LL_mDelay(500);
				OpenMv_angle = 0.0f;	
				LL_TIM_EnableIT_UPDATE(TIM6);		//开启角度环PID
				TDPS.P2_DStage = 10;	
			}
		break;
			
		case 10://等待角度校准
			Drive(0,G0.A_speed);
			if(Wait(10) == 1)
			{
				TDPS.P2_DStage = 11;
				LL_TIM_DisableIT_UPDATE(TIM6);		//关闭角度环PID
				Angle_init(); //清除积分
			}
		break;
			
		case 11://前往道路中央
			if(Run_time(49,6) == 1)
			{
				OpenMv_angle = 0.0f;	//角度清零
				LL_TIM_EnableIT_UPDATE(TIM6);		//开启角度环PID
				LL_mDelay(500);
				TDPS.P2_DStage = 12;	
			}	
		break;
			
		case 12://等待角度校准
			Drive(0,G0.A_speed);
			if(Wait(20) == 1)
			{
				LL_TIM_DisableIT_UPDATE(TIM6);		//关闭角度环PID
				Angle_init(); //清除积分
				TDPS.P2_DStage = 13;
				TDPS.P2_SStage = 1;
			}
		break;
			
		case 13: //前往各形状
				switch (shape)
				{
				case Square:
						
						switch (TDPS.P2_SStage)
						{
							case 1: //转向对准矩形
								OpenMv_angle = 90.0f;	
								LL_mDelay(100);
								LL_TIM_EnableIT_UPDATE(TIM6);		//开启角度环PID
								TDPS.P2_SStage = 2;	
							break;
							
							case 2: //等待转向完成
								Drive(0,G0.A_speed);
								if(Wait(20) == 1)
								{
									LL_TIM_DisableIT_UPDATE(TIM6);		//关闭角度环PID
									Angle_init(); //清除积分
									TDPS.P2_SStage = 3;
								}
							break;
							
							case 3:	//泥头车冲锋
								if(Run_time(8,15) == 1)
								{
									LL_mDelay(1000);
									LL_TIM_EnableIT_UPDATE(TIM6);	//开启角度环PID	
									TDPS.P2_SStage = 4;	
								}
							break;
								
							case 4://等待角度校准
								Drive(0,G0.A_speed);
								if(Wait(20) == 1)
								{
									TDPS.P2_SStage = 5;
									LL_TIM_DisableIT_UPDATE(TIM6);		//关闭角度环PID
									Angle_init(); //清除积分
								}
							break;
							
							case 5:	//泥头车冲锋
								if(Run_time(8,15) == 1)
								{
									LL_mDelay(1000);
									TDPS.P2_SStage = 6;	
								}
							break;
								
							case 6: //回到道路中心
								if(Run_time(10,-15) == 1)
								{
									LL_mDelay(1000);
									TDPS.P2_SStage = 7;	
								}
							break;
								
							case 7: //转向对准墙壁
								OpenMv_angle = 0;	
								LL_mDelay(100);
								LL_TIM_EnableIT_UPDATE(TIM6);	//开启角度环PID	
								TDPS.P2_SStage = 8;	
							break;
							
							case 8: //等待转向完成
								Drive(0,G0.A_speed);
								if(Wait(20) == 1)
								{
									LL_TIM_DisableIT_UPDATE(TIM6);		//关闭角度环PID
									Angle_init(); //清除积分
									TDPS.P2_SStage = 9;
								}
							break;
							
							case 9: //冲向墙壁
								if(Run_time(20,15) == 1)
								{
									LL_mDelay(200);
									LL_TIM_EnableIT_UPDATE(TIM6);	//开启角度环PID	
									TDPS.P2_SStage = 10;	
									TDPS.P2_DStage = 14;
								}
							break;
								
							default: break;
						}
						
				break; //switch square的结束
					
				case Triangle:
						
						switch (TDPS.P2_SStage)
						{
							case 1: //前进至菱形边线
								if(Run_time(36,6) == 1)
								{
									LL_mDelay(1000);
									TDPS.P2_SStage = 2;
								}	
							break;
								
							case 2: //转向对准三角形
								OpenMv_angle = -45.0f;	
								LL_mDelay(100);
								LL_TIM_EnableIT_UPDATE(TIM6);	//开启角度环PID
								TDPS.P2_SStage = 3;	
							break;
							
							case 3: //等待转向完成
								Drive(0,G0.A_speed);
								if(Wait(20) == 1)
								{
									LL_TIM_DisableIT_UPDATE(TIM6);		//关闭角度环PID
									Angle_init(); //清除积分
									TDPS.P2_SStage = 4;
								}
							break;
							
							case 4:	//泥头车冲锋
								if(Run_time(5,15) == 1)
								{
									LL_mDelay(1000);
									LL_TIM_EnableIT_UPDATE(TIM6);	//开启角度环PID	
									TDPS.P2_SStage = 5;	
								}
							break;
								
							case 5://等待角度校准
								Drive(0,G0.A_speed);
								if(Wait(20) == 1)
								{
									TDPS.P2_SStage = 6;
									LL_TIM_DisableIT_UPDATE(TIM6);		//关闭角度环PID
									Angle_init(); //清除积分
								}
							break;
							
							case 6:	//泥头车冲锋
								if(Run_time(5,15) == 1)
								{
									LL_mDelay(1000);
									TDPS.P2_SStage = 7;	
								}
							break;
							
							case 7: //回来
								if(Run_time(10,-15) == 1)
								{
									LL_mDelay(1000);
									LL_TIM_EnableIT_UPDATE(TIM6);	//开启角度环PID	
									TDPS.P2_SStage = 8;	
								}
							break;
								
							case 8: //转向对准墙壁
								OpenMv_angle = 0;	
								LL_mDelay(1000);
								LL_TIM_EnableIT_UPDATE(TIM6);	//开启角度环PID
								TDPS.P2_SStage = 9;	
							break;
							
							case 9: //等待转向完成
								Drive(0,G0.A_speed);
								if(Wait(20) == 1)
								{
									LL_TIM_DisableIT_UPDATE(TIM6);		//关闭角度环PID
									Angle_init(); //清除积分
									TDPS.P2_SStage = 10;
								}
							break;
							
							case 10: //冲向墙壁
								if(Run_time(10,15) == 1)
								{
									LL_mDelay(200);
									LL_TIM_EnableIT_UPDATE(TIM6);	//开启角度环PID	
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
							case 1: //向前走一点
								if(Run_time(4,1) == 1)
								{
									LL_mDelay(1000);
									TDPS.P2_SStage = 2;
								}	
							break;
								
							case 2: //转向对准圆圈
								OpenMv_angle = -90.0f;	
								LL_mDelay(1000);
								LL_TIM_EnableIT_UPDATE(TIM6);	//开启角度环PID	
								TDPS.P2_SStage = 3;	
							break;
							
							case 3: //等待转向完成
								Drive(0,G0.A_speed);
								if(Wait(20) == 1)
								{
									LL_TIM_DisableIT_UPDATE(TIM6);		//关闭角度环PID
									Angle_init(); //清除积分
									TDPS.P2_SStage = 4;
								}
							break;
							
							case 4:	//泥头车冲锋
								if(Run_time(8,15) == 1)
								{
									LL_mDelay(1000);
									LL_TIM_EnableIT_UPDATE(TIM6);	//开启角度环PID	
									TDPS.P2_SStage = 5;	
								}
							break;
								
							case 5://等待角度校准
								Drive(0,G0.A_speed);
								if(Wait(20) == 1)
								{
									LL_TIM_DisableIT_UPDATE(TIM6);		//关闭角度环PID
									Angle_init(); //清除积分
									TDPS.P2_SStage = 6;
								}
							break;
							
							case 6:	//泥头车冲锋
								if(Run_time(15,15) == 1)
								{
									LL_mDelay(1000);
									TDPS.P2_SStage = 7;	
								}
							break;
							
							case 7: //回到道路中心
								if(Run_time(20,-15) == 1)
								{
									LL_mDelay(1000);
									TDPS.P2_SStage = 8;	
								}
							break;
								
							case 8: //转向对准墙壁
								OpenMv_angle = 0.0f;	
								LL_mDelay(100);
								LL_TIM_EnableIT_UPDATE(TIM6);	//开启角度环PID	
								TDPS.P2_SStage = 9;	
							break;
							
							case 9: //等待转向完成
								Drive(0,G0.A_speed);
								if(Wait(20) == 1)
								{
									LL_TIM_DisableIT_UPDATE(TIM6);		//关闭角度环PID
									Angle_init(); //清除积分
									TDPS.P2_SStage = 10;
								}
							break;
							
							case 10: //冲向墙壁
								if(Run_time(20,15) == 1)
								{
									LL_mDelay(200);
									LL_TIM_EnableIT_UPDATE(TIM6);	//开启角度环PID	
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
				
		case 14: //等待角度校准
		Drive(0,G0.A_speed);
			if(Wait(20) == 1)
			{
				Angle_init(); //清除积分
				OpenMv_angle = -90.0f;	
				TDPS.P2_DStage = 15;
			}
		break;
		
		case 15: //等待角度校准
		Drive(0,G0.A_speed);
			if(Wait(20) == 1)
			{
				LL_TIM_DisableIT_UPDATE(TIM6);		//关闭角度环PID
				Angle_init(); //清除积分
				IMU_Command(ANGLE_INIT);
				OpenMv_angle = 0.0f;	
				TDPS.P2_DStage = 16;
				TDPS.P2_TStage = 1;
			}
		break;
			
		default: break;
		}
}

//完成形状识别后，开始超声波+角度环导航模式
void Trash(uint16_t d_fence, uint16_t d_turn, short speed)
{
	switch (TDPS.P2_TStage)
	{
		case 1: //前往栏杆
			LL_TIM_EnableIT_UPDATE(TIM6);	//开启角度环PID	
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
			
		case 2: //等待转向完毕
			Drive(0,G0.A_speed);
			if(Wait(20) == 1)
			{
				Angle_init(); //清除积分
				TDPS.P2_TStage = 3;
			}
		break;
			
		case 3: //前进直到右边超声波发生突变
			if(Myusonic.US_mean[1] < d_turn)
				Drive(speed,G0.A_speed);
			else
			{
				Drive(0,0);
				LL_mDelay(200);
				TDPS.P2_TStage = 4;
			}
		break;
			
		case 4: //前进一点点
			
			if(Run_time(8,6) == 1)
			{
				LL_mDelay(200);
				OpenMv_angle = 0.0f;	
				TDPS.P2_TStage = 5;	
			}
		break;	
			
		case 5: //等待转向完毕
			Drive(0,G0.A_speed);
			if(Wait(20) == 1)
			{
				Angle_init(); //清除积分
				TDPS.P2_TStage = 6;
			}
		break;
			
		case 6: //前往栏杆
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
			
		case 7: //等待转向完毕
			Drive(0,G0.A_speed);
			if(Wait(20) == 1)
			{
				Angle_init(); //清除积分
				TDPS.P2_TStage = 8;
			}
		break;
			
		case 8: //前进直到右边超声波发生突变
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
			
		case 9: //前进一点点
			
			if(Run_time(14,6) == 1)
			{
				LL_mDelay(200);
				OpenMv_angle = 0.0f;	
				TDPS.P2_TStage = 10;	
			}
		break;
			
		case 10: //等待转向完毕
			Drive(0,G0.A_speed);
			if(Wait(20) == 1)
			{
				Angle_init(); //清除积分
				LL_TIM_EnableIT_UPDATE(TIM6);	//开启角度环PID	
				TDPS.P2_TStage = 11;
			}
		break;
			
		case 11: //前往栏杆
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
			
		case 12: //等待转向完毕
			Drive(0,G0.A_speed);
			if(Wait(20) == 1)
			{
				Angle_init(); //清除积分
				TDPS.P2_TStage = 13;
			}
		break;
			
		case 13: //前进到垃圾桶
			if(Myusonic.US_mean[0] > 180)
				Drive(speed,G0.A_speed);
			else
			{
				Drive(0,0);
				IMU_Command(ANGLE_INIT);
				LL_TIM_DisableIT_UPDATE(TIM6);		//关闭角度环PID
				TDPS.P2_TStage = 14;
			}
		break;
			
		case 14: //向前推进一点
			if(Run_time(5,5) == 1)
			{
				LL_mDelay(200);
				Angle_init(); //清除积分
				TDPS.P2_TStage = 15;
			}	
		break;
			
		case 15://放球
			TIM15->CCR2 = 130;
		if(Wait(5) == 1)
			{
				TDPS.P2_TStage = 16;
			}
		break;
			
		case 16://收球 后退
			TIM15->CCR2 = 55;
		if(Run_time(5,-5) == 1)
			{
				LL_mDelay(500);
				OpenMv_angle = 90.0f;	
				LL_TIM_EnableIT_UPDATE(TIM6);		//开启角度环PID
				TDPS.P2_TStage = 17;
			}	
		break;
		
		case 17://等待转向完毕
			Drive(0,G0.A_speed);
			if(Wait(20) == 1)
			{
				Angle_init(); //清除积分
				LL_TIM_DisableIT_UPDATE(TIM6);		//关闭角度环PID
				TDPS.P2_TStage = 18;
				TDPS.P2_PStage = 1; //进入下一阶段
			}
		break;
		
		default: break;
	}
}

void Planter(short speed)
{
	switch (TDPS.P2_PStage)
	{
		case 1://前进一点点
			LL_TIM_EnableIT_UPDATE(TIM6);		//开启角度环PID
			if(Run_time(8,10) == 1)
			{
				LL_mDelay(1000);
				Angle_init(); //清除积分
				TDPS.P2_PStage = 2;
			}	
		break;
	
		case 2://等待角度校准
			Drive(0,G0.A_speed);
			if(Wait(10) == 1)
			{
				TDPS.P2_PStage = 3;
				memcpy(&pdis,&Myusonic.US_mean[1],sizeof(short));//得到一个位置
				Angle_init(); //清除积分
			}
		break;
		
		case 3: //到达第一个花坛
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
		
		case 4: //离开第一个花坛
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
		
		case 5: //到达第二个花坛
		if(Myusonic.US_mean[1] > pdis - 4000)
			Drive(speed,G0.A_speed);
		else
		{
			Drive(0,0);
			LL_mDelay(200);
			TDPS.P2_PStage = 6;
		}
		break;
			
		case 6: //发送时间
			Send_Time();
			Angle_init(); //清除积分
			LL_mDelay(500);
			TDPS.P2_PStage = 7;
		break;
		
		case 7://前进一段距离停下
			if(Myusonic.US_mean[0] > 900)
			Drive(speed,G0.A_speed);
			else
			{
				Drive(0,0);
				LL_mDelay(200);
				TDPS.P2_PStage = 8;
			}
		break;
		
		case 8://等待角度校准
			Drive(0,G0.A_speed);
			if(Wait(10) == 1)
			{
				TDPS.P2_PStage = 9;
				LL_TIM_DisableIT_UPDATE(TIM6);		//关闭角度环PID
				Angle_init(); //清除积分
			}
		break;			
		default: break;
	}
}


//void Planter(short speed)
//{
//	switch (TDPS.P2_PStage)
//	{
//		case 1://前进一点点
//			LL_TIM_EnableIT_UPDATE(TIM6);		//开启角度环PID
//			if(Run_time(8,10) == 1)
//			{
//				LL_mDelay(100);
//				Angle_init(); //清除积分
//				TDPS.P2_PStage = 2;
//			}	
//		break;
//	
//		case 2://等待角度校准
//			Drive(0,G0.A_speed);
//			if(Wait(10) == 1)
//			{
//				memcpy(&pdis,&Myusonic.US_mean[1],sizeof(short));//得到一个位置
//				IMU_Command(ANGLE_INIT);
//				Angle_init(); //清除积分
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
//		case 4: //等待转向完毕
//			Drive(0,G0.A_speed);
//			if(Wait(20) == 1)
//			{
//				Angle_init(); //清除积分
//				TDPS.P2_PStage = 5;
//			}
//		break;
//			
//		case 5: //后退直到右边超声波发生突变
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
//		case 6: //后退一点点
//			
//			if(Run_time(8,-6) == 1)
//			{
//				LL_mDelay(200);
//				OpenMv_angle = 0.0f;	
//				TDPS.P2_PStage = 7;	
//			}
//		break;	
//			
//		case 7: //等待转向完毕
//			Drive(0,G0.A_speed);
//			if(Wait(20) == 1)
//			{
//				Angle_init(); //清除积分
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
//				Angle_init(); //清除积分
//				TDPS.P2_PStage = 9;
//			}	
//		break;
//			
//		case 9://前进一段距离停下
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
//		case 10://等待角度校准
//			Drive(0,G0.A_speed);
//			if(Wait(10) == 1)
//			{
//				TDPS.P2_PStage = 11;
//				LL_TIM_DisableIT_UPDATE(TIM6);		//关闭角度环PID
//				Angle_init(); //清除积分
//			}
//		break;			
//			
//		default: break;
//	}
//}
