
/************************************************
joyhandleX decode lib


Modification
* + nrf_init()
* + Ack_load(uint8_t* ack_data, uint8_t lenth)
*
last modified by DQS

*使用方法：
	if(htim->Instance == TIM6) { 
			instruction_refresh();
			read_keys();
			uint8_t aaa[8] = {1,};			
			Ack_load(aaa,8);    				
	 }
***********************************************/

/* include --------------------------------------*/
#include "usart.h"
#include "my_robot.h"
#include "nrf.h"
#include "motion.h"
#include <string.h>
#include "arm_math.h"
#include "elapse.h"
#include "usonic.h"
#include "task.h" 

/* private variables --------------------------*/
uint8_t nrf_cmd[32];              //初始数据
int16_t nrf_trans_cmd[7];         //转换后数据
uint8_t button[26];									//按键  

int16_t abc = 0;
bool nrf_mode = 1;//nrf工作模式
short p_speed = 4;

uint16_t ttime = 0;
short tspeed = 0;
float tangle = 0.0f;

NRF_COM MyNRF;
extern CV_Data MyCV;
extern ESC_COM G0;
extern IMU MyIMU;
extern uint8_t Rx2Buffer[CVBUFFERSIZE];
extern float OpenMv_angle;
extern PID_Para Angle;
extern uint8_t CV_flag;
extern usonic Myusonic;
extern Patio TDPS;


void NRF_IRQ(void)
{
if(LL_TIM_IsActiveFlag_UPDATE(TIM15) == SET)
	{
		LL_TIM_ClearFlag_UPDATE(TIM15);//清除中断标志
		
		instruction_refresh();
		read_keys();
//		uint8_t aaa[8] = {1,};			
//		Ack_load(aaa,8);  
		KeyToSpeed();
		KeyToMode();
	}
}

void KeyToSpeed(void)
{
		arm_abs_q15(&nrf_trans_cmd[3],&MyNRF.ls_abs,1);
		arm_abs_q15(&nrf_trans_cmd[4],&MyNRF.as_abs,1);
		
		if(MyNRF.ls_abs >= 10)
			MyNRF.nrf_lspeed = - nrf_trans_cmd[3] / 10;
		else
			MyNRF.nrf_lspeed = 0;
		
		if(MyNRF.as_abs >= 10)
			MyNRF.nrf_aspeed = - nrf_trans_cmd[4] / 10;
		else
			MyNRF.nrf_aspeed = 0;
}

void KeyToMode(void)
{
	if(button[0] != 0)  //切换为手柄模式
		{
			OpenMv_angle = 0;
			MyNRF.Mode = 0;
		}
	if(button[1] != 0)  //切换为直线行驶模式
		{
			OpenMv_angle = 0;
			MyNRF.Mode = 1;
		}
	if(button[2] != 0)  //切换为巡线模式
		{
			MyNRF.Mode = 2;
		}
	if(button[3] != 0)  //切换为过桥模式
		{
			MyNRF.Mode = 3;
		}
	if(button[4] == 0 && MyNRF.Mode != 5)  //切换为装球
		{
			TIM15->CCR2 = 55;
		}
	if(button[4] != 0 && MyNRF.Mode != 5)  //切换为放球
		{
			TIM15->CCR2 = 130;
		}
	
	if(button[5] != 0)  //切换为过桥模式
		{
			MyNRF.Mode = 5;
			TDPS.P2_DStage = 1;	
		}
		
	if(button[6] != 0)  //切换为定时前进
		{
			MyNRF.Mode = 6;
		}
	if(button[7] != 0)  //切换为旋转
		{
			MyNRF.Mode = 7;
		}
	if(button[8] != 0) //发时间
		{
			MyNRF.Mode = 8;
		}
}	

static void Line_Patrol_PID(void)
{
	LL_TIM_EnableIT_UPDATE(TIM6);		//开启角度环PID
	if(CV_flag != 1) //未接收到数据
		CV_Receive(Rx2Buffer);
	else//接收到转弯角度，开始转弯
		{
			Drive(0,G0.A_speed);
			if(Wait(4) == 1)
			{
				IMU_Command(ANGLE_INIT);
				Angle_init(); //消除积分误差
				G0.A_speed = 0;	
				
				Drive(5,0);
				LL_mDelay(100);
				Drive(0,0);
				
				CV_flag = 0;	//转弯结束
				LL_mDelay(100);
			}
		}
}

static void Line_Patrol_OL(void)
{
	LL_TIM_DisableIT_UPDATE(TIM6);		//关闭角度环PID
	if(CV_flag != 1) //未接收到数据
		CV_Receive(Rx2Buffer);
	else //接收到转弯角度，开始转弯
		{
			if(MyCV.angle_transfer > 0)
			{
				if(Turn_angle_OL(3,7) == 1)
				{
				Drive(4,0);
				LL_mDelay(150);
				CV_flag = 0;	//转弯结束
				}
			}
			else
			{
				if(Turn_angle_OL(3,-7) == 1)
				{
				Drive(4,0);
				LL_mDelay(150);
				CV_flag = 0;	//转弯结束
				}
			}
		}
}

void Car_Drive(uint8_t mode)
{
	switch(mode)
	{
		case 0:
			Drive(MyNRF.nrf_lspeed,MyNRF.nrf_aspeed);
			IMU_Command(ANGLE_INIT);
			OpenMv_angle = 0.0f;
			CV_init();
			TDPS.P1_Stage = 0;
			LL_TIM_DisableIT_UPDATE(TIM6);		//关闭角度环PID
		break;
		
		case 1:
			Drive(MyNRF.nrf_lspeed,G0.A_speed);
			OpenMv_angle = 0.0f;
			CV_init();
			LL_TIM_EnableIT_UPDATE(TIM6);		//开启角度环PID
		break;
		
		case 2:
			Line_Patrol_PID();
//			Line_Patrol_OL();
		
			Drive(p_speed,G0.A_speed);
			if(Myusonic.US_mean[0] < 200) 
			{
				MyNRF.Mode = 3;
				TDPS.P1_Stage = 1; //开始
				LL_mDelay(200);
			}
		//测试用
//		Drive(MyNRF.nrf_lspeed,MyNRF.nrf_aspeed);
		break;
		
		case 3:
			Bridge(15);
		break;
		
		case 4:
			Receive_Shape();
		break;
		
		case 5:
			Diamond(15, MyCV.shape);
			Trash(400,550,15);
			Planter(15);
		break;
		
		case 6:
			if( Run_time(ttime,tspeed) == 1) MyNRF.Mode = 0;
		break;
		
		case 7:
			LL_TIM_EnableIT_UPDATE(TIM6);		//开启角度环PID
			if( Turn_angle(ttime,tangle) == 1) MyNRF.Mode = 0;
		break;
		
		case 8:
			Send_Time();
			MyNRF.Mode = 0;
		break;
		
		
		default: break;
	}
	
}
/**********************************************************************	
*@	function: 初始化						
*@	input		: none							
*@	output	: none															
***********************************************************************/ 
void nrf_init(void)
{
	MyNRF.nrf_lspeed = 0;
	MyNRF.nrf_aspeed = 0;

	MyNRF.ls_abs = 0;
	MyNRF.as_abs = 0;
	
	static bool gpio_init_flag = 0;
	if(!gpio_init_flag){ 
		HAL_Delay(10);
		gpio_init_flag = 1;
	}	
	
		while(nRF24L01_Check());	
			
		nRF24L01_Set_Config();
}

/**********************************************************************
*@ 	name		: instruction_refresh	
*@	function: 手柄数据刷新						
*@	input		: none							
*@	output	: none															
***********************************************************************/ 
void instruction_refresh(void)
{
	static bool nrf_mode_l = 1;
	static uint8_t cnt = 0;
	cnt++;
	/*每30次检查一下nrf是否断连，恢复后重新初始化*/
	if(cnt == 30){
		cnt = 0;
		if(nRF24L01_Check())
		{
			nrf_mode = false;		
			for(int i; i<7; i++)		nrf_trans_cmd[i] = 0;             //nrf断线，cmd清零
			for(int i; i<26; i++)		button[i] = 0;
		}  
		else  {
			nrf_mode = true;
			if(!nrf_mode_l)
				nrf_init();
		} 
		nrf_mode_l = nrf_mode;        
	}
	
	/*IRQ置低，FIFO有数据*/
// 	if(HAL_GPIO_ReadPin(NRF_IRQ_GPIO_Port, NRF_IRQ_Pin) == 1)
//			return;
	
	if(nrf_mode)
	{		
		int16_t* nrf_p;
		int len = nRF24L01_RxPacket(nrf_cmd);
		if(len != 0 && nrf_cmd[13] == 0)  
		{
			int i_nrf = 0;
			nrf_p = (int16_t*)nrf_cmd;
			for(i_nrf = 0;i_nrf < 7;i_nrf++)
			{
				nrf_trans_cmd[i_nrf] = nrf_p[i_nrf];
			}
		}			
	abc = nrf_trans_cmd[0];
	}
}
/**********************************************************************
*@ 	name		: Ack_load
*@	function: 装载回传数据						
*@	input		: 数据buff，lenth（0-32）							
*@	output	: none															
***********************************************************************/ 
void Ack_load(uint8_t* ack_data, uint8_t lenth)
{	
	if(lenth>32) lenth = 32;
	memcpy(nRF24L01_ack_pay.Ack_Buf, ack_data,lenth);
	nRF24L01_ack_pay.Ack_Len = lenth;       	
	
	#if (DYNPD_ACK_DATA)//回送反向的ACK数据
		 nRF24L01_Rx_AckPayload(nRF24L01_ack_pay);
	#endif
}

/**********************************************************************
*@ 	name		: read_rocker	
*@	functio	: 获得摇杆数据		编号0~3				
*@	input		: none							
*@	output	: none															
***********************************************************************/ 
int16_t read_rocker(int id){
	if(id==1 || id==0) return -nrf_trans_cmd[2 + id];
	else return nrf_trans_cmd[2 + id];			
}

/**********************************************************************
*@ 	name		: read_keys	
*@	functio	: 获得按键数据，放在 boll button[26]中						
*@	input		: none							
*@	output	: none		

		 20        |		  21    
 16	 17	 06    | 04  08  09  
 18	 19	 07    | 05  10  11    
 |22| |23|     |     12  13     
   00 |24|     |          |25|  
02    03       |     15  14      
   01          |                
***********************************************************************/ 
void read_keys(void)
{
		button[0] = (nrf_trans_cmd[0]&0x0001);
		button[1] = (nrf_trans_cmd[0]&0x0002)>>1;
		button[2] = (nrf_trans_cmd[0]&0x0004)>>2;
		button[3] = (nrf_trans_cmd[0]&0x0008)>>3;	
		button[4] = (nrf_trans_cmd[0]&0x0010)>>4;
		button[5] = (nrf_trans_cmd[0]&0x0020)>>5;
		button[6] = (nrf_trans_cmd[0]&0x0040)>>6;
		button[7] = (nrf_trans_cmd[0]&0x0080)>>7;
		button[8] = (nrf_trans_cmd[1]&0x0001);
		button[9] = (nrf_trans_cmd[1]&0x0002)>>1;
		button[10] = (nrf_trans_cmd[1]&0x0004)>>2;
		button[11] = (nrf_trans_cmd[1]&0x0008)>>3;
		button[12] = (nrf_trans_cmd[1]&0x0010)>>4;
		button[13] = (nrf_trans_cmd[1]&0x0020)>>5;
		button[14] = (nrf_trans_cmd[1]&0x0040)>>6;
		button[15] = (nrf_trans_cmd[1]&0x0080)>>7;	
		button[16] = (nrf_trans_cmd[0]&0x0400)>>10;
		button[17] = (nrf_trans_cmd[0]&0x0800)>>11;
		button[18] = (nrf_trans_cmd[0]&0x1000)>>12;
		button[19] = (nrf_trans_cmd[0]&0x2000)>>13;
		button[20] = (nrf_trans_cmd[0]&0x4000)>>14;
		button[21] = (nrf_trans_cmd[0]&0x8000)>>15;
		button[22] = (nrf_trans_cmd[6]&0x0008)>>3;
		button[23] = (nrf_trans_cmd[6]&0x0004)>>2;
		button[24] = (nrf_trans_cmd[6]&0x0002)>>1;
		button[25] = (nrf_trans_cmd[6]&0x0001);	
		
}






