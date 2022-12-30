#include "elapse.h"
#include "usart.h"
#include "rtc.h"
#include "motion.h"

RTC_DateTypeDef GetDate;  	//获取日期结构体
RTC_TimeTypeDef GetTime;   	//获取时间结构体
	
uint16_t clocker = 0;
uint8_t c_flag = 0;
uint8_t t_flag = 0;

extern uint8_t Rx3Buffer[IMUBUFFERSIZE];

extern ESC_COM G0;
extern IMU MyIMU;
extern float OpenMv_angle;
extern PID_Para Angle;
extern CV_Data MyCV;

void Clocker_RST(void)
{
	clocker = 0;
}

void Get_Time(void)
{	 
	HAL_RTC_GetDate(&hrtc, &GetDate, RTC_FORMAT_BIN);
	HAL_RTC_GetTime(&hrtc, &GetTime, RTC_FORMAT_BIN);

}

void Send_Time(void)
{	 
	Get_Time();
		
//		if (send == 1)
//		{
			USART1->TDR = 0x00;
			printf("\r\n");
			printf("Avada Kedavra \r\n");
			
			printf("Lin Jingxian ");
			printf("2019190602023 \r\n");
			
			printf("Xiong Juncheng ");
			printf("2019190602028 \r\n");
			
			printf("Lu Jitong ");
			printf("2019190602024 \r\n");
			
			printf("Li Jingning ");
			printf("2019190602022 \r\n");
			
			printf("Kuang Zhiyu ");
			printf("2019190601001 \r\n");
			
			printf("Jiang Boyue ");
			printf("2019190602019 \r\n");
			
			printf("Chen Zipei ");
			printf("2019190505009 \r\n");
			
			printf("Liu Sihui ");
			printf("2019190505003 \r\n");
			
			printf("Ma Mingzhuo ");
			printf("2019190506002 \r\n");
			
			printf("Du Yifei ");
			printf("2019190505001 \r\n");
			
      /* Display date Format : yy/mm/dd */
      printf("%02d/%02d/%02d\r\n",2000 + GetDate.Year, GetDate.Month, GetDate.Date);
      /* Display time Format : hh:mm:ss */
      printf("%02d:%02d:%02d\r\n",GetTime.Hours, GetTime.Minutes, GetTime.Seconds);
      if(GetDate.WeekDay==1){
			printf("Monday\r\n");
		}else if(GetDate.WeekDay==2){
			printf("Tuesday\r\n");
		}else if(GetDate.WeekDay==3){
			printf("Wednesday\r\n");
		}else if(GetDate.WeekDay==4){
			printf("Thursday\r\n");
		}else if(GetDate.WeekDay==5){
			printf("Friday\r\n");
		}else if(GetDate.WeekDay==6){
			printf("Saturday\r\n");
		}else if(GetDate.WeekDay==7){
			printf("Sunday\r\n");
		}
      printf("\r\n");
		LL_mDelay(10);
//		send = 0;
//	}
}

void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc)
{


		__HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(hrtc,RTC_FLAG_WUTF);
//		Get_Time();
		clocker++;
		if(clocker > 10000) clocker = 0;
	
}

uint8_t Wait(uint16_t seconds)
{
	if(c_flag == 0)
	{
		clocker = 0;
		c_flag = 1;
	}
	
	if(clocker >= seconds)
	{
		c_flag = 0;
		Drive(0,0);
		return 1;
	}
	else return 0;
}

uint8_t Update_Angle(uint16_t seconds)
{
	if(c_flag == 0)
	{
		clocker = 0;
		c_flag = 1;
	}
	
	if(clocker >= seconds)
	{
		IMU_Command(ANGLE_INIT);
		c_flag = 0;
		OpenMv_angle = 0.8 * MyCV.angle_transfer;
		return 1;
	}
	else return 0;
}

uint8_t Run_time(uint16_t seconds,short speed)
{
	if(c_flag == 0)
	{
		clocker = 0;
		c_flag = 1;
		Drive(speed,0);
	}
	
	if(clocker >= seconds)
	{
		c_flag = 0;
		Drive(0,0);
		return 1;
	}
	else return 0;
}

uint8_t Run_time_PID(uint16_t seconds,short speed)
{
	IMU_Receive(Rx3Buffer);
	if(c_flag == 0)
	{
		clocker = 0;
		c_flag = 1;
	}
	Drive(speed,G0.A_speed);
	if(clocker >= seconds)
	{
		c_flag = 0;
		Drive(0,0);
		return 1;
	}
	else return 0;
}

uint8_t Turn_angle(uint16_t seconds, float angle)
{
	IMU_Receive(Rx3Buffer);
	OpenMv_angle = angle;
	Drive(0,G0.A_speed);
	if(Wait(seconds) == 1)
		{
			return 1;
		}
	else return 0;
}

uint8_t Turn_angle_OL(uint16_t seconds, short speed)
{
	IMU_Receive(Rx3Buffer);
	if(t_flag == 0)
	{
		G0.A_speed = speed;
		clocker = 0;
		t_flag = 1;
	}
	
	if(clocker >= seconds)
	{
		t_flag = 0;
		G0.A_speed = 0;
		return 1;
	}
	else return 0;
}

