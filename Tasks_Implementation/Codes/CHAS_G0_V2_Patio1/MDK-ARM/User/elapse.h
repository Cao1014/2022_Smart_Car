/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ELAPSE_H__
#define __ELAPSE_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdlib.h>
#include "stdio.h"
#include "string.h"
#include "math.h"
	
void Clocker_RST(void);
	
void Get_Time(void);
void Send_Time(void);
	
uint8_t Wait(uint16_t seconds);
	
uint8_t Update_Angle(uint16_t seconds);	
uint8_t Run_time(uint16_t seconds, short speed);
uint8_t Run_time_PID(uint16_t seconds, short speed);
uint8_t Turn_angle_OL(uint16_t seconds, short speed);
uint8_t Turn_angle(uint16_t seconds, float angle);
	
#endif /* __ELAPSE_H__ */
