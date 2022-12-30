/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern UART_HandleTypeDef huart1;

extern UART_HandleTypeDef huart2;

extern UART_HandleTypeDef huart3;

extern UART_HandleTypeDef huart4;

/* USER CODE BEGIN Private defines */
#define IMUBUFFERSIZE ((uint32_t)  11)
#define CVBUFFERSIZE ((uint32_t)  5)
#define BLBUFFERSIZE ((uint32_t)  22)
	
#define ANGLE_SAMPLE_SIZE ((uint32_t)  8)
	
#define ANGLE_INIT 0x52
#define ACCLE_CALIBRA 0x67
#define SLEEP 0x60
#define HORIZONTAL 0x65
#define VERTICAL 0x66

/* USER CODE END Private defines */

void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);
void MX_USART3_UART_Init(void);
void MX_USART4_UART_Init(void);

/* USER CODE BEGIN Prototypes */
void IMU_Init(void);
void IMU_Command(uint8_t Command);
void IMU_Receive(uint8_t Buffer[]);
void IMU_Decode(uint8_t Receive[]);

void CV_init(void);
void CV_Receive(uint8_t Buffer[]);
void CV_Decode(uint8_t Receive[]);

void BL_Receive(uint8_t Buffer[]);
void BL_Decode(uint8_t Receive[]);

typedef struct
{
	
	uint8_t IMU_Command[3];
	
	volatile float w_z;
	
	float Roll;
	float Pitch;
	float Yaw;
	float Temp;
	
}IMU;

typedef struct
{
	short raw_angle[ANGLE_SAMPLE_SIZE];
	short angle_mean[1];
	short angle_transfer;
	uint8_t angle_counter;
	
	uint8_t shape;
	
} CV_Data;


/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

