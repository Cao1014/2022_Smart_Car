/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */
#include "rtc.h"
#include "string.h"
#include "arm_math.h"

uint8_t Rx3Frame = {0}; 					//接受数据单帧
uint8_t Rx3Buffer[IMUBUFFERSIZE]; //接收数据
uint8_t Usart3_Rx_Cnt = 0;				//接收缓冲计数

uint8_t Rx2Frame = {0}; 					//视觉数据单帧
uint8_t Rx2Buffer[CVBUFFERSIZE];  //视觉数据
uint8_t Usart2_Rx_Cnt = 0;				//视觉缓冲计数

uint8_t Rx1Frame = {0}; 					//蓝牙数据单帧
uint8_t Rx1Buffer[BLBUFFERSIZE];  //蓝牙数据
uint8_t Usart1_Rx_Cnt = 0;				//蓝牙缓冲计数

RTC_DateTypeDef UARTDate;  	//电脑日期结构体
RTC_TimeTypeDef UARTTime;   //电脑时间结构体

uint8_t flag = 0;
uint8_t CV_flag = 0;

float OpenMv_angle = 0;

IMU MyIMU;
CV_Data MyCV;

/* USER CODE END 0 */

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart4;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 19200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}
/* USART2 init function */

void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}
/* USART3 init function */

void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}
/* USART4 init function */

void MX_USART4_UART_Init(void)
{

  /* USER CODE BEGIN USART4_Init 0 */

  /* USER CODE END USART4_Init 0 */

  /* USER CODE BEGIN USART4_Init 1 */

  /* USER CODE END USART4_Init 1 */
  huart4.Instance = USART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART4_Init 2 */

  /* USER CODE END USART4_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    LL_RCC_LSE_SetDriveCapability(LL_RCC_LSEDRIVE_LOW);
    LL_RCC_SetUSARTClockSource(LL_RCC_USART1_CLKSOURCE_PCLK1);

    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PB6     ------> USART1_TX
    PB7     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF0_USART1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */

  /* USER CODE END USART2_MspInit 0 */

    LL_RCC_LSE_SetDriveCapability(LL_RCC_LSEDRIVE_LOW);
    LL_RCC_SetUSARTClockSource(LL_RCC_USART2_CLKSOURCE_PCLK1);

    /* USART2 clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART2 interrupt Init */
    HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspInit 1 */

  /* USER CODE END USART2_MspInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspInit 0 */

  /* USER CODE END USART3_MspInit 0 */

    LL_RCC_LSE_SetDriveCapability(LL_RCC_LSEDRIVE_LOW);

    /* USART3 clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**USART3 GPIO Configuration
    PB8     ------> USART3_TX
    PB9     ------> USART3_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_USART3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USART3 interrupt Init */
    HAL_NVIC_SetPriority(USART3_4_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART3_4_IRQn);
  /* USER CODE BEGIN USART3_MspInit 1 */

  /* USER CODE END USART3_MspInit 1 */
  }
  else if(uartHandle->Instance==USART4)
  {
  /* USER CODE BEGIN USART4_MspInit 0 */

  /* USER CODE END USART4_MspInit 0 */

    LL_RCC_LSE_SetDriveCapability(LL_RCC_LSEDRIVE_LOW);

    /* USART4 clock enable */
    __HAL_RCC_USART4_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART4 GPIO Configuration
    PA0     ------> USART4_TX
    PA1     ------> USART4_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF4_USART4;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART4 interrupt Init */
    HAL_NVIC_SetPriority(USART3_4_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART3_4_IRQn);
  /* USER CODE BEGIN USART4_MspInit 1 */

  /* USER CODE END USART4_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PB6     ------> USART1_TX
    PB7     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6|GPIO_PIN_7);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspDeInit 0 */

  /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();

    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3);

    /* USART2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspDeInit 1 */

  /* USER CODE END USART2_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspDeInit 0 */

  /* USER CODE END USART3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART3_CLK_DISABLE();

    /**USART3 GPIO Configuration
    PB8     ------> USART3_TX
    PB9     ------> USART3_RX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8|GPIO_PIN_9);

    /* USART3 interrupt Deinit */
  /* USER CODE BEGIN USART3:USART3_4_IRQn disable */
    /**
    * Uncomment the line below to disable the "USART3_4_IRQn" interrupt
    * Be aware, disabling shared interrupt may affect other IPs
    */
    /* HAL_NVIC_DisableIRQ(USART3_4_IRQn); */
  /* USER CODE END USART3:USART3_4_IRQn disable */

  /* USER CODE BEGIN USART3_MspDeInit 1 */

  /* USER CODE END USART3_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART4)
  {
  /* USER CODE BEGIN USART4_MspDeInit 0 */

  /* USER CODE END USART4_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART4_CLK_DISABLE();

    /**USART4 GPIO Configuration
    PA0     ------> USART4_TX
    PA1     ------> USART4_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0|GPIO_PIN_1);

    /* USART4 interrupt Deinit */
  /* USER CODE BEGIN USART4:USART3_4_IRQn disable */
    /**
    * Uncomment the line below to disable the "USART3_4_IRQn" interrupt
    * Be aware, disabling shared interrupt may affect other IPs
    */
    /* HAL_NVIC_DisableIRQ(USART3_4_IRQn); */
  /* USER CODE END USART4:USART3_4_IRQn disable */

  /* USER CODE BEGIN USART4_MspDeInit 1 */

  /* USER CODE END USART4_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void IMU_Init(void)
{
	
	MyIMU.IMU_Command[0] = 0xFF;
	MyIMU.IMU_Command[1] = 0xAA;
	
	MyIMU.Roll = 0.0;
	MyIMU.Pitch = 0.0;
	MyIMU.Yaw = 0.0;
	
	MyIMU.Temp = 0.0;
	
}

void IMU_Command(uint8_t Command)
{
	switch(Command)
	{
		case ANGLE_INIT:
			MyIMU.IMU_Command[2] = 0X52;
		break;
		
		case ACCLE_CALIBRA:
			MyIMU.IMU_Command[2] = 0X67;
		break;
		
		case SLEEP:
			MyIMU.IMU_Command[2] = 0X60;
		break;
		
		case HORIZONTAL:
			MyIMU.IMU_Command[2] = 0X65;
		break;
		
		case VERTICAL:
			MyIMU.IMU_Command[2] = 0X66;
		break;
		
		default: break;
	}	
	HAL_UART_Transmit(&huart3, (uint8_t *)MyIMU.IMU_Command, 3, 0xFF); 
}

void IMU_Receive(uint8_t Buffer[])
{

  HAL_UART_Receive_IT(&huart3, (uint8_t *)Buffer, 1);
	
}

void IMU_Decode(uint8_t Receive[])
{
	if(flag)
	{
		if(Receive[1] == 0x53)
		{
//	MyIMU.Roll = ((short)(Receive[3]<<8|Receive[2]))/32768.0*180;
//	MyIMU.Pitch = ((short)(Receive[5]<<8|Receive[4]))/32768.0*180;
		MyIMU.Yaw = ((short)(Receive[7]<<8|Receive[6]))/32768.0*180;
		}
//	else if(Receive[1] == 0x52)
//	MyIMU.Temp = ((short)(Receive[9]<<8|Receive[8]))/340.0+36.25;
		
//		if(Receive[1] == 0x52)
//		{
//			MyIMU.w_z = ((short)(Receive[7]<<8|Receive[6]))/32768.0*2000;
//			MyIMU.Yaw += MyIMU.w_z * 0.2;
//		}
		flag = 0;
	}
}

void CV_init(void)
{
	memset(&MyCV.raw_angle, 0x00, ANGLE_SAMPLE_SIZE);
	memset(&MyCV.angle_mean, 0x00, 2);
	MyCV.angle_transfer = 0;
		
}
void CV_Receive(uint8_t Buffer[])
{

  HAL_UART_Receive_IT(&huart2, (uint8_t *)Buffer, 1);
	
}

void CV_Decode(uint8_t Receive[])
{
	if(Receive[1] == 0xEE && Receive[4] == 0xBB) //Patio1
	{
		
		MyCV.raw_angle[MyCV.angle_counter] = (Receive[3]<<8|Receive[2]);
		MyCV.angle_counter ++;
		
		if(MyCV.angle_counter == ANGLE_SAMPLE_SIZE )
		{
			arm_mean_q15(MyCV.raw_angle,ANGLE_SAMPLE_SIZE,&MyCV.angle_mean[0]);
			memcpy(&MyCV.angle_transfer, &MyCV.angle_mean[0], sizeof(short));
			
			if(MyCV.angle_transfer < 11.0f && MyCV.angle_transfer > -11.0f) MyCV.angle_transfer = 0.0f;
			
			MyCV.angle_counter = 0;
			
			OpenMv_angle = 0.8 * MyCV.angle_transfer;
			CV_flag = 1; //数据解算结束	
		}

	}
	else if(Receive[1] == 0xFF) //Patio2
	{
		MyCV.shape = (Receive[3]<<8|Receive[2]);
		CV_flag = 1; //数据解算结束	

	}
}

void BL_Receive(uint8_t Buffer[])
{

  HAL_UART_Receive_IT(&huart1, (uint8_t *)Buffer, 1);
	
}

void BL_Decode(uint8_t Receive[])
{

	if(Receive[21] == 0x0A) //判断结束位
	{
		UARTDate.Year = RTC_ByteToBcd2((Receive[3] - '0') * 10 + (Receive[4] - '0'));
		UARTDate.Month = RTC_ByteToBcd2((Receive[6] - '0') * 10 + (Receive[7] - '0'));
		UARTDate.Date = RTC_ByteToBcd2((Receive[9] - '0') * 10 + (Receive[10] - '0'));
		
		UARTTime.Hours = RTC_ByteToBcd2((Receive[12] - '0') * 10 + (Receive[13] - '0'));
		UARTTime.Minutes = RTC_ByteToBcd2((Receive[15] - '0') * 10 + (Receive[16] - '0'));
		UARTTime.Seconds = RTC_ByteToBcd2((Receive[18] - '0') * 10 + (Receive[19] - '0'));
		
		UARTDate.WeekDay = RTC_ByteToBcd2((Receive[20] - '0'));
		
	}

}


/**
  * @brief  Rx Transfer completed callback
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report end of DMA Rx transfer, and 
  *         you can add your own implementation.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{

  /* NOTE: This function Should not be modified, when the callback is needed,
           the HAL_UART_TxCpltCallback could be implemented in the user file
   */
	if(UartHandle->Instance == USART3) 
	{
		Rx3Buffer[Usart3_Rx_Cnt] = Rx3Frame;
		if(Usart3_Rx_Cnt >= IMUBUFFERSIZE - 1)  //溢出判断
		{
			Usart3_Rx_Cnt = 0;
			flag = 1;
//			memset(Rx3Buffer,0x00,sizeof(Rx3Buffer));
//			HAL_UART_Transmit(&huart3, (uint8_t *)"数据溢出", 10,0xFFFF); 	
					
		}

		else
		{
			if(Usart3_Rx_Cnt == 0 && Rx3Buffer[0] != 0x55) 				Usart3_Rx_Cnt = 0;
			else if(Usart3_Rx_Cnt == 0 && Rx3Buffer[0] == 0x55) 	Usart3_Rx_Cnt++;
			else if(Usart3_Rx_Cnt == 1 && Rx3Buffer[1] != 0x53) 	Usart3_Rx_Cnt = 0;
			else if(Rx3Buffer[1] == 0x53) 												Usart3_Rx_Cnt++;
				
		}
		
		HAL_UART_Receive_IT(&huart3, (uint8_t *)&Rx3Frame, 1);   //再开启接收中断
	}
	else if(UartHandle->Instance == USART2) 
	{
		
		Rx2Buffer[Usart2_Rx_Cnt] = Rx2Frame;
		Usart2_Rx_Cnt++;
		
		
		if(Rx2Buffer[0] != 0xAA) //判断起始帧
		{
			memset(Rx2Buffer,0x00,CVBUFFERSIZE);
			Usart2_Rx_Cnt = 0;
		}
		
		if(Usart2_Rx_Cnt >= CVBUFFERSIZE)  //溢出判断
		{
			Usart2_Rx_Cnt = 0;
		}
		
		Rx2Frame = 0;
		HAL_UART_Receive_IT(&huart2, (uint8_t *)&Rx2Frame, 1);   //再开启接收中断
	}
	
	else if(UartHandle->Instance == USART1) 
	{
		Rx1Buffer[Usart1_Rx_Cnt] = Rx1Frame;
		
		if(Usart1_Rx_Cnt >= BLBUFFERSIZE - 1)  //溢出判断
		{
			Usart1_Rx_Cnt = 0;
		}
		
		else //数据未接收完成
		{
			if(Usart1_Rx_Cnt == 0 && Rx1Buffer[0] != 0x41)
				Usart1_Rx_Cnt = 0;
			else
				Usart1_Rx_Cnt++;   //接收数据转存
		}
		HAL_UART_Receive_IT(&huart1, (uint8_t *)&Rx1Frame, 1);   //再开启接收中断
	}
	
}

/* USER CODE END 1 */
