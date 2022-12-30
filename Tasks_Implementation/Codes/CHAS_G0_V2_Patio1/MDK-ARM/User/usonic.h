/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USONIC_H__
#define __USONIC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
	
#define Sound_Speed 344
#define US_SAMPLE_SIZE 32

typedef struct
{
	volatile uint16_t CNT1;
	float Dis1;
	volatile uint16_t CNT2;
	float Dis2;
	
	short US1_rawdata[US_SAMPLE_SIZE];
	short US2_rawdata[US_SAMPLE_SIZE];
	
	short US_mean[2];
	
} usonic;

void USonic1(void);
void USonic2(void);

void US_Sample(void);
void Mean_Filter(void);

#endif /* __USONIC_H__ */
