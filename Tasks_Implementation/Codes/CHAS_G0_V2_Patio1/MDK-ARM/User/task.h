/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TASK_H__
#define __TASK_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

#define Square 		0xBB
#define Triangle 	0x03	
#define Circle		0xAA
	
typedef struct
{
	uint16_t P1_Stage;
	
	uint16_t P2_DStage;
	uint16_t P2_SStage;
	
	uint16_t P2_TStage;
	uint16_t P2_PStage;
	
}	Patio;

void Bridge(short speed);
uint8_t Receive_Shape(void);
void Diamond(short speed, uint8_t shape);
void Trash(uint16_t d_fence, uint16_t d_turn, short speed);
void Planter(short speed);

#endif /* __TAKS_H__ */
