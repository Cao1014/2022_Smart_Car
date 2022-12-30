#ifndef __MY_ROBOT__
#define __MY_ROBOT__
#include "nrf.h"
#include "stdbool.h"

//#define ROCKER_LX		0
//#define ROCKER_LY		1
//#define ROCKER_RX		2
//#define ROCKER_RY		3

/**********设置接受模式init***********
nrf_mode = true;
Handle_Init(&hspi4);
***********中断中更新手柄cmd**********
instruction_refresh()；
***********得到摇杆、按键值***********
read_rocker(int id)
void read_keys(void)
*/

typedef struct
{
	
	short nrf_lspeed;
	short nrf_aspeed;

	short ls_abs;
	short as_abs;
	
	uint8_t Mode;
	
} NRF_COM;

extern bool nrf_mode;//nrf工作模式

extern uint8_t button[26];//按键

extern int16_t nrf_trans_cmd[7];   //解析后手柄全部数据

void NRF_IRQ(void);
void KeyToSpeed(void);
void KeyToMode(void);
void Car_Drive(uint8_t mode);

void nrf_init(void);

/******************************************************************
*@ 	name		: instruction_refresh	
*@	functio	: 手柄数据接收						
*@	input		: none							
*@	output	: none															
*******************************************************************/ 
void instruction_refresh(void);

void Ack_load(uint8_t* ack_data, uint8_t lenth);

/******************************************************************
*@ 	name		: read_rocker	
*@	functio	: 获得摇杆数据		编号0~3				
*@	input		: none							
*@	output	: none															
*******************************************************************/ 
int16_t read_rocker(int id);



/******************************************************************
*@ 	name		: read_keys	
*@	functio	: 获得按键数据，放在 boll button[26]中						
*@	input		: none							
*@	output	: none															
*******************************************************************/ 
void read_keys(void);

#endif
