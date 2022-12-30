//2013-10-28���Զ෢һ������,����:
//�̶������շ�,ʹ��ACK����ʹ��ACK
//�ɱ䳤���շ�,ʹ��ACK����ʹ��ACK,�Լ�ʹ�ô��ش����ݵ�ACK
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "nrf.h"
#include "stdio.h"
#include "spi.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define car_num 1
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Extern variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void delay_us(uint32_t n);
/* Private functions ---------------------------------------------------------*/
ACK_PAYLOAD nRF24L01_ack_pay;

uint8_t nRF24L01_TxBuf[32]={0};
uint8_t nRF24L01_RxBuf[32]={0};

//���ջ�
uint8_t RX_ADDRESS0[RX_ADR_WIDTH] = {0x20,0x22,0x2c,0x01,0x01};	// �ŵ�0 = TXDEV0_TX_ADDR = DEV1_RX_ADDRESS0{0x30, 0x37, 0x33, 0x34, 0x30}
uint8_t RX_ADDRESS1[RX_ADR_WIDTH] = {0x20,0x22,0x2c,0x01,0x22};	// �ŵ�1 = TXDEV1_TX_ADDR = DEV1_RX_ADDRESS0{0x31, 0x37, 0x33, 0x34, 0x30}
uint8_t RX_ADDRESS2[1]= {0x32};										// �ŵ�2 = TXDEV2_TX_ADDR = DEV1_RX_ADDRESS0{0x32, 0x37, 0x33, 0x34, 0x30}
uint8_t RX_ADDRESS3[1]= {0x33};										// �ŵ�3 = TXDEV3_TX_ADDR = DEV1_RX_ADDRESS0{0x33, 0x37, 0x33, 0x34, 0x30}
uint8_t RX_ADDRESS4[1]= {0x34};										// �ŵ�4 = TXDEV4_TX_ADDR = DEV1_RX_ADDRESS0{0x34, 0x37, 0x33, 0x34, 0x30}
uint8_t RX_ADDRESS5[1]= {0x35};										// �ŵ�5 = TXDEV5_TX_ADDR = DEV1_RX_ADDRESS0{0x35, 0x37, 0x33, 0x34, 0x30}

uint8_t tx_channel_addr[RX_ADR_WIDTH] = {0x20,0x22,0x2c,0x01,0x01};
uint8_t nRF24_SPI_Send_Byte(uint8_t data)
{
	uint8_t	ret;
	HAL_SPI_TransmitReceive(&hspi2,&data,&ret,1,100);
	return ret;	
}

/**********************************************************************	
*@	function: ��delay				
*@	input		: none							
*@	output	: none															
***********************************************************************/ 
void delay_us(uint32_t udelay)
{
  uint32_t startval,tickn,delays,wait;

  startval = SysTick->VAL;
  tickn = HAL_GetTick();
  //sysc = 72000;  //SystemCoreClock / (1000U / uwTickFreq);
  delays =udelay * 64; //sysc / 1000 * udelay;
  if(delays > startval)
    {
      while(HAL_GetTick() == tickn)
        {

        }
      wait = 64000 + startval - delays;
      while(wait < SysTick->VAL)
        {

        }
    }
  else
    {
      wait = startval - delays;
      while(wait < SysTick->VAL && HAL_GetTick() == tickn)
        {

        }
    }
}


/*===============================�Ĵ�����д==========================================*/
uint8_t nRF24L01_Read_Reg(uint8_t reg)
{
	uint8_t reg_val;

	CSN_L();						// CSN low, initialize SPI communication...
	nRF24_SPI_Send_Byte(reg);		// Select register to read from..
	reg_val = nRF24_SPI_Send_Byte(0xff);// ..then read registervalue
	CSN_H();						// CSN high, terminate SPI communication

	return(reg_val);				// return register value
}

uint8_t nRF24L01_Write_Reg(uint8_t reg, uint8_t value)
{
	uint8_t status;

	CSN_L();						// CSN low, init SPI transaction
	status = nRF24_SPI_Send_Byte(reg);// select register
	nRF24_SPI_Send_Byte(value);		// ..and write value to it..
	CSN_H();						// CSN high again

	return(status);					// return nRF24L01 status uint8_t
}

uint8_t nRF24L01_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t Len)
{
	uint8_t status, i;

	CSN_L();							// Set CSN low, init SPI tranaction
	status = nRF24_SPI_Send_Byte(reg);	// Select register to write to and read status uint8_t

	for(i = 0; i < Len; i++)
	{
		pBuf[i] = nRF24_SPI_Send_Byte(0xff);
	}

	CSN_H();                           

	return(status);						// return nRF24L01 status uint8_t
}

uint8_t nRF24L01_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t Len)
{
	uint8_t status, i;

	CSN_L();				//SPIʹ��       
	status = nRF24_SPI_Send_Byte(reg);   
	for(i = 0; i < Len; i++)//
	{
		nRF24_SPI_Send_Byte(pBuf[i]);
	}
	CSN_H();				//�ر�SPI
	return(status);			// 
}

/*===============================end==========================================*/




/**********************************************************************	
*@	function: �Ĵ�����ʼ��			
*@	input		: none							
*@	output	: none															
***********************************************************************/ 
void nRF24L01_Set_Config(void)
{	
	CE_L();

		nRF24L01_Write_Reg(nRF_WRITE_REG | SETUP_AW,    0x03);											// 0x03��Ӧ���5
		nRF24L01_Write_Buf(nRF_WRITE_REG | RX_ADDR_P0, RX_ADDRESS0, RX_ADR_WIDTH);	// д���ն˵�ַ
//		nRF24L01_Write_Buf(nRF_WRITE_REG | RX_ADDR_P1, RX_ADDRESS1, RX_ADR_WIDTH);	// д���ն˵�ַ
	//nRF24L01_Write_Buf(nRF_WRITE_REG | TX_ADDR,    RX_ADDRESS0, RX_ADR_WIDTH);	//2-5ͨ��ʹ��1�ֽ�
		nRF24L01_Write_Reg(nRF_WRITE_REG | EN_AA,       0x01);											// Ƶ��0�Զ�	ACKӦ������	
		nRF24L01_Write_Reg(nRF_WRITE_REG | EN_RXADDR,   0x01);											//ͨ��0�����������
		nRF24L01_Write_Reg(nRF_WRITE_REG | SETUP_RETR,  0x1a);											//�Զ��ط���ʱ��500+86us��10��
		nRF24L01_Write_Reg(nRF_WRITE_REG | RF_CH,       50);												//����ͨ��Ƶ�ʣ��շ�һ������
		nRF24L01_Write_Reg(nRF_WRITE_REG | RF_SETUP,    0x0f);											//���ݴ����ʣ�2Mbps�����书�ʣ�0DB��
		nRF24L01_Write_Reg(nRF_WRITE_REG | CONFIG,      0x0f);											//16λCRCʹ�ܣ�����ģʽ������ȫ���ж�
		nRF24L01_Write_Reg(nRF_WRITE_REG | nRF_FLUSH_TX, 0xff);											//���TX FIFO�Ĵ���.����ģʽ����
		nRF24L01_Write_Reg(nRF_WRITE_REG | nRF_FLUSH_RX, 0xff);											//���RX FIFO�Ĵ���.����ģʽ����,�ڴ���Ӧ���źŹ����в�Ӧִ�д�ָ��
		nRF24L01_Write_Reg(nRF_WRITE_REG | DYNPD, 0x01);														//ʹ��ͨ��0�Ķ�̬�غɳ���
		nRF24L01_Write_Reg(nRF_WRITE_REG | FEATURE,  0x06);													//ʹ��ACK����,ʹ��ACK�Ķ�̬�غɳ���

	

	CE_H();	// Set CE pin high to enable RX device
	delay_us(130);
//	CE_L();
}

/**********************************************************************	
*@	function:���24L01�Ƿ����		
*@	input		: none							
*@	output	: ����ֵ:�ɹ� 0 �� ʧ��	1													
***********************************************************************/ 
uint8_t nRF24L01_Check(void)		
{
	uint8_t buf[5] = {0xA5, 0xA5, 0xA5, 0xA5, 0xA5};
	uint8_t i;
	
	nRF24L01_Write_Buf(nRF_WRITE_REG | TX_ADDR, buf, 5);	//д��5���ֽڵĵ�ַ.
	nRF24L01_Read_Buf(nRF_READ_REG + TX_ADDR, buf, 5);		//����д��ĵ�ַ
	for(i = 0; i < 5; i++)
	{
		if(buf[i] != 0xA5)
		{
			break;
		}
	}
	if(i != 5)
	{
		debug_out(("nRF24L01 TEST FAUSE\r\n"));
		return 1;											//���24L01����
	}
	else 
	{
		debug_out(("nRF24L01 TEST OK\r\n"));
		return 0;											//��⵽24L01
	}
}

/**********************************************************************	
*@	function:�л�ģʽ��ûɶ�ã�	
*@	input		: none							
*@	output	: none												
***********************************************************************/ 
void nRF24L01_Set_Mode(uint8_t rx_mode)	//�޸�Ϊ����/����ģʽ
{
	uint8_t mode = nRF24L01_Read_Reg(CONFIG);
	//debug_out(("mode=0x%02x\r\n",mode));
	if(rx_mode)	//1����ģʽ
	{
	 	nRF24L01_Write_Reg(nRF_WRITE_REG | CONFIG, mode | 0x01);	
	}
	else		//0����ģʽ
	{
	 	nRF24L01_Write_Reg(nRF_WRITE_REG | CONFIG, mode & 0xfe);	
	}
}

//==============================���ա��������ݰ�===========================================
uint8_t nRF24L01_RxPacket(uint8_t * rx_buf)
{
	uint8_t len = 0;
	uint8_t status;
	
//	CE_L();
	status = nRF24L01_Read_Reg(STATUS);							// ��ȡ״̬�Ĵ������ж����ݽ���״��
//	nRF24L01_Write_Reg(nRF_WRITE_REG | STATUS, status & RX_DR);			//���յ����ݺ�RX_DR,TX_DS,MAX_RT���ø�Ϊ1��ͨ��д1������жϱ�־
	nRF24L01_Write_Reg(nRF_WRITE_REG | STATUS, status);			//���յ����ݺ�RX_DR,TX_DS,MAX_RT���ø�Ϊ1��ͨ��д1������жϱ�־
	
#if (DYNPD_ACK_DATA)
	if(status & TX_DS)//
	{
		//debug_out(("PRX TX_DS\r\n"));
	}
#endif
	if(status & RX_DR)											// �ж��Ƿ���յ�����
	{
		len = nRF24L01_Read_Reg(nRF_R_RX_PL_WID);
		if(len < 33)
		{
			nRF24L01_Read_Buf(nRF_R_RX_PLOAD, rx_buf, len);	// read receive payload from RX_FIFO buffer
		}
		else 
		{
			len = 0;
		}
		nRF24L01_Write_Reg(nRF_WRITE_REG | nRF_FLUSH_RX,0xff);	//��ջ�����
		
		nRF24L01_ack_pay.Ack_Channel = (status & 0x0e) >> 1;
	}
	
//	CE_H();	//�ø�CE���������ݷ���
	return len;
}
uint8_t nRF24L01_TxPacket(uint8_t tx_channel, uint8_t * tx_buf, uint8_t len)
{
	CE_L();
	
	if((tx_channel == 0))
	{
		nRF24L01_Write_Buf(nRF_WRITE_REG | TX_ADDR,    RX_ADDRESS0, RX_ADR_WIDTH);//2-5ͨ��ʹ��1�ֽ�
		nRF24L01_Write_Buf(nRF_WRITE_REG | RX_ADDR_P0, RX_ADDRESS0, RX_ADR_WIDTH);// д���ն˵�ַ
	}
	else
	{
		tx_channel_addr[0] = 0x30 + tx_channel;
		nRF24L01_Write_Buf(nRF_WRITE_REG | TX_ADDR,    tx_channel_addr, RX_ADR_WIDTH);//2-5ͨ��ʹ��1�ֽ�
		nRF24L01_Write_Buf(nRF_WRITE_REG | RX_ADDR_P0, tx_channel_addr, RX_ADR_WIDTH);// д���ն˵�ַ
	}
#if DYNAMIC_PAYLOAD_ACK
	nRF24L01_Write_Buf(nRF_WRITE_REG | nRF_W_TX_PLOAD, tx_buf, len);	// װ������	
#else
	//nRF24L01_Write_Buf(nRF_WRITE_REG | nRF_W_TX_PLOAD, tx_buf, len);	// װ������	
	nRF24L01_Write_Buf(nRF_WRITE_REG | nRF_W_TX_PAYLOAD_NOACK, tx_buf, len);	// װ������
#endif
	
	CE_H();	//�ø�CE���������ݷ���
	delay_us(20);
	
	while(nRF_IRQ());										//�ȴ��������
	
	return (nRF24L01_Tx_Ack(&nRF24L01_ack_pay));
}
uint8_t nRF24L01_Tx_Ack(ACK_PAYLOAD *ack_pay)
{
 	uint8_t tmp = 200;
	uint8_t status;
	while(1)
	{
		status = nRF24L01_Read_Reg(STATUS);					//��ȡ״̬�Ĵ�����ֵ
		
		nRF24L01_Write_Reg(nRF_WRITE_REG | STATUS, status);	// ���TX_DS��MAX_RT�жϱ�־
		
#if (DYNPD_ACK_DATA)
		if(status & RX_DR)		// �ж��Ƿ���յ�����
		{
			ack_pay->Ack_Len = nRF24L01_Read_Reg(nRF_R_RX_PL_WID);
			if(ack_pay->Ack_Len < 33)
			{
				nRF24L01_Read_Buf(nRF_R_RX_PLOAD, ack_pay->Ack_Buf, ack_pay->Ack_Len);	// read receive payload from RX_FIFO buffer
				ack_pay->Ack_Status = 1;
			}
			else 
			{
				ack_pay->Ack_Len = 0;
			}
			nRF24L01_Write_Reg(nRF_WRITE_REG | nRF_FLUSH_RX,0xff);	//��ջ�����
			
			ack_pay->Ack_Channel = (status & 0x0e) >> 1;
			debug_out(("PTX RX_DR\r\n"));
		}
#endif
		if(status & MAX_RT)		//�ﵽ����ط�����
		{
			nRF24L01_Write_Reg(nRF_WRITE_REG | nRF_FLUSH_TX, 0xff);//���TX FIFO�Ĵ��� 
			debug_out(("PTX MAX_RT\r\n"));
			return MAX_RT;
		}
		else if(status & TX_DS)	//�������
		{
			debug_out(("PTX TX_DS\r\n"));
			return TX_DS;
		}
		else					//�ȴ�
		{
			delay_us(10);
			tmp--;
			if(tmp == 0)
			{
				nRF24L01_Write_Reg(nRF_WRITE_REG | nRF_FLUSH_TX, 0xff);//���TX FIFO�Ĵ��� 
				debug_out(("PTX time_out\r\n"));
				return 0xff;//����ԭ����ʧ��
			}
		}
	}
}

/*****************************************
*@ function : װ�ػش����ݰ�
*@
*****************************************/
#if (DYNPD_ACK_DATA)//�����ݵ�ACK����
void nRF24L01_Rx_AckPayload(ACK_PAYLOAD ack_pay)
{
	CE_L();
	nRF24L01_Write_Reg(nRF_WRITE_REG | nRF_FLUSH_TX, 0xff);//���TX FIFO�Ĵ��� 
//	switch(nRF24L01_ack_pay.Ack_Channel){
//		case 1:       
//			nRF24L01_Write_Buf(nRF_WRITE_REG | RX_ADDR_P0, RX_ADDRESS1, RX_ADR_WIDTH);// д���ն˵�ַ
//		break;
//		
//	}
		

	nRF24L01_Write_Buf(nRF_WRITE_REG | nRF_W_ACK_PAYLOAD | ack_pay.Ack_Channel, ack_pay.Ack_Buf, ack_pay.Ack_Len);// װ������
	CE_H();
	//debug_out(("���ش�����ACK\r\n"));
}
#endif
//==============================end===========================================

//-------------ʹ��˵��---------------------//
// 1.1.PTX�������ݵ�PRX
// #if (DYNAMIC_PAYLOAD) //��̬���ݳ���
// 		nRF24L01_TxPacket(channel, tmp_data, tmp_len);
// #else                 //�̶����ݳ���
// 		nRF24L01_TxPacket(channel, tmp_data, RX_PLOAD_WIDTH);
// #endif
// 1.2.PTX��PRX���շ���ACK����
// #if (DYNPD_ACK_DATA)  //���շ�������
//     if(nRF24L01_ack_pay.Ack_Status)
//     {
//         nRF24L01_ack_pay.Ack_Status = 0;
//         debug_out(("[%d]ACK(%d):%s\r\n",nRF24L01_ack_pay.Ack_Channel,nRF24L01_ack_pay.Ack_Len,nRF24L01_ack_pay.Ack_Buf));
//     }
// #endif
// 2.PRX����PTX������������
// tmp_len = nRF24L01_RxPacket(data, &channel);//��������
// if(tmp_len)
// {
// #if (DYNPD_ACK_DATA)//���ͷ����ACK����
//     nRF24L01_Rx_AckPayload(nRF24L01_ack_pay);
// #endif
// 		debug_out(("���յ�[%d]�ֽ� @%d:%s\r\n", tmp_len, channel, data));
// }
//-----------------------------------------//
/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
