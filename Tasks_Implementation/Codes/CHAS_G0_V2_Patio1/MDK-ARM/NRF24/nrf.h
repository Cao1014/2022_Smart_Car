/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __nRF24L01_H
#define __nRF24L01_H

//ͨ������� NRF24SPI_Send_Byte ����ʹ��Ӳ��SPI������ʹ��ģ��SPI
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "gpio.h"
//#define nRF24_SPI_Send_Byte
/* Exported types ------------------------------------------------------------*/
typedef struct
{
	uint8_t		Ack_Buf[32];
	uint8_t		Ack_Len;
	uint8_t		Ack_Channel;
	uint8_t		Ack_Status;
}ACK_PAYLOAD;
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported define -----------------------------------------------------------*/
	// 1.GND	2.VCC
	// 3.CE		4.nCS		//RX/TXģʽѡ���		//SPIƬѡ��SS
	// 5.SCK	6.MOSI		//SPIʱ�Ӷ�				//SPI��������ӻ������
	// 7.MISO	8.IRQ		//SPI��������ӻ������	//�������ж϶�
//==============================


	#define debug_out(x)		//printf x
	#define CE_H()		( HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, GPIO_PIN_SET) )
	#define CE_L()		( HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, GPIO_PIN_RESET) )		//GPIO_PORT_nRF->BRR  = GPIO_Pin_nRF_CE
	#define CSN_H()		( HAL_GPIO_WritePin(NRF_CS_GPIO_Port, NRF_CS_Pin, GPIO_PIN_SET) )		//GPIO_PORT_nRF_SPI->BSRR = GPIO_Pin_nRF_nCS
	#define CSN_L()		( HAL_GPIO_WritePin(NRF_CS_GPIO_Port, NRF_CS_Pin, GPIO_PIN_RESET) )		//GPIO_PORT_nRF_SPI->BRR  = GPIO_Pin_nRF_nCS

	#define nRF_IRQ()			HAL_GPIO_ReadPin(NRF_IRQ_GPIO_Port,NRF_IRQ_Pin)//GPIO_ReadInputDataBit(GPIO_PORT_nRF_IRQ, GPIO_Pin_nRF_IRQ)
//==============================
//nRF24L01�Ĵ�����������
	#define nRF_READ_REG        0x00	//�����üĴ���,��5λΪ�Ĵ�����ַ
	#define nRF_WRITE_REG       0x20	//д���üĴ���,��5λΪ�Ĵ�����ַ
	
	#define nRF_R_RX_PLOAD		0x61	//����������,1~32�ֽ�
	#define nRF_W_TX_PLOAD		0xA0	//д��������,1~32�ֽ�
	
	#define nRF_FLUSH_RX        0xE2	//���RX FIFO�Ĵ���.����ģʽ����,�ڴ���Ӧ���źŹ����в�Ӧִ�д�ָ��
	#define nRF_FLUSH_TX        0xE1	//���TX FIFO�Ĵ���.����ģʽ����
	
	#define nRF_REUSE_TX_PL     0xE3	//�����ط�,����ʹ����һ������,CEΪ��,���ݰ������Ϸ���.
	
	#define nRF_R_RX_PL_WID		0x60	//��RX FIFO����R_RX_PLOAD�غɳ���,������32�ֽ������RX FIFO
	#define nRF_W_ACK_PAYLOAD	0xA8//+ppp	//RXģʽ��,��PIPEppp(ppp�ķ�ΧΪ000~101)�е�AcK�����غ�һ��д��,��������3�������ACK��.
	#define nRF_W_TX_PAYLOAD_NOACK	0xB0//��������,����ֹ�Զ�Ӧ��,����ģʽ��
	
	#define nRF_NOP				0xFF	//�ղ���,����������״̬�Ĵ���
//=======�������
	#define Activate			0x50
	#define Code_Activate		0x73
//==============================
//SPI(nRF24L01)�Ĵ�����ַ
	#define CONFIG				0x00//  �����շ�״̬��CRCУ��ģʽ�Լ��շ�״̬��Ӧ��ʽ
	#define EN_AA				0x01//  �Զ�Ӧ��������
	#define EN_RXADDR			0x02//  �����ŵ�����
	#define SETUP_AW			0x03// �շ���ַ�������
	#define SETUP_RETR			0x04// �Զ��ط���������
	#define RF_CH				0x05// ����Ƶ������
	#define RF_SETUP			0x06// �������ʡ����Ĺ�������0b0010 0111
	#define STATUS				0x07  // ״̬�Ĵ�����д1�����Ӧ���жϣ�
	#define OBSERVE_TX			0x08  // ���ͼ�⹦��
	#define CD					0x09  // ��ַ���
	#define RX_ADDR_P0			0x0A  // Ƶ��0�������ݵ�ַ��ͨ��0�ĵ�ַ ע�� λ39��λ0��������� ��
	#define RX_ADDR_P1			0x0B  // Ƶ��1�������ݵ�ַ��ͨ��1�ĵ�ַ��ע��λ39��λ0��������ģ�
	#define RX_ADDR_P2			0x0C  // Ƶ��2�������ݵ�ַ��ͨ��2�ĵ�ַ��ע��λ39��λ8ͬͨ��1,λ7��λ0��������ģ�
	#define RX_ADDR_P3			0x0D  // Ƶ��3�������ݵ�ַ��ͨ��3�ĵ�ַ��ע��λ39��λ8ͬͨ��1,λ7��λ0��������ģ�
	#define RX_ADDR_P4			0x0E  // Ƶ��4�������ݵ�ַ��ͨ��4�ĵ�ַ��ע��λ39��λ8ͬͨ��1,λ7��λ0��������ģ�
	#define RX_ADDR_P5			0x0F  // Ƶ��5�������ݵ�ַ��ͨ��5�ĵ�ַ��ע��λ39��λ8ͬͨ��1,λ7��λ0��������ģ�
	#define TX_ADDR				0x10  // ���͵�ַ�Ĵ����������ַ��ע�� λ39��λ0��������ģ�
	#define RX_PW_P0			0x11  // ����Ƶ��0�������ݳ���
	#define RX_PW_P1			0x12  // ����Ƶ��1�������ݳ���
	#define RX_PW_P2			0x13  // ����Ƶ��2�������ݳ���
	#define RX_PW_P3			0x14  // ����Ƶ��3�������ݳ���
	#define RX_PW_P4			0x15  // ����Ƶ��4�������ݳ���
	#define RX_PW_P5			0x16  // ����Ƶ��5�������ݳ���
	#define FIFO_STATUS			0x17  // FIFOջ��ջ��״̬�Ĵ������ã�ֻ����
//=======�������
	#define DYNPD				0x1C  // nRF24L01 Dynamic payload setup
	#define FEATURE				0x1D  // nRF24L01 Exclusive feature setup
//==============================
	#define MAX_RT				0x10	//�ﵽ����ʹ����ж�
	#define TX_DS   			0x20	//TX��������ж�
	#define RX_DR   			0x40	//���յ������ж�

	#define nRF_Mode_TX			0
	#define nRF_Mode_RX			1
//==============================
//24L01���ͽ������ݿ�ȶ���
	#define TX_ADR_WIDTH    	5		//5�ֽڵĵ�ַ���
	#define RX_ADR_WIDTH    	5		//5�ֽڵĵ�ַ���
	#define RX_PLOAD_WIDTH  	32		//ָ�����û����ݿ��
	#define ACK_PLOAD_WIDTH  	5		//ָ�����û����ݿ��
	
	//���ջ�,ACK Ӧ��ͨ��N,����ͨ��0~5,(���250*4=1ms,2��,)����250K,0db,У��9λ,����ģʽ
	#define nRF_RX_CONFIG		0x0,  0x0,  0x26, 40, 0x26, 0x0f			//��Ҫ�����û���õ�
	//�����,ACK Ӧ��ͨ��0,����ͨ��0,���250*4=1ms,2��,����250K,0db,У��9λ,����ģʽ
	#define nRF_TX_CONFIG		0x01, 0x01, 0x1a, 40, 0x26, 0x0e			//��Ҫ�����û���õ�
//==============================
	#define DYNAMIC_PAYLOAD		1//ʹ�ö�̬�������ݳ���[�������ʱ,���ջ����Ծ�̬���շ�����Ķ�̬����]
	#if		DYNAMIC_PAYLOAD
	#define DYNAMIC_PAYLOAD_ACK	1//ʹ�ö�̬���ݳ���ʱACK����
	#if		DYNAMIC_PAYLOAD_ACK
	#define DYNPD_ACK_DATA		1//ʹ�ö�̬���ݳ���ʱ������ACK����
	#endif
	#endif
	#define nRF_TX_Channel		0//����ͨ����(����ڽ��ջ��Ľ���ͨ����)
	
	#define nRF_PTX				0//�������־
/* Exported functions ------------------------------------------------------- */
	void nRF24L01_Configuration(void);
	void nRF24L01_Set_Config(void);
	uint8_t nRF24L01_Check(void);
	void nRF24L01_Set_Mode(uint8_t rx_mode);
//==============================
	uint8_t nRF24L01_RxPacket(uint8_t * rx_buf);
	uint8_t nRF24L01_TxPacket(uint8_t tx_channel, uint8_t * tx_buf, uint8_t len);
	uint8_t nRF24L01_Tx_Ack(ACK_PAYLOAD *ack_pay);
	void nRF24L01_Rx_AckPayload(ACK_PAYLOAD ack_pay);
/* External variables --------------------------------------------------------*/
	extern uint8_t nRF24L01_TxBuf[32];
	extern uint8_t nRF24L01_RxBuf[32];

	extern ACK_PAYLOAD nRF24L01_ack_pay;
#endif
//==============================
