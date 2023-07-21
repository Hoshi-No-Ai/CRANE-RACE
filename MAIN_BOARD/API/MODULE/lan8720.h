#ifndef __LAN8720_H
#define __LAN8720_H

#include "stm32f4x7_eth.h"
			
#define LAN8720_PHY_ADDRESS  	0x01				//LAN8720 PHYоƬ��ַ.

extern ETH_DMADESCTypeDef  *DMATxDescToSet;			//DMA����������׷��ָ��
extern ETH_DMADESCTypeDef  *DMARxDescToGet; 		//DMA����������׷��ָ�� 
extern ETH_DMA_Rx_Frame_infos *DMA_RX_FRAME_infos;	//DMA�����յ���֡��Ϣָ��
extern __align(4) ETH_DMADESCTypeDef DMARxDscrTab[ETH_RXBUFNB];	
extern __align(4) ETH_DMADESCTypeDef DMATxDscrTab[ETH_TXBUFNB];	
extern __align(4) uint8_t Rx_Buff[ETH_RX_BUF_SIZE*ETH_RXBUFNB]; 	
extern __align(4) uint8_t Tx_Buff[ETH_TX_BUF_SIZE*ETH_TXBUFNB]; 

u8 LAN8720_Init(void);
u8 LAN8720_Get_Speed(void);
u8 ETH_MACDMA_Config(void);
FrameTypeDef ETH_Rx_Packet(void);
u8 ETH_Tx_Packet(u16 FrameLength);
u32 ETH_GetCurrentTxBuffer(void);
u8 ETH_Mem_Malloc(void);
void ETH_Mem_Free(void);
static void ETHERNET_NVICConfiguration(void);

#endif 

