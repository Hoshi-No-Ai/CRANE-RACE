#ifndef __SPI_H
#define __SPI_H
#include "STM32Lib\\stm32f10x_spi.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//Mini STM32������
//SPI ��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2010/6/13 
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ����ԭ�� 2009-2019
//All rights reserved
////////////////////////////////////////////////////////////////////////////////// 	  

 
 				  	    													  
void SPI2_Init(void);                          //SPI2��ʼ��
u8 SPI2_ReadWriteByte(u8 TxData);              //SPI2��дһ���ֽ�
void SPI2_SetSpeed(u8 SpeedSet);	           //����SPI�ٶ�

#endif

