#ifndef _FPGA_REGS_H
#define _FPGA_REGS_H
/* =============================================================================
 * ��Ȩ�����������󾺼������˶ӣ�HITCRT��
 * �� �� ��: FPGA_REGS_2013.h
 * �������ڣ�2013/05/03 23:03:48
 * ģ�鹦�ܣ�ARM��FPGAͨ�ŵļĴ�����ַ����
 * ��ǰ�汾��1.0
 * �汾˵����1.0	2013/05/04 13:33:40
 *			 ȥ���˾ɰ����Ĵ���ע�ͣ������������2013�µװ�Ĵ��룬�������˳���
 * ��    ע��
 * ========================================================================== */
/*-----------------------------------------------------------------------------------------------------
	FPGA_2016�ĸ���ģ��
	<-0->	FPGA General	FPGA��ȫ�ֿ���
	<-1->	Buzzer			����������
	<-2->	Coder			����
	<-3->	Gyro			ģ����������������
	<-4->	Joystick		�ֱ�
	<-5->	Lcd12864		12864Һ��
	<-6->	PsKey			PS/2����
	<-7->	UART			FPGA����
-----------------------------------------------------------------------------------------------------*/

#include "stm32f4xx.h"


#define BANK1_SRAM3_ADDR    ((uint32_t)0x68000000)


/*--------------------------------------<-0-> FPGAȫ�ֿ���--------------------------------------------*/
#define	OFST_NRST_FPGA			0x40
#define	OFST_CE_FPGA			0x41
#define	FPGA_NRST				(*(volatile uint16_t*)(BANK1_SRAM3_ADDR | (OFST_NRST_FPGA << 10)))
#define	FPGA_CE					(*(volatile uint16_t*)(BANK1_SRAM3_ADDR | (OFST_CE_FPGA << 10)))

#define	OFST_TEST_WR			0x42
#define	OFST_TEST_RD			0x43
#define	FPGA_WR					(*(volatile uint16_t*)(BANK1_SRAM3_ADDR | (OFST_TEST_WR << 10)))
#define	FPGA_RD					(*(volatile uint16_t*)(BANK1_SRAM3_ADDR | (OFST_TEST_RD << 10)))
#define	INIT_FPGA()				FPGA_NRST = 0x000;\
								FPGA_NRST = 0xffff;\
								while(FPGA_NRST != 0xffff){FPGA_NRST = 0xffff;}while(FPGA_CE != 0xffff){FPGA_CE = 0xffff;}
/*-----------------------------------------------------------------------------------------------------*/

/*--------------------------------------<-1-> Buzzer ������--------------------------------------------*/
#define	OFST_BUZZER_MODE		0x081
#define	OFST_BUZZER_PERIOD		0x082
#define	OFST_BUZZER_CTRL		0x083

#define	REG_BUZZER_MODE_WO		(*(volatile uint16_t*)(BANK1_SRAM3_ADDR | (OFST_BUZZER_MODE) << 10))
#define	REG_BUZZER_PERIOD_WO	(*(volatile uint16_t*)(BANK1_SRAM3_ADDR | (OFST_BUZZER_PERIOD) << 10))
#define	REG_BUZZER_CTRL_RW		(*(volatile uint16_t*)(BANK1_SRAM3_ADDR | (OFST_BUZZER_CTRL) << 10))

//��ȡ�˼Ĵ����Ի�õ�ǰʣ�����������
#define	BUZZER_STATUS			REG_BUZZER_CTRL_RW

//�������������ڣ�Ĭ��Ϊ 1000 ms
#define	DEFAULT_BUZZER_PERIOD	1000

//�ڴ˶������������ģʽ(16-bits)
#define	BEEP_MODE_1				0xCCCC  
#define	BEEP_MODE_2				0xAA00
#define	BEEP_MODE_3				0xCC00
#define	BEEP_MODE_4				0xFF00

//ʹ����������ָ����ģʽ bm ����һ������
#define	BuzzerBeep(bm)	\
_BUZZER_BEEP(bm, DEFAULT_BUZZER_PERIOD, 0x0001)

//ʹ����������ָ����ģʽ bm ���� num ������
#define	BuzzerBeepBeep(bm, num)	\
_BUZZER_BEEP(bm, DEFAULT_BUZZER_PERIOD, num)

//ʹ����������ֹͣ����
#define	BuzzerShutup()	\
REG_BUZZER_CTRL_RW = 0x0000

#define _BUZZER_BEEP(bm, peri, num)	\
REG_BUZZER_CTRL_RW = 0x0000;	\
REG_BUZZER_MODE_WO = (uint16_t) bm;	\
REG_BUZZER_PERIOD_WO = (uint16_t) peri;	\
REG_BUZZER_CTRL_RW = (uint16_t) num
/*-----------------------------------------------------------------------------------------------------*/

/*-----------------------------------------<-2-> Coder ����--------------------------------------------*/
//�ɰ����Ϊ1-12ͨ�����°��������Ϊ1-6ͨ������������̲���ֵ
#define	OFST_CODER				0x00
#define	OFST_CODER_SEL      	0x1d 
#define	OFST_CODER_RST			0x1e  

#define	ADDR_CODE_L(CH)			(*(volatile uint16_t*)(BANK1_SRAM3_ADDR + ((OFST_CODER + CH) << 10)))
#define	ADDR_CODE_H(CH)			(*(volatile uint16_t*)(BANK1_SRAM3_ADDR + (((OFST_CODER + 16) + CH) << 10)))
#define	READ_CODER(CH)			(ADDR_CODE_L(CH) + (ADDR_CODE_H(CH) << 16))

#define	CODER_FPGA_SEL			(*((volatile uint16_t*)	(BANK1_SRAM3_ADDR + (OFST_CODER_SEL<< 10))))
#define	CODER_FPGA_RST			(*((volatile uint16_t*)	(BANK1_SRAM3_ADDR + (OFST_CODER_RST<< 10))))
#define	CODER_INIT()			CODER_FPGA_SEL = 0xffff; CODER_FPGA_RST = 0x0000; CODER_FPGA_RST = 0xffff
/*-----------------------------------------------------------------------------------------------------*/

/*------------------------------------------<-3-> Gyro ������ģ������----------------------------------*/
#define OFST_GYRO				0x58
#define OFST_GYRO_OFFSET        0x59

//��ȡ���ǻ����˽Ƕ�
#define GYRO_Q					(*((volatile int16_t*)(BANK1_SRAM3_ADDR + ((OFST_GYRO + 0)<< 10)))) //��ȥ��Ư
#define GYRO_OFFSET				(*((volatile int16_t*)(BANK1_SRAM3_ADDR + ((OFST_GYRO_OFFSET + 0)<< 10))))

//��������ݵĽ����ʶ�ȡ
#define	OFST_GYRO_VELT			0x5A
#define GYRO_VELT				(*((volatile int16_t*)(BANK1_SRAM3_ADDR + ((OFST_GYRO_VELT + 0)<< 10))))

//Mems MEMS����
//#define OFST_MEMS_GYRO			0x5C
//#define MEMS_GYRO_Q				(*((volatile int16_t*)(BANK1_SRAM3_ADDR + ((OFST_MEMS_GYRO + 0)<< 10))))
/*-----------------------------------------------------------------------------------------------------*/

/*-------------------------------------------<-4-> Joystick �ֱ�---------------------------------------*/
//���ֱ�Ϊ���ģʽʱJOYSTICK_STATEֵΪ0x73������Ϊ0x41
#define OFST_JOYSTICK			0x50
#define ADDR_JS_STATE			0x53
#define JOYSTICK_RESERVED 		(*((volatile uint16_t*)(BANK1_SRAM3_ADDR | ((OFST_JOYSTICK + 0)<< 10))))
#define JOYSTICK_LEFT 			(*((volatile uint16_t*)(BANK1_SRAM3_ADDR | ((OFST_JOYSTICK + 1)<< 10))))
#define JOYSTICK_RIGTH 			(*((volatile uint16_t*)(BANK1_SRAM3_ADDR | ((OFST_JOYSTICK + 2)<< 10))))
#define JOYSTICK_STATE 			(*((volatile uint16_t*)(BANK1_SRAM3_ADDR | ((OFST_JOYSTICK + 3)<< 10))))
/*-----------------------------------------------------------------------------------------------------*/


/*-------------------------------------------<-5-> Lcd12864 Һ��---------------------------------------*/
//LCD��һ�е�һ�е�ַ
#define	OFST_LCD				0x56
#define OFST_NRST_LCD   		0x57 
#define MAX_LEN_NUM				16		            //Һ����ʾ����ʱ����󳤶ȣ���λ���ֽڣ�
#define LCD_FPGA_CTRL			(*(volatile uint16_t*)(BANK1_SRAM3_ADDR | (OFST_LCD << 10)))
#define LCD_FPGA_NRST			(*(volatile uint16_t*)(BANK1_SRAM3_ADDR | (OFST_NRST_FPGA << 10)))
/*-----------------------------------------------------------------------------------------------------*/

/*--------------------------------------------<-6->	PsKey PS/2����-------------------------------------*/
//Ϊ��ɰ������ݣ����ص�˫�ֽڼ�ֵ��ֻȡ���ֽڣ����ֽ�����0xff             
#define OFST_PSKEY              0x54
#define PSKEY  					(*((volatile uint16_t*)(BANK1_SRAM3_ADDR | ((OFST_PSKEY + 0)<< 10))))
/*-----------------------------------------------------------------------------------------------------*/


/*-----------------------------------------------------------------------------------------------------*/


/*---------------------------------------------<-7-> UART FPGA����-------------------------------------*/
//UART1	���������
#define OFST_UART_RX1           0xB0
#define OFST_UART_TX1           0xB1
#define FPGA_UART_RX1 			(*((volatile uint16_t*)(BANK1_SRAM3_ADDR | ((OFST_UART_RX2+ 0)<< 10))))//����
#define FPGA_UART_TX1			(*((volatile uint16_t*)(BANK1_SRAM3_ADDR | ((OFST_UART_TX2+ 0)<< 10))))//����

//UART2	�����״�
#define	OFST_RADAR_L			0x5c
#define	OFST_RADAR_H			0x5d 

#define	ADDR_RADAR_L			(*(volatile uint16_t*)(BANK1_SRAM3_ADDR + ((OFST_RADAR_L << 10))))
#define	ADDR_RADAR_H			(*(volatile uint16_t*)(BANK1_SRAM3_ADDR + ((OFST_RADAR_H + 16) << 10)))
#define	READ_RADAR				((ADDR_RADAR_L + (ADDR_RADAR_H) << 16))

//UART3	�Ӿ�Ѳ��
#define	OFST_TRACLINE_L			0x42
#define	OFST_TRACLINE_H			0x43

#define	ADDR_TRACLINE_L			(*(volatile uint16_t*)(BANK1_SRAM3_ADDR + (OFST_TRACLINE_L << 10)))
#define	ADDR_TRACLINE_H			(*(volatile uint16_t*)(BANK1_SRAM3_ADDR + (OFST_TRACLINE_H << 10)))
#define	READ_TRACLINE			((ADDR_TRACLINE_L + (ADDR_TRACLINE_H) << 16))
/*-----------------------------------------------------------------------------------------------------*/


///*<-11->	FPGA����*/             
//#define OFST_UART_RX1             0xB0
//#define OFST_UART_TX1             0xB1
////#define OFST_UART_CTRL1         0xB2
//#define FPGA_UART_RX1 					(*((volatile uint16_t*)(BANK1_SRAM3_ADDR | ((OFST_UART_RX1+ 0)<< 10))))//����
//#define FPGA_UART_TX1					(*((volatile uint16_t*)(BANK1_SRAM3_ADDR | ((OFST_UART_TX1+ 0)<< 10))))//����
////#define FPGA_UART_CTRL1 				(*((volatile uint16_t*)(BANK1_SRAM3_ADDR | ((OFST_UART_CTRL1 + 0)<< 10))))//����
//
////�����������ݵĺ�
//#define FPGA_UART1_TX(data)	\
//FPGA_UART_CTRL1  = 0x0000;	\
//FPGA_UART_TX1= (uint16_t) data;	\
//FPGA_UART_CTRL1 = 0x0001
//#define OFST_UART_RX2             0xB2
//#define OFST_UART_TX2             0xB3
//#define FPGA_UART_RX2 					(*((volatile uint16_t*)(BANK1_SRAM3_ADDR | ((OFST_UART_RX2+ 0)<< 10))))//����
//#define FPGA_UART_TX2					(*((volatile uint16_t*)(BANK1_SRAM3_ADDR | ((OFST_UART_TX2+ 0)<< 10))))//����
//
//#define OFST_UART_RX3            0xB4
//#define OFST_UART_TX3            0xB5
//#define FPGA_UART_RX3 					(*((volatile uint16_t*)(BANK1_SRAM3_ADDR | ((OFST_UART_RX3+ 0)<< 10))))//����
//#define FPGA_UART_TX3					(*((volatile uint16_t*)(BANK1_SRAM3_ADDR | ((OFST_UART_TX3+ 0)<< 10))))//����

#endif

