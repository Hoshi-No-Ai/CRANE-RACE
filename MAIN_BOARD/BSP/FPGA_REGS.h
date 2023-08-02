#ifndef _FPGA_REGS_H
#define _FPGA_REGS_H
/* =============================================================================
 * 版权声明：哈工大竞技机器人队（HITCRT）
 * 文 件 名: FPGA_REGS_2013.h
 * 创建日期：2013/05/03 23:03:48
 * 模块功能：ARM与FPGA通信的寄存器地址定义
 * 当前版本：1.0
 * 版本说明：1.0	2013/05/04 13:33:40
 *			 去除了旧版代码的错误注释，添加了适用于2013新底板的代码，并整理了程序
 * 备    注：
 * ========================================================================== */
/*-----------------------------------------------------------------------------------------------------
	FPGA_2016的各个模块
	<-0->	FPGA General	FPGA的全局控制
	<-1->	Buzzer			蜂鸣器外设
	<-2->	Coder			码盘
	<-3->	Gyro			模拟陀螺与数字陀螺
	<-4->	Joystick		手柄
	<-5->	Lcd12864		12864液晶
	<-6->	PsKey			PS/2键盘
	<-7->	UART			FPGA串口
-----------------------------------------------------------------------------------------------------*/

#include "stm32f4xx.h"

#define BANK1_SRAM3_ADDR ((uint32_t)0x68000000)

/*--------------------------------------<-0-> FPGA全局控制--------------------------------------------*/
#define OFST_NRST_FPGA 0x40
#define OFST_CE_FPGA 0x41
#define FPGA_NRST (*(volatile uint16_t *)(BANK1_SRAM3_ADDR | (OFST_NRST_FPGA << 10)))
#define FPGA_CE (*(volatile uint16_t *)(BANK1_SRAM3_ADDR | (OFST_CE_FPGA << 10)))

#define OFST_TEST_WR 0x42
#define OFST_TEST_RD 0x43
#define FPGA_WR (*(volatile uint16_t *)(BANK1_SRAM3_ADDR | (OFST_TEST_WR << 10)))
#define FPGA_RD (*(volatile uint16_t *)(BANK1_SRAM3_ADDR | (OFST_TEST_RD << 10)))
#define INIT_FPGA()             \
	FPGA_NRST = 0x000;          \
	FPGA_NRST = 0xffff;         \
	while (FPGA_NRST != 0xffff) \
	{                           \
		FPGA_NRST = 0xffff;     \
	}                           \
	while (FPGA_CE != 0xffff)   \
	{                           \
		FPGA_CE = 0xffff;       \
	}
/*-----------------------------------------------------------------------------------------------------*/

/*--------------------------------------<-1-> Buzzer 蜂鸣器--------------------------------------------*/
#define OFST_BUZZER_MODE 0x081
#define OFST_BUZZER_PERIOD 0x082
#define OFST_BUZZER_CTRL 0x083

#define REG_BUZZER_MODE_WO (*(volatile uint16_t *)(BANK1_SRAM3_ADDR | (OFST_BUZZER_MODE) << 10))
#define REG_BUZZER_PERIOD_WO (*(volatile uint16_t *)(BANK1_SRAM3_ADDR | (OFST_BUZZER_PERIOD) << 10))
#define REG_BUZZER_CTRL_RW (*(volatile uint16_t *)(BANK1_SRAM3_ADDR | (OFST_BUZZER_CTRL) << 10))

// 读取此寄存器以获得当前剩余蜂鸣周期数
#define BUZZER_STATUS REG_BUZZER_CTRL_RW

// 蜂鸣器鸣响周期，默认为 1000 ms
#define DEFAULT_BUZZER_PERIOD 1000

// 在此定义蜂鸣器鸣响模式(16-bits)
#define BEEP_MODE_1 0xCCCC
#define BEEP_MODE_2 0xAA00
#define BEEP_MODE_3 0xCC00
#define BEEP_MODE_4 0xFF00

// 使蜂鸣器按照指定的模式 bm 鸣响一个周期
#define BuzzerBeep(bm) \
	_BUZZER_BEEP(bm, DEFAULT_BUZZER_PERIOD, 0x0001)

// 使蜂鸣器按照指定的模式 bm 鸣响 num 个周期
#define BuzzerBeepBeep(bm, num) \
	_BUZZER_BEEP(bm, DEFAULT_BUZZER_PERIOD, num)

// 使蜂鸣器立即停止蜂鸣
#define BuzzerShutup() \
	REG_BUZZER_CTRL_RW = 0x0000

#define _BUZZER_BEEP(bm, peri, num)        \
	REG_BUZZER_CTRL_RW = 0x0000;           \
	REG_BUZZER_MODE_WO = (uint16_t)bm;     \
	REG_BUZZER_PERIOD_WO = (uint16_t)peri; \
	REG_BUZZER_CTRL_RW = (uint16_t)num
/*-----------------------------------------------------------------------------------------------------*/

/*-----------------------------------------<-2-> Coder 码盘--------------------------------------------*/
// 旧版程序为1-12通道，新版程序缩减为1-6通道，并添加码盘测速值
#define OFST_CODER 0x00
#define OFST_CODER_SEL 0x1d
#define OFST_CODER_RST 0x1e

#define ADDR_CODE_L(CH) (*(volatile uint16_t *)(BANK1_SRAM3_ADDR + ((OFST_CODER + CH) << 10)))
#define ADDR_CODE_H(CH) (*(volatile uint16_t *)(BANK1_SRAM3_ADDR + (((OFST_CODER + 16) + CH) << 10)))
#define READ_CODER(CH) (ADDR_CODE_L(CH) + (ADDR_CODE_H(CH) << 16))

#define CODER_FPGA_SEL (*((volatile uint16_t *)(BANK1_SRAM3_ADDR + (OFST_CODER_SEL << 10))))
#define CODER_FPGA_RST (*((volatile uint16_t *)(BANK1_SRAM3_ADDR + (OFST_CODER_RST << 10))))
#define CODER_INIT()         \
	CODER_FPGA_SEL = 0xffff; \
	CODER_FPGA_RST = 0x0000; \
	CODER_FPGA_RST = 0xffff
/*-----------------------------------------------------------------------------------------------------*/

/*------------------------------------------<-3-> Gyro 数字与模拟陀螺----------------------------------*/
#define OFST_GYRO 0x58
#define OFST_GYRO_OFFSET 0x59

// 读取的是机器人角度
#define GYRO_Q (*((volatile int16_t *)(BANK1_SRAM3_ADDR + ((OFST_GYRO + 0) << 10)))) // 已去零漂
#define GYRO_OFFSET (*((volatile int16_t *)(BANK1_SRAM3_ADDR + ((OFST_GYRO_OFFSET + 0) << 10))))

// 新添加陀螺的角速率读取
#define OFST_GYRO_VELT 0x5A
#define GYRO_VELT (*((volatile int16_t *)(BANK1_SRAM3_ADDR + ((OFST_GYRO_VELT + 0) << 10))))

// Mems MEMS陀螺
// #define OFST_MEMS_GYRO			0x5C
// #define MEMS_GYRO_Q				(*((volatile int16_t*)(BANK1_SRAM3_ADDR + ((OFST_MEMS_GYRO + 0)<< 10))))
/*-----------------------------------------------------------------------------------------------------*/

/*-------------------------------------------<-4-> Joystick 手柄---------------------------------------*/
// 当手柄为红灯模式时JOYSTICK_STATE值为0x73，否则为0x41
#define OFST_JOYSTICK 0x50
#define ADDR_JS_STATE 0x53
#define JOYSTICK_RESERVED (*((volatile uint16_t *)(BANK1_SRAM3_ADDR | ((OFST_JOYSTICK + 0) << 10))))
#define JOYSTICK_LEFT (*((volatile uint16_t *)(BANK1_SRAM3_ADDR | ((OFST_JOYSTICK + 1) << 10))))
#define JOYSTICK_RIGTH (*((volatile uint16_t *)(BANK1_SRAM3_ADDR | ((OFST_JOYSTICK + 2) << 10))))
#define JOYSTICK_STATE (*((volatile uint16_t *)(BANK1_SRAM3_ADDR | ((OFST_JOYSTICK + 3) << 10))))
/*-----------------------------------------------------------------------------------------------------*/

/*-------------------------------------------<-5-> Lcd12864 液晶---------------------------------------*/
// LCD第一行第一列地址
#define OFST_LCD 0x56
#define OFST_NRST_LCD 0x57
#define MAX_LEN_NUM 16 // 液晶显示数字时的最大长度（单位：字节）
#define LCD_FPGA_CTRL (*(volatile uint16_t *)(BANK1_SRAM3_ADDR | (OFST_LCD << 10)))
#define LCD_FPGA_NRST (*(volatile uint16_t *)(BANK1_SRAM3_ADDR | (OFST_NRST_FPGA << 10)))
/*-----------------------------------------------------------------------------------------------------*/

/*--------------------------------------------<-6->	PsKey PS/2键盘-------------------------------------*/
// 为与旧版程序兼容，返回的双字节键值仍只取低字节，高字节仍置0xff
#define OFST_PSKEY 0x54
#define PSKEY (*((volatile uint16_t *)(BANK1_SRAM3_ADDR | ((OFST_PSKEY + 0) << 10))))
/*-----------------------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------------------------------------------*/

/*---------------------------------------------<-7-> UART FPGA串口-------------------------------------*/
// UART1	超声波测距
#define OFST_UART_RX1 0xB0
#define OFST_UART_TX1 0xB1
#define FPGA_UART_RX1 (*((volatile uint16_t *)(BANK1_SRAM3_ADDR | ((OFST_UART_RX2 + 0) << 10)))) // 接收
#define FPGA_UART_TX1 (*((volatile uint16_t *)(BANK1_SRAM3_ADDR | ((OFST_UART_TX2 + 0) << 10)))) // 发送

// UART2	激光雷达
#define OFST_RADAR_L 0x5c
#define OFST_RADAR_H 0x5d

#define ADDR_RADAR_L (*(volatile uint16_t *)(BANK1_SRAM3_ADDR + ((OFST_RADAR_L << 10))))
#define ADDR_RADAR_H (*(volatile uint16_t *)(BANK1_SRAM3_ADDR + ((OFST_RADAR_H + 16) << 10)))
#define READ_RADAR ((ADDR_RADAR_L + (ADDR_RADAR_H) << 16))

// UART3	视觉巡线
#define OFST_TRACLINE_L 0x42
#define OFST_TRACLINE_H 0x43

#define ADDR_TRACLINE_L (*(volatile uint16_t *)(BANK1_SRAM3_ADDR + (OFST_TRACLINE_L << 10)))
#define ADDR_TRACLINE_H (*(volatile uint16_t *)(BANK1_SRAM3_ADDR + (OFST_TRACLINE_H << 10)))
#define READ_TRACLINE ((ADDR_TRACLINE_L + (ADDR_TRACLINE_H) << 16))
/*-----------------------------------------------------------------------------------------------------*/

///*<-11->	FPGA串口*/
// #define OFST_UART_RX1             0xB0
// #define OFST_UART_TX1             0xB1
////#define OFST_UART_CTRL1         0xB2
// #define FPGA_UART_RX1 					(*((volatile uint16_t*)(BANK1_SRAM3_ADDR | ((OFST_UART_RX1+ 0)<< 10))))//接收
// #define FPGA_UART_TX1					(*((volatile uint16_t*)(BANK1_SRAM3_ADDR | ((OFST_UART_TX1+ 0)<< 10))))//发送
////#define FPGA_UART_CTRL1 				(*((volatile uint16_t*)(BANK1_SRAM3_ADDR | ((OFST_UART_CTRL1 + 0)<< 10))))//发送
//
////声明发送数据的宏
// #define FPGA_UART1_TX(data)	\
//FPGA_UART_CTRL1  = 0x0000;	\
//FPGA_UART_TX1= (uint16_t) data;	\
//FPGA_UART_CTRL1 = 0x0001
// #define OFST_UART_RX2             0xB2
// #define OFST_UART_TX2             0xB3
// #define FPGA_UART_RX2 					(*((volatile uint16_t*)(BANK1_SRAM3_ADDR | ((OFST_UART_RX2+ 0)<< 10))))//接收
// #define FPGA_UART_TX2					(*((volatile uint16_t*)(BANK1_SRAM3_ADDR | ((OFST_UART_TX2+ 0)<< 10))))//发送
//
// #define OFST_UART_RX3            0xB4
// #define OFST_UART_TX3            0xB5
// #define FPGA_UART_RX3 					(*((volatile uint16_t*)(BANK1_SRAM3_ADDR | ((OFST_UART_RX3+ 0)<< 10))))//接收
// #define FPGA_UART_TX3					(*((volatile uint16_t*)(BANK1_SRAM3_ADDR | ((OFST_UART_TX3+ 0)<< 10))))//发送

#endif
