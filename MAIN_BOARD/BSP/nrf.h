#ifndef __nrf_H__
#define __nrf_H__

#include "stm32f4xx.h"
#include "spi.h"
#include "led.h"

//////////////////////////////////////////////////////////////////////////////////
// 位带操作,实现51类似的GPIO控制功能
// 具体实现思想,参考<<CM3权威指南>>第五章(87页~92页).M4同M3类似,只是寄存器地址变了.
// IO口操作宏定义
#define BITBAND(addr, bitnum) ((addr & 0xF0000000) + 0x2000000 + ((addr & 0xFFFFF) << 5) + (bitnum << 2))
#define MEM_ADDR(addr) *((volatile unsigned long *)(addr))
#define BIT_ADDR(addr, bitnum) MEM_ADDR(BITBAND(addr, bitnum))
// IO口地址映射
#define GPIOA_ODR_Addr (GPIOA_BASE + 20) // 0x40020014
#define GPIOB_ODR_Addr (GPIOB_BASE + 20) // 0x40020414
#define GPIOC_ODR_Addr (GPIOC_BASE + 20) // 0x40020814
#define GPIOD_ODR_Addr (GPIOD_BASE + 20) // 0x40020C14
#define GPIOE_ODR_Addr (GPIOE_BASE + 20) // 0x40021014
#define GPIOF_ODR_Addr (GPIOF_BASE + 20) // 0x40021414
#define GPIOG_ODR_Addr (GPIOG_BASE + 20) // 0x40021814
#define GPIOH_ODR_Addr (GPIOH_BASE + 20) // 0x40021C14
#define GPIOI_ODR_Addr (GPIOI_BASE + 20) // 0x40022014

#define GPIOA_IDR_Addr (GPIOA_BASE + 16) // 0x40020010
#define GPIOB_IDR_Addr (GPIOB_BASE + 16) // 0x40020410
#define GPIOC_IDR_Addr (GPIOC_BASE + 16) // 0x40020810
#define GPIOD_IDR_Addr (GPIOD_BASE + 16) // 0x40020C10
#define GPIOE_IDR_Addr (GPIOE_BASE + 16) // 0x40021010
#define GPIOF_IDR_Addr (GPIOF_BASE + 16) // 0x40021410
#define GPIOG_IDR_Addr (GPIOG_BASE + 16) // 0x40021810
#define GPIOH_IDR_Addr (GPIOH_BASE + 16) // 0x40021C10
#define GPIOI_IDR_Addr (GPIOI_BASE + 16) // 0x40022010

// IO口操作,只对单一的IO口!
// 确保n的值小于16!
#define PAout(n) BIT_ADDR(GPIOA_ODR_Addr, n) // 输出
#define PAin(n) BIT_ADDR(GPIOA_IDR_Addr, n)  // 输入

#define PBout(n) BIT_ADDR(GPIOB_ODR_Addr, n) // 输出
#define PBin(n) BIT_ADDR(GPIOB_IDR_Addr, n)  // 输入

#define PCout(n) BIT_ADDR(GPIOC_ODR_Addr, n) // 输出
#define PCin(n) BIT_ADDR(GPIOC_IDR_Addr, n)  // 输入

#define PDout(n) BIT_ADDR(GPIOD_ODR_Addr, n) // 输出
#define PDin(n) BIT_ADDR(GPIOD_IDR_Addr, n)  // 输入

#define PEout(n) BIT_ADDR(GPIOE_ODR_Addr, n) // 输出
#define PEin(n) BIT_ADDR(GPIOE_IDR_Addr, n)  // 输入

#define PFout(n) BIT_ADDR(GPIOF_ODR_Addr, n) // 输出
#define PFin(n) BIT_ADDR(GPIOF_IDR_Addr, n)  // 输入

#define PGout(n) BIT_ADDR(GPIOG_ODR_Addr, n) // 输出
#define PGin(n) BIT_ADDR(GPIOG_IDR_Addr, n)  // 输入

#define PHout(n) BIT_ADDR(GPIOH_ODR_Addr, n) // 输出
#define PHin(n) BIT_ADDR(GPIOH_IDR_Addr, n)  // 输入

#define PIout(n) BIT_ADDR(GPIOI_ODR_Addr, n) // 输出
#define PIin(n) BIT_ADDR(GPIOI_IDR_Addr, n)  // 输入

//////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////////////
// NRF24L01寄存器操作命令
#define NRF_READ_REG 0x00  // 读配置寄存器,低5位为寄存器地址
#define NRF_WRITE_REG 0x20 // 写配置寄存器,低5位为寄存器地址
#define RD_RX_PLOAD 0x61   // 读RX有效数据,1~32字节
#define WR_TX_PLOAD 0xA0   // 写TX有效数据,1~32字节
#define FLUSH_TX 0xE1      // 清除TX FIFO寄存器.发射模式下用
#define FLUSH_RX 0xE2      // 清除RX FIFO寄存器.接收模式下用
#define REUSE_TX_PL 0xE3   // 重新使用上一包数据,CE为高,数据包被不断发送.
#define NOP 0xFF           // 空操作,可以用来读状态寄存器
// SPI(NRF24L01)寄存器地址
#define CONFIG 0x00 // 配置寄存器地址;bit0:1接收模式,0发射模式;bit1:电选择;bit2:CRC模式;bit3:CRC使能;
                    // bit4:中断MAX_RT(达到最大重发次数中断)使能;bit5:中断TX_DS使能;bit6:中断RX_DR使能
#define EN_AA 0x01      // 使能自动应答功能  bit0~5,对应通道0~5
#define EN_RXADDR 0x02  // 接收地址允许,bit0~5,对应通道0~5
#define SETUP_AW 0x03   // 设置地址宽度(所有数据通道):bit1,0:00,3字节;01,4字节;02,5字节;
#define SETUP_RETR 0x04 // 建立自动重发;bit3:0,自动重发计数器;bit7:4,自动重发延时 250*x+86us
#define RF_CH 0x05      // RF通道,bit6:0,工作通道频率;
#define RF_SETUP 0x06   // RF寄存器;bit3:传输速率(0:1Mbps,1:2Mbps);bit2:1,发射功率;bit0:低噪声放大器增益
#define STATUS 0x07     // 状态寄存器;bit0:TX FIFO满标志;bit3:1,接收数据通道号(最大:6);bit4,达到最多次重发
                    // bit5:数据发送完成中断;bit6:接收数据中断;
#define MAX_TX 0x10 // 达到最大发送次数中断
#define TX_OK 0x20  // TX发送完成中断
#define RX_OK 0x40  // 接收到数据中断

#define OBSERVE_TX 0x08      // 发送检测寄存器,bit7:4,数据包丢失计数器;bit3:0,重发计数器
#define CD 0x09              // 载波检测寄存器,bit0,载波检测;
#define RX_ADDR_P0 0x0A      // 数据通道0接收地址,最大长度5个字节,低字节在前
#define RX_ADDR_P1 0x0B      // 数据通道1接收地址,最大长度5个字节,低字节在前
#define RX_ADDR_P2 0x0C      // 数据通道2接收地址,最低字节可设置,高字节,必须同RX_ADDR_P1[39:8]相等;
#define RX_ADDR_P3 0x0D      // 数据通道3接收地址,最低字节可设置,高字节,必须同RX_ADDR_P1[39:8]相等;
#define RX_ADDR_P4 0x0E      // 数据通道4接收地址,最低字节可设置,高字节,必须同RX_ADDR_P1[39:8]相等;
#define RX_ADDR_P5 0x0F      // 数据通道5接收地址,最低字节可设置,高字节,必须同RX_ADDR_P1[39:8]相等;
#define TX_ADDR 0x10         // 发送地址(低字节在前),ShockBurstTM模式下,RX_ADDR_P0与此地址相等
#define RX_PW_P0 0x11        // 接收数据通道0有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P1 0x12        // 接收数据通道1有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P2 0x13        // 接收数据通道2有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P3 0x14        // 接收数据通道3有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P4 0x15        // 接收数据通道4有效数据宽度(1~32字节),设置为0则非法
#define RX_PW_P5 0x16        // 接收数据通道5有效数据宽度(1~32字节),设置为0则非法
#define NRF_FIFO_STATUS 0x17 // FIFO状态寄存器;bit0,RX FIFO寄存器空标志;bit1,RX FIFO满标志;bit2,3,保留
                             // bit4,TX FIFO空标志;bit5,TX FIFO满标志;bit6,1,循环发送上一数据包.0,不循环;
//////////////////////////////////////////////////////////////////////////////////////////////////////////
// 24L01操作线
// #define NRF24L01_CE   PGout(6) 	//24L01片选信号
// #define NRF24L01_CSN  PGout(7) 	//SPI片选信号
// #define NRF24L01_IRQ  PGin(8)  	//IRQ主机数据输入
#define NRF24L01_CE PGout(0)  // 24L01片选信号
#define NRF24L01_CSN PGout(1) // SPI片选信号
#define NRF24L01_IRQ PGout(2) // IRQ主机数据输入

// 24L01发送接收数据宽度定义
#define TX_ADR_WIDTH 5    // 5字节的地址宽度
#define RX_ADR_WIDTH 5    // 5字节的地址宽度
#define TX_PLOAD_WIDTH 32 // 32字节的用户数据宽度
#define RX_PLOAD_WIDTH 32 // 32字节的用户数据宽度

/*两车通信所用ID*/
#define CAN_Rx_ID1 0x1F0
#define CAN_Rx_ID2 0x1F1
#define CAN_Rx_ID3 0x1F2
#define CAN_Rx_ID4 0x1F3
#define CAN_Tx_ID1 0x1F4
#define CAN_Tx_ID2 0x1F5
#define CAN_Tx_ID3 0x1F6
#define CAN_Tx_ID4 0x1F7

#define L1 PAout(10)

void NRF24L01_Init(void);                                                                               // 初始化
void NRF24L01_RX_Mode(void);                                                                            // 配置为接收模式
void NRF24L01_TX_Mode(void);                                                                            // 配置为发送模式
void NRF24L01_Send_message(uint32_t StdID, int16_t data1, int16_t data2, int16_t data3, int16_t data4); // 无线模块发送数据
void NRF24L01_Clear_message(void);                                                                      // 清零无线模块与主控通讯的数据
uint8_t NRF24L01_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t u8s);                                    // 写数据区
uint8_t NRF24L01_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t u8s);                                     // 读数据区
uint8_t NRF24L01_Read_Reg(uint8_t reg);                                                                 // 读寄存器
uint8_t NRF24L01_Write_Reg(uint8_t reg, uint8_t value);                                                 // 写寄存器
uint8_t NRF24L01_Check(void);                                                                           // 检查24L01是否存在
uint8_t NRF24L01_TxPacket(uint8_t *txbuf);                                                              // 发送一个包的数据
uint8_t NRF24L01_RxPacket(uint8_t *rxbuf);                                                              // 接收一个包的数据

#endif