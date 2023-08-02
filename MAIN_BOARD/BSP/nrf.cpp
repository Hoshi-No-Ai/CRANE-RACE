#include "nrf.h"
//////////////////////////////////////////////////////////////////////////////////
/*****************************************************************************
Copyright @HITCRT 2002-2017. All rights reserved.

File name:  SPI.cpp
Author:     pyx&Chris
Version:    1.0
Date:       2017.05.02
-------------------------------------------------------------------------------------
Description: 24l01&SPI1初始化，主机模式 ,不用无线手柄既可以关掉
Others:

--------------------------------------------------------------------------------------
History:
1. Date:
   Author:
   Modification:
***************************************************************************/
//////////////////////////////////////////////////////////////////////////////////

const uint8_t TX_ADDRESS[TX_ADR_WIDTH] = {0x34, 0x43, 0x10, 0x10, 0x01}; // 发送地址
const uint8_t RX_ADDRESS[RX_ADR_WIDTH] = {0x34, 0x43, 0x10, 0x10, 0x01}; // 发送地址

void NRF24L01_SPI_Init(void)
{

	SPI_InitTypeDef SPI_InitStructure;

	SPI_Cmd(SPI2, DISABLE); // 失能SPI外设

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;	 // 设置SPI单向或者双向的数据模式:SPI设置为双线双向全双工
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;						 // 设置SPI工作模式:设置为主SPI
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;					 // 设置SPI的数据大小:SPI发送接收8位帧结构
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;							 // 串行同步时钟的空闲状态为低电平
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;						 // 串行同步时钟的第1个跳变沿（上升或下降）数据被采样
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;							 // NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256; // 定义波特率预分频的值:波特率预分频值为256
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;					 // 指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始
	SPI_InitStructure.SPI_CRCPolynomial = 7;							 // CRC值计算的多项式
	SPI_Init(SPI2, &SPI_InitStructure);									 // 根据SPI_InitStruct中指定的参数初始化外设SPIx寄存器

	SPI_Cmd(SPI2, ENABLE); // 使能SPI外设
}

// 初始化24L01的IO口
void NRF24L01_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE); // 使能GPIOc

	// GPIOG6,7推挽输出
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;	   // 普通输出模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	   // 推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; // 100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	   // 上拉
	GPIO_Init(GPIOG, &GPIO_InitStructure);			   // 初始化PG6,7

	// GPIOG.8上拉输入
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN; // 输入
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; // 上拉
	GPIO_Init(GPIOG, &GPIO_InitStructure);		 // 初始化PG8

	SPI2_Configuration(); // 初始化SPI1

	NRF24L01_SPI_Init(); // 针对NRF的特点修改SPI的设置

	NRF24L01_CE = 0;  // 使能24L01
	NRF24L01_CSN = 1; // SPI片选取消
}

// 检测24L01是否存在
// 返回值:0，成功;1，失败
uint8_t NRF24L01_Check(void)
{
	uint8_t buf[5] = {0XA5, 0XA5, 0XA5, 0XA5, 0XA5};
	uint8_t i;
	SPI2_SetSpeed(SPI_BaudRatePrescaler_8);				 // spi速度为10.5Mhz（24L01的最大SPI时钟为10Mhz）
	NRF24L01_Write_Buf(NRF_WRITE_REG + TX_ADDR, buf, 5); // 写入5个字节的地址.
	NRF24L01_Read_Buf(TX_ADDR, buf, 5);					 // 读出写入的地址
	for (i = 0; i < 5; i++)
		if (buf[i] != 0XA5)
			break;
	if (i != 5)
		return 1; // 检测24L01错误
	return 0;	  // 检测到24L01
}

// SPI写寄存器
// reg:指定寄存器地址
// value:写入的值
uint8_t NRF24L01_Write_Reg(uint8_t reg, uint8_t value)
{
	uint8_t status;
	NRF24L01_CSN = 0;				  // 使能SPI传输
	status = SPI2_ReadWriteByte(reg); // 发送寄存器号
	SPI2_ReadWriteByte(value);		  // 写入寄存器的值
	NRF24L01_CSN = 1;				  // 禁止SPI传输
	return (status);				  // 返回状态值
}

// 读取SPI寄存器值
// reg:要读的寄存器
uint8_t NRF24L01_Read_Reg(uint8_t reg)
{
	uint8_t reg_val;
	NRF24L01_CSN = 0;					// 使能SPI传输
	SPI2_ReadWriteByte(reg);			// 发送寄存器号
	reg_val = SPI2_ReadWriteByte(0XFF); // 读取寄存器内容
	NRF24L01_CSN = 1;					// 禁止SPI传输
	return (reg_val);					// 返回状态值
}

// 在指定位置读出指定长度的数据
// reg:寄存器(位置)
//*pBuf:数据指针
// len:数据长度
// 返回值,此次读到的状态寄存器值
uint8_t NRF24L01_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
	uint8_t status, u8_ctr;
	NRF24L01_CSN = 0;				  // 使能SPI传输
	status = SPI2_ReadWriteByte(reg); // 发送寄存器值(位置),并读取状态值
	for (u8_ctr = 0; u8_ctr < len; u8_ctr++)
		pBuf[u8_ctr] = SPI2_ReadWriteByte(0XFF); // 读出数据
	NRF24L01_CSN = 1;							 // 关闭SPI传输
	return status;								 // 返回读到的状态值
}

// 在指定位置写指定长度的数据
// reg:寄存器(位置)
//*pBuf:数据指针
// len:数据长度
// 返回值,此次读到的状态寄存器值
uint8_t NRF24L01_Write_Buf(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
	uint8_t status, u8_ctr;
	NRF24L01_CSN = 0;				  // 使能SPI传输
	status = SPI2_ReadWriteByte(reg); // 发送寄存器值(位置),并读取状态值
	for (u8_ctr = 0; u8_ctr < len; u8_ctr++)
		SPI2_ReadWriteByte(*pBuf++); // 写入数据
	NRF24L01_CSN = 1;				 // 关闭SPI传输
	return status;					 // 返回读到的状态值
}

// 启动NRF24L01发送一次数据
// txbuf:待发送数据首地址
// 返回值:发送完成状况
uint8_t NRF24L01_TxPacket(uint8_t *txbuf)
{
	uint8_t sta;
	SPI2_SetSpeed(SPI_BaudRatePrescaler_8); // spi速度为10.5Mhz（24L01的最大SPI时钟为10Mhz）
	NRF24L01_CE = 0;
	NRF24L01_Write_Buf(WR_TX_PLOAD, txbuf, TX_PLOAD_WIDTH); // 写数据到TX BUF  32个字节
	NRF24L01_CE = 1;										// 启动发送
	while (NRF24L01_IRQ != 0)
		;											 // 等待发送完成
	sta = NRF24L01_Read_Reg(STATUS);				 // 读取状态寄存器的值
	NRF24L01_Write_Reg(NRF_WRITE_REG + STATUS, sta); // 清除TX_DS或MAX_RT中断标志
	if (sta & MAX_TX)								 // 达到最大重发次数
	{
		NRF24L01_Write_Reg(FLUSH_TX, 0xff); // 清除TX FIFO寄存器
		return MAX_TX;
	}
	if (sta & TX_OK) // 发送完成
	{
		return TX_OK;
	}
	return 0xff; // 其他原因发送失败
}

// 启动NRF24L01发送一次数据
// txbuf:待发送数据首地址
// 返回值:0，接收完成；其他，错误代码
uint8_t NRF24L01_RxPacket(uint8_t *rxbuf)
{
	uint8_t sta;
	SPI2_SetSpeed(SPI_BaudRatePrescaler_8);			 // spi速度为10.5Mhz（24L01的最大SPI时钟为10Mhz）
	sta = NRF24L01_Read_Reg(STATUS);				 // 读取状态寄存器的值
	NRF24L01_Write_Reg(NRF_WRITE_REG + STATUS, sta); // 清除TX_DS或MAX_RT中断标志
	if (sta & RX_OK)								 // 接收到数据
	{
		NRF24L01_Read_Buf(RD_RX_PLOAD, rxbuf, RX_PLOAD_WIDTH); // 读取数据
		NRF24L01_Write_Reg(FLUSH_RX, 0xff);					   // 清除RX FIFO寄存器
		L1 = 1;
		return 0;
	}
	L1 = 0;
	return 1; // 没收到任何数据
}

// 该函数初始化NRF24L01到RX模式
// 设置RX地址,写RX数据宽度,选择RF频道,波特率和LNA HCURR
// 当CE变高后,即进入RX模式,并可以接收数据了
void NRF24L01_RX_Mode(void)
{
	NRF24L01_CE = 0;
	NRF24L01_Write_Buf(NRF_WRITE_REG + RX_ADDR_P0, (uint8_t *)RX_ADDRESS, RX_ADR_WIDTH); // 写RX节点地址

	NRF24L01_Write_Reg(NRF_WRITE_REG + EN_AA, 0x01);			  // 使能通道0的自动应答
	NRF24L01_Write_Reg(NRF_WRITE_REG + EN_RXADDR, 0x01);		  // 使能通道0的接收地址
	NRF24L01_Write_Reg(NRF_WRITE_REG + RF_CH, 35);				  // 设置RF通信频率70	原10
	NRF24L01_Write_Reg(NRF_WRITE_REG + RX_PW_P0, RX_PLOAD_WIDTH); // 选择通道0的有效数据宽度
	NRF24L01_Write_Reg(NRF_WRITE_REG + RF_SETUP, 0x0f);			  // 设置TX发射参数,0db增益,2Mbps,低噪声增益开启
	NRF24L01_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0f);			  // 配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式
	NRF24L01_CE = 1;											  // CE为高,进入接收模式
}

// 该函数初始化NRF24L01到TX模式
// 设置TX地址,写TX数据宽度,设置RX自动应答的地址,填充TX发送数据,选择RF频道,波特率和LNA HCURR
// PWR_UP,CRC使能
// 当CE变高后,即进入RX模式,并可以接收数据了
// CE为高大于10us,则启动发送.
void NRF24L01_TX_Mode(void)
{
	NRF24L01_CE = 0;
	NRF24L01_Write_Buf(NRF_WRITE_REG + TX_ADDR, (uint8_t *)TX_ADDRESS, TX_ADR_WIDTH);	 // 写TX节点地址
	NRF24L01_Write_Buf(NRF_WRITE_REG + RX_ADDR_P0, (uint8_t *)RX_ADDRESS, RX_ADR_WIDTH); // 设置TX节点地址,主要为了使能ACK

	NRF24L01_Write_Reg(NRF_WRITE_REG + EN_AA, 0x01);	  // 使能通道0的自动应答
	NRF24L01_Write_Reg(NRF_WRITE_REG + EN_RXADDR, 0x01);  // 使能通道0的接收地址
	NRF24L01_Write_Reg(NRF_WRITE_REG + SETUP_RETR, 0xfa); // 设置自动重发间隔时间:500us + 86us;最大自动重发次数:10次
	NRF24L01_Write_Reg(NRF_WRITE_REG + RF_CH, 70);		  // 设置RF通道为70
	NRF24L01_Write_Reg(NRF_WRITE_REG + RF_SETUP, 0x0f);	  // 设置TX发射参数,0db增益,2Mbps,低噪声增益开启
	NRF24L01_Write_Reg(NRF_WRITE_REG + CONFIG, 0x0e);	  // 配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式,开启所有中断
	NRF24L01_CE = 1;									  // CE为高,10us后启动发送
}

/**************************************************************
函数名  :NRF24L01_Send_message
函数功能:通过CAN1发送数据至无线通讯模块，再传递给另一模块
输入: StdID ID号
	  data数据
输出：无
备注: 若接收消息后告知对方且希望对方能立即收到，需要延时10ms再发送
***************************************************************/
void NRF24L01_Send_message(uint32_t StdID, int16_t data1, int16_t data2, int16_t data3, int16_t data4)
{
	//	OS_ERR err;
	//	NormalSendByCAN_1(CAN_Tx_ID4,0,0,0,0);//将模块设置为发送模式
	//	NormalSendByCAN_1(StdID,data1,data2,data3,data4);
	//	NormalSendByCAN_1(StdID,data1,data2,data3,data4);
	//	NormalSendByCAN_1(StdID,data1,data2,data3,data4);//防止掉包，发送三次
	//	OSTimeDly_ms(50);
	//	NormalSendByCAN_1(CAN_Tx_ID4,1,0,1,1);//将模块设置为接收模式
}

/**************************************************************
函数名  :NRF24L01_Clear_message
函数功能:清零数据
输入: 无
输出：无
备注: 在使用完模块发送过来的数据后使用，每次使用前最好延时10ms
***************************************************************/
void NRF24L01_Clear_message(void)
{
	//	OS_ERR err;
	//	OSTimeDly_ms(60);//防止清零完后对面模块还在发送
	//	NormalSendByCAN_1(CAN_Tx_ID4,1,1,1,1);//将模块时刻发送给主控的数据清0一次
}