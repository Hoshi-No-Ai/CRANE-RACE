#include "spi.h"

/*****************************************************************************
Copyright @HITCRT 2002-2017. All rights reserved.

File name:  SPI.cpp
Author:     pyx&Chris
Version:    1.0
Date:       2017.05.02
-------------------------------------------------------------------------------------
Description: SPI1初始化，主机模式 ,不用无线手柄既可以关掉
Others:

--------------------------------------------------------------------------------------
History:
1. Date:
   Author:
   Modification:
***************************************************************************/
// SPI1初始化，主机模式

void SPI1_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef SPI_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); // 使能GPIOA时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);  // 使能SPI1时钟

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7; // Pa5~7复用功能输出
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;						// 复用功能
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;						// 推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;					// 100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;						// 上拉
	GPIO_Init(GPIOA, &GPIO_InitStructure);								// 初始化

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1); // PB3复用为 SPI1
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1); // PB4复用为 SPI1
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1); // PB5复用为 SPI1

	// 这里只针对SPI口初始化
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1, ENABLE);  // 复位SPI1
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1, DISABLE); // 停止复位SPI1

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;	 // 设置SPI单向或者双向的数据模式:SPI设置为双线双向全双工
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;						 // 设置SPI工作模式:设置为主SPI
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;					 // 设置SPI的数据大小:SPI发送接收8位帧结构
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;							 // 串行同步时钟的空闲状态为高电平
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;						 // 串行同步时钟的第二个跳变沿（上升或下降）数据被采样
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;							 // NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256; // 定义波特率预分频的值:波特率预分频值为256
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;					 // 指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始
	SPI_InitStructure.SPI_CRCPolynomial = 7;							 // CRC值计算的多项式
	SPI_Init(SPI1, &SPI_InitStructure);									 // 根据SPI_InitStruct中指定的参数初始化外设SPIx寄存器

	SPI_Cmd(SPI1, ENABLE); // 使能SPI外设

	SPI1_ReadWriteByte(0xff); // 启动传输
}

// SPI1速度设置函数
// SPI速度=fAPB2/分频系数
//@ref SPI_BaudRate_Prescaler:SPI_BaudRatePrescaler_2~SPI_BaudRatePrescaler_256
// fAPB2时钟一般为84Mhz：
void SPI1_SetSpeed(u8 SPI_BaudRatePrescaler)
{
	assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_BaudRatePrescaler)); // 判断有效性
	SPI1->CR1 &= 0XFFC7;											// 位3-5清零，用来设置波特率
	SPI1->CR1 |= SPI_BaudRatePrescaler;								// 设置SPI1速度
	SPI_Cmd(SPI1, ENABLE);											// 使能SPI1
}

// SPI1 读写一个字节
// TxData:要写入的字节
// 返回值:读取到的字节
u8 SPI1_ReadWriteByte(u8 TxData)
{

	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET)
	{
	} // 等待发送区空

	SPI_I2S_SendData(SPI1, TxData); // 通过外设SPIx发送一个byte  数据

	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET)
	{
	} // 等待接收完一个byte

	return SPI_I2S_ReceiveData(SPI1); // 返回通过SPIx最近接收的数据
}

// void SPI2_Configuration(void)
//{
//   GPIO_InitTypeDef  GPIO_InitStructure;
//   SPI_InitTypeDef  SPI_InitStructure;
//
//   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//使能GPIOB时钟
////  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);//使能GPIOC时钟
//  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);//使能SPI2时钟
//
//  //GPIOFB3,4,5初始化设置
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
//  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
//  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
//  GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化
//
//	GPIO_PinAFConfig(GPIOB,GPIO_PinSource13,GPIO_AF_SPI2); //PB3复用为 SPI1
//	GPIO_PinAFConfig(GPIOB,GPIO_PinSource14,GPIO_AF_SPI2); //PB4复用为 SPI1
//	GPIO_PinAFConfig(GPIOB,GPIO_PinSource15,GPIO_AF_SPI2); //PB5复用为 SPI1
//
//	//这里只针对SPI口初始化
//	RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI2,ENABLE);//复位SPI1
//	RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI2,DISABLE);//停止复位SPI1

//	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //设置SPI单向或者双向的数据模式:SPI设置为双线双向全双工
//	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//设置SPI工作模式:设置为主SPI
//	SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;		//设置SPI的数据大小:SPI发送接收16位帧结构
//	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;		//串行同步时钟的空闲状态为高电平
//	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;	//串行同步时钟的第二个跳变沿（上升或下降）数据被采样
//	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		//NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制
//	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128;		//定义波特率预分频的值:波特率预分频值为256
//	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始
//	SPI_InitStructure.SPI_CRCPolynomial = 7;	//CRC值计算的多项式
//	SPI_Init(SPI2, &SPI_InitStructure);  //根据SPI_InitStruct中指定的参数初始化外设SPIx寄存器
//
//	SPI_Cmd(SPI2, ENABLE); //使能SPI外设

//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);//使能GPIOF时钟

//	  //GPIOF9,F10初始化设置
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
//	GPIO_Init(GPIOG, &GPIO_InitStructure);//初始化
//
//	GPIO_SetBits(GPIOG,GPIO_Pin_1);
//}

// USHORT16 spi2_read_write_byte(USHORT16 txc)
//{
//     while((SPI2->SR&SPI_SR_TXE)==0);    //等待发送结束
//     SPI2->DR = txc;
//     while((SPI2->SR&SPI_SR_RXNE)==0);   //等待接收结束
//     return SPI2->DR;
// }

void SPI2_SetSpeed(u8 SPI_BaudRatePrescaler)
{
	assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_BaudRatePrescaler)); // 判断有效性
	SPI2->CR1 &= 0XFFC7;											// 位3-5清零，用来设置波特率
	SPI2->CR1 |= SPI_BaudRatePrescaler;								// 设置SPI1速度
	SPI_Cmd(SPI2, ENABLE);											// 使能SPI1
}

// SPI1 读写一个字节
// TxData:要写入的字节
// 返回值:读取到的字节
u8 SPI2_ReadWriteByte(u8 TxData)
{

	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET)
	{
	} // 等待发送区空

	SPI_I2S_SendData(SPI2, TxData); // 通过外设SPIx发送一个byte  数据

	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET)
	{
	} // 等待接收完一个byte

	return SPI_I2S_ReceiveData(SPI2); // 返回通过SPIx最近接收的数据
}

void SPI2_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef SPI_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); // 使能GPIOB时钟
														  //   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);//使能GPIOC时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE); // 使能SPI2时钟

	// GPIOFB3,4,5初始化设置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	   // 复用功能
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	   // 推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; // 100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	   // 上拉
	GPIO_Init(GPIOB, &GPIO_InitStructure);			   // 初始化

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_SPI2); // PB3复用为 SPI1
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_SPI2); // PB4复用为 SPI1
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_SPI2); // PB5复用为 SPI1

	// 这里只针对SPI口初始化
	//	RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI2,ENABLE);//复位SPI1
	//	RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI2,DISABLE);//停止复位SPI1

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;	 // 设置SPI单向或者双向的数据模式:SPI设置为双线双向全双工
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;						 // 设置SPI工作模式:设置为主SPI
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;					 // 设置SPI的数据大小:SPI发送接收16位帧结构
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;							 // 串行同步时钟的空闲状态为高电平
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;						 // 串行同步时钟的第二个跳变沿（上升或下降）数据被采样
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;							 // NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256; // 定义波特率预分频的值:波特率预分频值为256
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;					 // 指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始
	SPI_InitStructure.SPI_CRCPolynomial = 7;							 // CRC值计算的多项式
	SPI_Init(SPI2, &SPI_InitStructure);									 // 根据SPI_InitStruct中指定的参数初始化外设SPIx寄存器

	SPI_Cmd(SPI2, ENABLE);	  // 使能SPI外设
	SPI2_ReadWriteByte(0xff); // 启动传输
}
