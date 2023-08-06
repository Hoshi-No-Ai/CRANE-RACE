#include "SPI.h"	  

//以下是SPI2模块的初始化代码，配置成主机模式，访问SD Card/W25X16/24L01/JF24C							  
//SPI2口初始化
//这里针是对SPI2的初始化
SPI_InitTypeDef  SPI2_InitStructure;
void SPI2_Init(void)
{
	
	GPIO_InitTypeDef GPIO_InitStructure;
 /****Initial SPI2******************/
 
 /* Enable SPI2 and GPIOB clocks */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
 
 

 /* Configure SPI2 pins: NSS, SCK, MISO and MOSI */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
 
 //SPI2 NSS 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
 
	GPIO_SetBits(GPIOB, GPIO_Pin_12);
 
  /* SPI2 configuration */ 
	SPI2_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; //SPI1设置为两线全双工
	SPI2_InitStructure.SPI_Mode = SPI_Mode_Master;                    //设置SPI2为主模式
	SPI2_InitStructure.SPI_DataSize = SPI_DataSize_8b;                  //SPI发送接收8位帧结构
	SPI2_InitStructure.SPI_CPOL = SPI_CPOL_Low;                   //串行时钟在不操作时，时钟为低电平
	SPI2_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;                 //第一个时钟沿开始采样数据
	SPI2_InitStructure.SPI_NSS = SPI_NSS_Soft;                  //NSS信号由软件（使用SSI位）管理
	SPI2_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256; //定义波特率预分频的值:波特率预分频值为8
	SPI2_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;       //数据传输从MSB位开始
	SPI2_InitStructure.SPI_CRCPolynomial = 7;         //CRC值计算的多项式
	SPI_Init(SPI2, &SPI2_InitStructure);
 /* Enable SPI2  */
	SPI_Cmd(SPI2, ENABLE);  
} 
//SPI 速度设置函数
//SpeedSet:
//SPI_BaudRatePrescaler_2   2分频   (SPI 36M@sys 72M)
//SPI_BaudRatePrescaler_8   8分频   (SPI 9M@sys 72M)
//SPI_BaudRatePrescaler_16  16分频  (SPI 4.5M@sys 72M)
//SPI_BaudRatePrescaler_256 256分频 (SPI 281.25K@sys 72M)
  
void SPI2_SetSpeed(u8 SpeedSet)
{
	SPI2_InitStructure.SPI_BaudRatePrescaler = SpeedSet ;
  	SPI_Init(SPI2, &SPI2_InitStructure);
	SPI_Cmd(SPI2,ENABLE);
} 

//SPIx 读写一个字节
//TxData:要写入的字节
//返回值:读取到的字节
u8 SPI2_ReadWriteByte(u8 TxData)
{		
//	u8 retry=0;				 	
//	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET) //检查指定的SPI标志位设置与否:发送缓存空标志位
//		{
////		retry++;
////		if(retry>200)return 0;
//		}			  
//	SPI_I2S_SendData(SPI2, TxData); //通过外设SPIx发送一个数据
//	retry=0;
//	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET)//检查指定的SPI标志位设置与否:接受缓存非空标志位
//		{
////		retry++;
////		if(retry>200)return 0;
//		}	  						    
//	return SPI_I2S_ReceiveData(SPI2); //返回通过SPIx最近接收的数据
  unsigned char RxData=0;

  while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_TXE)==RESET); //                                                   
  SPI_I2S_SendData(SPI2,TxData);

   while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_RXNE)==RESET);

   RxData=SPI_I2S_ReceiveData(SPI2);

    return RxData;
  return RxData;
  
}































