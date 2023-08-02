#ifndef __SPI_H__
#define __SPI_H__

#include "stm32f4xx_spi.h"
#include "gimbal_control_types.h"

// PC7 	CSB1
// PB12 	CSB2
// PC9	PS（固定为GND）

#define Open_SPIx_CS_PIN GPIO_Pin_1
#define Open_SPIx_CS_PORT GPIOG
#define Open_SPIx_CS_GPIO_CLK RCC_AHB1Periph_GPIOG

#define Open_SPIx SPI2
#define Open_SPIx_CLK RCC_APB1Periph_SPI2
#define Open_SPIx_CLK_INIT RCC_APB1PeriphClockCmd
#define Open_SPIx_IRQn SPI2_IRQn
#define Open_SPIx_IRQHANDLER SPI2_IRQHandler

#define Open_SPIx_SCK_PIN GPIO_Pin_13
#define Open_SPIx_SCK_GPIO_PORT GPIOB
#define Open_SPIx_SCK_GPIO_CLK RCC_AHB1Periph_GPIOB
#define Open_SPIx_SCK_SOURCE GPIO_PinSource13
#define Open_SPIx_SCK_AF GPIO_AF_SPI2

#define Open_SPIx_MISO_PIN GPIO_Pin_14
#define Open_SPIx_MISO_GPIO_PORT GPIOB
#define Open_SPIx_MISO_GPIO_CLK RCC_AHB1Periph_GPIOB
#define Open_SPIx_MISO_SOURCE GPIO_PinSource14
#define Open_SPIx_MISO_AF GPIO_AF_SPI2

#define Open_SPIx_MOSI_PIN GPIO_Pin_15
#define Open_SPIx_MOSI_GPIO_PORT GPIOB
#define Open_SPIx_MOSI_GPIO_CLK RCC_AHB1Periph_GPIOB
#define Open_SPIx_MOSI_SOURCE GPIO_PinSource15
#define Open_SPIx_MOSI_AF GPIO_AF_SPI2

#define CS PGout(1) // DS0

#define CS_H GPIO_SetBits(Open_SPIx_CS_PORT, Open_SPIx_CS_PIN)
#define CS_L GPIO_ResetBits(Open_SPIx_CS_PORT, Open_SPIx_CS_PIN)

// #define  GYRO_CS_H    GPIO_SetBits(Open_SPIx_CSB2_PORT, Open_SPIx_CSB2_PIN)//GPIOB   GPIO_Pin_12
// #define  GYRO_CS_L    GPIO_ResetBits(Open_SPIx_CSB2_PORT, Open_SPIx_CSB2_PIN)//GPIOB   GPIO_Pin_12

// #define  ACC_CS_H    GPIO_SetBits(Open_SPIx_CSB1_PORT, Open_SPIx_CSB1_PIN)//GPIOC  GPIO_Pin_8
// #define  ACC_CS_L    GPIO_ResetBits(Open_SPIx_CSB1_PORT, Open_SPIx_CSB1_PIN)//GPIOC  GPIO_Pin_8

void SPI1_Init(void);                       // 初始化SPI1口
void SPI1_SetSpeed(uint8_t SpeedSet);       // 设置SPI1速度
uint8_t SPI1_ReadWriteByte(uint8_t TxData); // SPI1总线读写一个字节

// void SPI2_Configuration(void);
// USHORT16 spi2_read_write_byte(USHORT16 txc);

void SPI2_SetSpeed(u8 SPI_BaudRatePrescaler);
u8 SPI2_ReadWriteByte(u8 TxData);
void SPI2_Configuration(void);

#endif
