#include "STM32Lib\\stm32f10x.h"
#include "main.h"

void ADS1256_GPIO_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_ADS1256Reset | RCC_ADS1256DRDY, ENABLE); 

  	GPIO_InitStructure.GPIO_Pin = GPIO_RCC_ADS1256Reset; 
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  
  	GPIO_Init(GPIO_RCC_ADS1256Reset_PORT, &GPIO_InitStructure);  
  	GPIO_ResetBits(GPIO_RCC_ADS1256Reset_PORT, GPIO_RCC_ADS1256Reset );


	GPIO_InitStructure.GPIO_Pin = GPIO_ADS1256DRDY; 
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  
  	GPIO_Init(GPIO_ADS1256DRDY_PORT, &GPIO_InitStructure);  
	
	
}
void ADS1256_Init(void)
{
//	OS_ERR err;
	GPIO_ResetBits(GPIOB, GPIO_Pin_12);
//	SPI_SendByte(ADS1256_CMD_REST);
	GPIO_ResetBits(GPIOB, GPIO_Pin_11);
	SysTickDelayms(100);
	GPIO_SetBits(GPIOB, GPIO_Pin_11);
	SysTickDelayms(10);//10
    while(GPIO_ReadInputDataBit(GPIO_ADS1256DRDY_PORT,GPIO_ADS1256DRDY));
	SPI2_ReadWriteByte(ADS1256_CMD_SYNC);                 //同步命令
	SPI2_ReadWriteByte(ADS1256_CMD_WAKEUP);               //同步唤醒	
	while(GPIO_ReadInputDataBit(GPIO_ADS1256DRDY_PORT,GPIO_ADS1256DRDY));
	SPI2_ReadWriteByte(ADS1256_CMD_WREG | ADS1256_STATUS);//连续写入4个寄存器
	SPI2_ReadWriteByte(3);
	SPI2_ReadWriteByte(0x06);                             //高位在前，使用内部校准，使用缓存
	SPI2_ReadWriteByte(ADS1256_MUXP_AIN0 | ADS1256_MUXN_AINCOM);//端口输入A2为正，A3位负
	SPI2_ReadWriteByte(ADS1256_GAIN_1);                   //放大倍数设置
//	SPI2_ReadWriteByte(ADS1256_DRATE_30000SPS);//30000       //采集速度设置
	SPI2_ReadWriteByte(ADS1256_DRATE_7500SPS);//ADS1256_DRATE_7500SPS
	ADS1256WREG(ADS1256_IO,0x00);
	SysTickDelayus(100);
	while(GPIO_ReadInputDataBit(GPIO_ADS1256DRDY_PORT,GPIO_ADS1256DRDY)){};
    SPI2_ReadWriteByte(ADS1256_CMD_SELFCAL);              //偏移和增益自动校准
    GPIO_SetBits(GPIOB, GPIO_Pin_12);
	
}
unsigned int ADS1256ReadData(void)
{
    unsigned int sum=0;
	GPIO_ResetBits(GPIOB, GPIO_Pin_12);

	while(GPIO_ReadInputDataBit(GPIO_ADS1256DRDY_PORT,GPIO_ADS1256DRDY));    //当ADS1256_DRDY为低时才能写寄存器 
//	ADS1256WREG(ADS1256_MUX,channel);		                                 //设置通道
	SPI2_ReadWriteByte(ADS1256_CMD_SYNC);
	SPI2_ReadWriteByte(ADS1256_CMD_WAKEUP);	               
	SPI2_ReadWriteByte(ADS1256_CMD_RDATA);

   	sum |= (SPI2_ReadWriteByte(0xff) << 16);
	sum |= (SPI2_ReadWriteByte(0xff) << 8);
	sum |= SPI2_ReadWriteByte(0xff);

	GPIO_SetBits(GPIOB, GPIO_Pin_12); 
    return sum;
}
void ADS1256WREG(unsigned char regaddr,unsigned char databyte)
{
	GPIO_ResetBits(GPIOB, GPIO_Pin_12);
	while(GPIO_ReadInputDataBit(GPIO_ADS1256DRDY_PORT,GPIO_ADS1256DRDY)){};//当ADS1256_DRDY为低时才能写寄存器
	//向寄存器写入数据地址
    SPI2_ReadWriteByte(ADS1256_CMD_WREG | (regaddr & 0x0F));
    //写入数据的个数n-1
    SPI2_ReadWriteByte(0x00);
    //向regaddr地址指向的寄存器写入数据databyte
    SPI2_ReadWriteByte(databyte);
	GPIO_SetBits(GPIOB, GPIO_Pin_12);
}
unsigned int ADS_sum(unsigned char channel)
{
	ADS1256WREG(ADS1256_MUX,channel);		//设置通道
	return ADS1256ReadData();//读取AD值，返回24位数据。
}

long double k=4.0;
long ulResult;
long double ldVolutage[11];
long int test_ads;

//3*8 帧率200
long double TVolutage;
int arrivage_count = 3;//平均次数
long double TureVolutage;
long double TureVolutage1;
long double GetDistance(int iq)
{
	int j = 0;
	int i = 0;
	u8 ADS1256_MUXP_AINx;
	if(iq == 1)
	{
		ADS1256_MUXP_AINx = ADS1256_MUXP_AIN1;
	}
	else if(iq == 2)
	{
		ADS1256_MUXP_AINx = ADS1256_MUXP_AIN2;
	}
	else if(iq == 3)
	{
		ADS1256_MUXP_AINx = ADS1256_MUXP_AIN3;
	}
	else if(iq == 4)
	{
		ADS1256_MUXP_AINx = ADS1256_MUXP_AIN4;
	}
	else if(iq == 5)
	{
		ADS1256_MUXP_AINx = ADS1256_MUXP_AIN5;
	}
	else if(iq == 6)
	{
		ADS1256_MUXP_AINx = ADS1256_MUXP_AIN6;
	}
	else if(iq == 7)
	{
		ADS1256_MUXP_AINx = ADS1256_MUXP_AIN7;
	}
	for( j = 0;j < arrivage_count ;j++ )
	{
//		ulResult = ADS_sum((i << 4) | ADS1256_MUXN_AINCOM);                           //00000001->00010000;00010000|00001000 = 00011000 = 0x18	，第二种形式，不建议用因为算起来很麻烦	
		ulResult = ADS_sum( ADS1256_MUXP_AINx | ADS1256_MUXN_AINCOM);
//		ulResult = ADS_sum( ADS1256_MUXP_AIN1 | ADS1256_MUXN_AINCOM);			
		//读出的数和实际电压的关系
		//V实=u读/5V*0x7fffff
		if( ulResult & 0x800000 )
		{
			ulResult = ~(unsigned long)ulResult;
			ulResult &= 0x7fffff;
			ulResult += 1;
			ulResult = -ulResult;
		}
//		TVolutage=(long double)ulResult*0.59604644775390625*2*2;
		ldVolutage[j] = (long double)ulResult*0.59604644775390625*k;                 //我并不知道它到底是咋来的，是拟合出来的吗？
	}
	TureVolutage = 0;
	//ADS1256第一次采集时，ADS1256不稳定，采集的数据不准确
	//可以连续多采集几次然后舍弃前两次数据，再做均值处理
	for(i = 1;i < arrivage_count;i++)
	{
		TureVolutage = TureVolutage + ldVolutage[i];
	}
	TureVolutage1 = TureVolutage*0.000001/(arrivage_count-1);
	return TureVolutage1;
}

