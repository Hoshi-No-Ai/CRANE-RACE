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
	SPI2_ReadWriteByte(ADS1256_CMD_SYNC);                 //ͬ������
	SPI2_ReadWriteByte(ADS1256_CMD_WAKEUP);               //ͬ������	
	while(GPIO_ReadInputDataBit(GPIO_ADS1256DRDY_PORT,GPIO_ADS1256DRDY));
	SPI2_ReadWriteByte(ADS1256_CMD_WREG | ADS1256_STATUS);//����д��4���Ĵ���
	SPI2_ReadWriteByte(3);
	SPI2_ReadWriteByte(0x06);                             //��λ��ǰ��ʹ���ڲ�У׼��ʹ�û���
	SPI2_ReadWriteByte(ADS1256_MUXP_AIN0 | ADS1256_MUXN_AINCOM);//�˿�����A2Ϊ����A3λ��
	SPI2_ReadWriteByte(ADS1256_GAIN_1);                   //�Ŵ�������
//	SPI2_ReadWriteByte(ADS1256_DRATE_30000SPS);//30000       //�ɼ��ٶ�����
	SPI2_ReadWriteByte(ADS1256_DRATE_7500SPS);//ADS1256_DRATE_7500SPS
	ADS1256WREG(ADS1256_IO,0x00);
	SysTickDelayus(100);
	while(GPIO_ReadInputDataBit(GPIO_ADS1256DRDY_PORT,GPIO_ADS1256DRDY)){};
    SPI2_ReadWriteByte(ADS1256_CMD_SELFCAL);              //ƫ�ƺ������Զ�У׼
    GPIO_SetBits(GPIOB, GPIO_Pin_12);
	
}
unsigned int ADS1256ReadData(void)
{
    unsigned int sum=0;
	GPIO_ResetBits(GPIOB, GPIO_Pin_12);

	while(GPIO_ReadInputDataBit(GPIO_ADS1256DRDY_PORT,GPIO_ADS1256DRDY));    //��ADS1256_DRDYΪ��ʱ����д�Ĵ��� 
//	ADS1256WREG(ADS1256_MUX,channel);		                                 //����ͨ��
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
	while(GPIO_ReadInputDataBit(GPIO_ADS1256DRDY_PORT,GPIO_ADS1256DRDY)){};//��ADS1256_DRDYΪ��ʱ����д�Ĵ���
	//��Ĵ���д�����ݵ�ַ
    SPI2_ReadWriteByte(ADS1256_CMD_WREG | (regaddr & 0x0F));
    //д�����ݵĸ���n-1
    SPI2_ReadWriteByte(0x00);
    //��regaddr��ַָ��ļĴ���д������databyte
    SPI2_ReadWriteByte(databyte);
	GPIO_SetBits(GPIOB, GPIO_Pin_12);
}
unsigned int ADS_sum(unsigned char channel)
{
	ADS1256WREG(ADS1256_MUX,channel);		//����ͨ��
	return ADS1256ReadData();//��ȡADֵ������24λ���ݡ�
}

long double k=4.0;
long ulResult;
long double ldVolutage[11];
long int test_ads;

//3*8 ֡��200
long double TVolutage;
int arrivage_count = 3;//ƽ������
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
//		ulResult = ADS_sum((i << 4) | ADS1256_MUXN_AINCOM);                           //00000001->00010000;00010000|00001000 = 00011000 = 0x18	���ڶ�����ʽ������������Ϊ���������鷳	
		ulResult = ADS_sum( ADS1256_MUXP_AINx | ADS1256_MUXN_AINCOM);
//		ulResult = ADS_sum( ADS1256_MUXP_AIN1 | ADS1256_MUXN_AINCOM);			
		//����������ʵ�ʵ�ѹ�Ĺ�ϵ
		//Vʵ=u��/5V*0x7fffff
		if( ulResult & 0x800000 )
		{
			ulResult = ~(unsigned long)ulResult;
			ulResult &= 0x7fffff;
			ulResult += 1;
			ulResult = -ulResult;
		}
//		TVolutage=(long double)ulResult*0.59604644775390625*2*2;
		ldVolutage[j] = (long double)ulResult*0.59604644775390625*k;                 //�Ҳ���֪����������զ���ģ�����ϳ�������
	}
	TureVolutage = 0;
	//ADS1256��һ�βɼ�ʱ��ADS1256���ȶ����ɼ������ݲ�׼ȷ
	//����������ɼ�����Ȼ������ǰ�������ݣ�������ֵ����
	for(i = 1;i < arrivage_count;i++)
	{
		TureVolutage = TureVolutage + ldVolutage[i];
	}
	TureVolutage1 = TureVolutage*0.000001/(arrivage_count-1);
	return TureVolutage1;
}

