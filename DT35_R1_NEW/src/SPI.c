#include "SPI.h"	  

//������SPI2ģ��ĳ�ʼ�����룬���ó�����ģʽ������SD Card/W25X16/24L01/JF24C							  
//SPI2�ڳ�ʼ��
//�������Ƕ�SPI2�ĳ�ʼ��
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
	SPI2_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; //SPI1����Ϊ����ȫ˫��
	SPI2_InitStructure.SPI_Mode = SPI_Mode_Master;                    //����SPI2Ϊ��ģʽ
	SPI2_InitStructure.SPI_DataSize = SPI_DataSize_8b;                  //SPI���ͽ���8λ֡�ṹ
	SPI2_InitStructure.SPI_CPOL = SPI_CPOL_Low;                   //����ʱ���ڲ�����ʱ��ʱ��Ϊ�͵�ƽ
	SPI2_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;                 //��һ��ʱ���ؿ�ʼ��������
	SPI2_InitStructure.SPI_NSS = SPI_NSS_Soft;                  //NSS�ź��������ʹ��SSIλ������
	SPI2_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256; //���岨����Ԥ��Ƶ��ֵ:������Ԥ��ƵֵΪ8
	SPI2_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;       //���ݴ����MSBλ��ʼ
	SPI2_InitStructure.SPI_CRCPolynomial = 7;         //CRCֵ����Ķ���ʽ
	SPI_Init(SPI2, &SPI2_InitStructure);
 /* Enable SPI2  */
	SPI_Cmd(SPI2, ENABLE);  
} 
//SPI �ٶ����ú���
//SpeedSet:
//SPI_BaudRatePrescaler_2   2��Ƶ   (SPI 36M@sys 72M)
//SPI_BaudRatePrescaler_8   8��Ƶ   (SPI 9M@sys 72M)
//SPI_BaudRatePrescaler_16  16��Ƶ  (SPI 4.5M@sys 72M)
//SPI_BaudRatePrescaler_256 256��Ƶ (SPI 281.25K@sys 72M)
  
void SPI2_SetSpeed(u8 SpeedSet)
{
	SPI2_InitStructure.SPI_BaudRatePrescaler = SpeedSet ;
  	SPI_Init(SPI2, &SPI2_InitStructure);
	SPI_Cmd(SPI2,ENABLE);
} 

//SPIx ��дһ���ֽ�
//TxData:Ҫд����ֽ�
//����ֵ:��ȡ�����ֽ�
u8 SPI2_ReadWriteByte(u8 TxData)
{		
//	u8 retry=0;				 	
//	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET) //���ָ����SPI��־λ�������:���ͻ���ձ�־λ
//		{
////		retry++;
////		if(retry>200)return 0;
//		}			  
//	SPI_I2S_SendData(SPI2, TxData); //ͨ������SPIx����һ������
//	retry=0;
//	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET)//���ָ����SPI��־λ�������:���ܻ���ǿձ�־λ
//		{
////		retry++;
////		if(retry>200)return 0;
//		}	  						    
//	return SPI_I2S_ReceiveData(SPI2); //����ͨ��SPIx������յ�����
  unsigned char RxData=0;

  while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_TXE)==RESET); //                                                   
  SPI_I2S_SendData(SPI2,TxData);

   while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_RXNE)==RESET);

   RxData=SPI_I2S_ReceiveData(SPI2);

    return RxData;
  return RxData;
  
}































