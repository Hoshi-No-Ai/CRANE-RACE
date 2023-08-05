#include "STM32Lib\\stm32f10x.h"
#include "main.h"

/*******************************************************************************
* Function Name  : GPIO_Configuration
* ���������ӿ�Ϊ���
* �����г̿��ؽӿ�Ϊ����
*******************************************************************************/
void GPIO_Configuration(void)
{
	GPIO_InitTypeDef     GPIO_InitStructure;
	
	/*��������CLOCK,��ʹ��GPIO֮ǰ����������Ӧ�˵�ʱ��.*/
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
	
  /* PB7���,RUNָʾ�� */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	//��©���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//50Mʱ���ٶ�
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/* PA4,5,8,9,10,11��� */
	GPIO_SetBits(GPIOA, GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11);		//Ԥ��Ϊ��,����

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;	//��©���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//50Mʱ���ٶ�
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/*PB10,11,12,13,14,15���*/
	GPIO_SetBits(GPIOB, GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15);		//Ԥ��Ϊ��,����

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;	//��©���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//50Mʱ���ٶ�
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/* PC6,7,8,9���*/
	GPIO_SetBits(GPIOC, GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9);		//Ԥ��Ϊ��,����

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;	//��©���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//50Mʱ���ٶ�
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	/* PB5,6��������*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;		//��������
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* PC3,10,11,12,13,14,15��������*/
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;		//��������
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	/* PD2��������*/
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;		//��������
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	
//	/* PB3,4��������*/
//	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);    
//	/* Disable JLink, enable SW */
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE); 
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;   
//	/* Push-pill output, it can be other output types */
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
//	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void GPIO_JTAG_Configuration(void)
{
	GPIO_InitTypeDef     GPIO_InitStructure;
	
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);    
	/* Disable JLink, enable SW */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE); 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;   
	/* Push-pill output, it can be other output types */
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}
