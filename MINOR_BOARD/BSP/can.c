#include "can.h"

/*------------------------------------------------------------
函数名:CAN_Configuration()
函数功能: 初始化系统CAN总线，CAN1&CAN2
输入: 无
输出：无
备注:
-------------------------------------------------------------*/
void CAN_Configuration(void)
{
	/* Private variables ---------------------------------------------------------*/
	CAN_InitTypeDef        CAN_InitStructure;
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
	GPIO_InitTypeDef  GPIO_InitStructure;
	CanTxMsg TxMessage;
  
	/* CAN GPIOs configuration **************************************************/
	/* Connect CAN pins to AF9 */
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_CAN1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_CAN1); 

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_CAN2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_CAN2); 
  
	/* Configure CAN RX and TX pins */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* CAN configuration ********************************************************/  
	/* Enable CAN clock */
  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1 | RCC_APB1Periph_CAN2, ENABLE);
	/* CAN register init */
	CAN_DeInit(CAN1);
	CAN_DeInit(CAN2);


	/* CAN cell init */
	CAN_InitStructure.CAN_TTCM = DISABLE;//事件触发
	CAN_InitStructure.CAN_ABOM = ENABLE;//自动离线管理
	CAN_InitStructure.CAN_AWUM = ENABLE;//自动唤醒
	CAN_InitStructure.CAN_NART = DISABLE;//错误重传
	CAN_InitStructure.CAN_RFLM = DISABLE;
	CAN_InitStructure.CAN_TXFP = ENABLE;
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
	CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
	CAN_InitStructure.CAN_BS1 = CAN_BS1_9tq;
	CAN_InitStructure.CAN_BS2 = CAN_BS2_4tq;
	CAN_InitStructure.CAN_Prescaler = 3;//500K
	CAN_Init(CAN2, &CAN_InitStructure);

	CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
	CAN_InitStructure.CAN_BS1 = CAN_BS1_9tq;
	CAN_InitStructure.CAN_BS2 = CAN_BS2_4tq;
	CAN_InitStructure.CAN_Prescaler = 3;//1M
	CAN_Init(CAN1, &CAN_InitStructure);
	/* CAN filter init */
	CAN_FilterInitStructure.CAN_FilterNumber = 0;
	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
	CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
	CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FIFO0;
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
	CAN_FilterInit(&CAN_FilterInitStructure);
	
	CAN_FilterInitStructure.CAN_FilterNumber = 14;
	CAN_FilterInit(&CAN_FilterInitStructure);
	
	/* Transmit Structure preparation */
	TxMessage.StdId = 0x00;
	TxMessage.ExtId = 0x00;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.IDE = CAN_ID_STD;
	TxMessage.DLC = 1;
	CAN_Transmit(CAN1,&TxMessage);
	CAN_Transmit(CAN2,&TxMessage);
  
	/* Enable FIFO 0 message pending Interrupt */
	CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);//接收中断
	CAN_ITConfig(CAN2, CAN_IT_FMP0, ENABLE);//接收中断		
}
