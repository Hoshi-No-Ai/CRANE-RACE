#include "STM32Lib\\stm32f10x.h"
#include "main.h"

static CanRxMsg CAN_RX_Buf[CAN_RX_BUF_MAX];		//���Ͷ���
static CanTxMsg CAN_TX_Buf[CAN_TX_BUF_MAX];		//���ն���

volatile u32 CAN_Rx_Head = 0;					//����ͷ-���յ�ʱ���ƶ�
volatile u32 CAN_Rx_Tail = 0;					//����β-��ȡ��ʱ���ƶ�

volatile u32 CAN_Tx_Head = 0;					//����ͷ-���յ�ʱ���ƶ�
volatile u32 CAN_Tx_Tail = 0;					//����β-��ȡ��ʱ���ƶ�

void CAN_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB, ENABLE);	
	
	/* PB8-CAN RX */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/*PB9-CAN TX */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_PinRemapConfig(GPIO_Remap1_CAN1,ENABLE);    //�˿���ӳ�䵽PB8,PB9
}


/*******************************************************************************
**CAN�ж�
*******************************************************************************/
void CAN_Interrupt(void)
{
	CAN_InitTypeDef        CAN_InitStructure;
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
//	CanTxMsg TxMessage;
	
	/* CAN register init */
	CAN_DeInit(CAN1);			  //��CAN��������Ϊ��ʼֵ
	CAN_StructInit(&CAN_InitStructure);	   //��CAN_InitStruct�е�ÿһ��������ȱʡֵ����		

	/* CAN cell init */
	CAN_InitStructure.CAN_TTCM=ENABLE;		//ʹ��ʱ�䴥��ģʽ
	CAN_InitStructure.CAN_ABOM=ENABLE;		//ʹ���Զ����߹���
	CAN_InitStructure.CAN_AWUM=ENABLE;		//ʹ���Զ�����
	CAN_InitStructure.CAN_NART=DISABLE;		//ENABLE:�����Զ��ش� DISABLE:�ش�
	CAN_InitStructure.CAN_RFLM=DISABLE;		//ʧ��FIFO����
	CAN_InitStructure.CAN_TXFP=DISABLE;		//ʧ�ܷ���FIFO���ȼ�
	CAN_InitStructure.CAN_Mode=CAN_Mode_Normal;		//��������ģʽ
	CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;			//1-4  ͬ����Ծ���(SJW)������ÿλ�п����ӳ������̶��ٸ�ʱ�䵥λ������
	CAN_InitStructure.CAN_BS1=CAN_BS1_2tq;			//1-16 �趨��ʱ��� 1 ��ʱ�䵥λ��
	CAN_InitStructure.CAN_BS2=CAN_BS2_1tq;			//1-8  �趨��ʱ��� 2 ��ʱ�䵥λ��Ŀ
	CAN_InitStructure.CAN_Prescaler=9;				//������Ϊ 36/(9*(1+2+1))=1M
	CAN_Init(CAN1,&CAN_InitStructure);				//���� CAN_InitStruct ��ָ���Ĳ�����ʼ������ CAN �ļĴ���
	//CAN_InitStruct��ָ��ṹ CAN_InitTypeDef ��ָ�룬������ָ������ CAN��������Ϣ

	
	/* CAN ���������ã�ƥ��CANRXSERVOID */
	CAN_FilterInitStructure.CAN_FilterNumber=0;			   //ָ���˴���ʼ���Ĺ�����
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;	//��ʶ������λģʽ 
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;	   //������λ��:1 �� 32 λ������ 

	CAN_FilterInitStructure.CAN_FilterIdHigh=(((u32)CANRXSERVOID<<21)&0xFFFF0000)>>16;	   //�趨��������ʶ��
	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;//�趨��������ʶ��

	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0xffe0;   //�趨���������α�ʶ��  
	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;	   //�趨���������α�ʶ��
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_FIFO0;	//ָ��������� FIFO��0 �� 1��
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;		//ʹ�ܹ�����
	CAN_FilterInit(&CAN_FilterInitStructure);					//���� CAN_FilterInitStruct ��ָ���Ĳ�����ʼ������ CAN �ļĴ���
												//CAN_InitStruct��ָ��ṹ CAN_InitTypeDef ��ָ�룬������ָ������ CAN��������Ϣ
	

	/* CAN ����������,ƥ��CANRXVALVEID */
	CAN_FilterInitStructure.CAN_FilterNumber=1;			   //ָ���˴���ʼ���Ĺ�����
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;	//��ʶ������λģʽ 
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;	   //������λ��:1 �� 32 λ������ 

	CAN_FilterInitStructure.CAN_FilterIdHigh=(((u32)CANRXVALVEID<<21)&0xFFFF0000)>>16;	   //�趨��������ʶ��
	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;//�趨��������ʶ��

	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0xffe0;   //�趨���������α�ʶ��  
	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;	   //�趨���������α�ʶ��
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_FIFO0;	//ָ��������� FIFO��0 �� 1��
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;		//ʹ�ܹ�����
	CAN_FilterInit(&CAN_FilterInitStructure);					//���� CA N_FilterInitStruct ��ָ���Ĳ�����ʼ������ CAN �ļĴ���
												//CAN_InitStruct��ָ��ṹ CAN_InitTypeDef ��ָ�룬������ָ������ CAN��������Ϣ
		
	/* ����FMP0�ж�*/ 
	CAN_ITConfig(CAN1,CAN_IT_FMP0, ENABLE);		  //ʹ�ܻ���ʧ��ָ���� CAN �ж�

}



/*------------------------------------------------------------------
                    ���º���������й�
------------------------------------------------------------------*/

/**************************************************************
** ������:CANPutDatatoRxBuf
** ����:�����ݷŽ�������
** ע������:Ӧ���ڴ��ڽ����ж��е��ô˺���
***************************************************************/
void CANPutDatatoRxBuf(CanRxMsg *pRxMessage)
{
	u32 tmphead;
	tmphead = ( CAN_Rx_Head + 1 ) & CAN_RX_BUF_MARK;//����ͷ�����ֵ�ж�,�������,����0
	CAN_Rx_Head = tmphead; 					// ÿ��һ������,����ͷ����1 
	CAN_RX_Buf[tmphead] = *pRxMessage; 		// �ѽ��յ������ݷŽ������� 		
}
/*************************************************
**������:CANIsDataInRxBuf
**����:��֪�������Ƿ�������
**ע������:�����е�ͷ��β����ȵ�ʱ��,�ʹ������������ڻ�����
*************************************************/
bool CANIsDataInRxBuf( void )
{
	if( CAN_Rx_Head == CAN_Rx_Tail )
		return FALSE;
	else 
		return TRUE; 
}
/*************************************************
**������:CANGetRxBufLen
**����:��ȡ��������Ч���ݵĳ���
**ע������:��ȡ��ֵ��ȻΪ��Сֵ,��Ϊ��ʵ���Ȼ᲻�ϱ仯.������32λ��ARMϵͳ,��ȡ32λ���ݲ������ٽ�����,���Կ��Բ����ǹ��ж�
**************************************************/
u32 CANGetRxBufLen(void)
{
	//__disalbe_irq();
	if(CAN_Rx_Head >= CAN_Rx_Tail)
	{
		//__enable_irq();	
		return(CAN_Rx_Head - CAN_Rx_Tail);
	}
	else
	{
		//__enable_irq();	
		return(CAN_RX_BUF_MAX + CAN_Rx_Head - CAN_Rx_Tail);
	}
}
/**************************************************
**������:CANGetRxBufDat
**����:�Ӷ����л�ȡ����
**ע������:���ô˺���ǰ����ȷ��������������!!���������Ӳ�ȴ�
**************************************************/
CanRxMsg *CANGetRxBufDat( void )
{
	u32 tmptail;
	while ( CAN_Rx_Head == CAN_Rx_Tail );//Ϊ��ֹ���ݻ��Ҷ�Ū�ϵ�Ӳ�ȴ�
	tmptail = ( CAN_Rx_Tail + 1 ) & CAN_RX_BUF_MARK;
	CAN_Rx_Tail = tmptail;
	return &CAN_RX_Buf[tmptail];		 //���ض����еȴ��������ݵĵ�ַ
}


/*------------------------------------------------------------------
                    ���º����뷢���й�
------------------------------------------------------------------*/

/**************************************************************
** ������:CANPutDatatoTxBuf
** ����:�����ݷŽ����Ͷ�����
** ע������:�û���Ҫ�����ݷ���ʱ��ʹ��
***************************************************************/
bool CANPutDatatoTxBuf(CanTxMsg *pTxMessage)
{
	u32 tmphead;
	tmphead = ( CAN_Tx_Head + 1 ) & CAN_TX_BUF_MARK;//����ĩ���ж�,����ĩ��,����0
	if(tmphead == CAN_Tx_Tail)
		return FALSE;
	
	CAN_Tx_Head = tmphead; 	// ÿ����,����ͷ����1 
	CAN_TX_Buf[tmphead] = *pTxMessage; 	
	return TRUE;			
}

/*************************************************
**������:CANIsDataInTxBuf
**����:��֪�������Ƿ�������
**ע������:�����е�ͷ��β����ȵ�ʱ��,�ʹ������������ڻ�����
*************************************************/
bool CANIsDataInTxBuf( void )
{
	if( CAN_Tx_Head == CAN_Tx_Tail )
		return FALSE;
	else 
		return TRUE; 
}

/*************************************************
**������:CANGetTxBufLen
**����:��ȡ��������Ч���ݵĳ���
**ע������:��ȡ��ֵ��ȻΪ��Сֵ,��Ϊ��ʵ���Ȼ᲻�ϱ仯.������32λ��ARMϵͳ,��ȡ32λ���ݲ������ٽ�����,���Կ��Բ����ǹ��ж�
**			��ν��Ч,��ָʣ��Ŀ��ó���
**************************************************/
u32 CANGetTxBufLen(void)
{
	//__disalbe_irq();
	if(CAN_Tx_Head >= CAN_Tx_Tail)
	{
		//__enable_irq();	
		return(CAN_TX_BUF_MAX - (CAN_Tx_Head-CAN_Tx_Tail));
	}
	else
	{
		//__enable_irq();	
		return(CAN_Tx_Tail - CAN_Tx_Head);
	}
}
/**************************************************
**������:CANGetTxBufDat
**����:�Ӷ����л�ȡ����
**ע������:���ô˺���ǰ����ȷ��������������!!
**************************************************/
CanTxMsg *CANGetTxBufDat( void )
{
	u32 tmptail;
	while ( CAN_Tx_Head == CAN_Tx_Tail );//Ϊ��ֹ���ݻ��Ҷ�Ū�ϵ�Ӳ�ȴ�
	tmptail = ( CAN_Tx_Tail + 1 ) & CAN_TX_BUF_MARK;
	CAN_Tx_Tail = tmptail;
	return &CAN_TX_Buf[tmptail];
}

/*******************************************************
**������:CANBeginSend/StopSend
**����:��������
**ע������:����ʹ�ÿ��жϷ�ʽ����,ֻҪ���ͼĴ���Ϊ��,�����뷢�Ϳ��ж�,ϵͳ�����ж��н��з���
********************************************************/
void CANBeginSend(void)
{
//	CAN_ITConfig(CAN1,CAN_IT_TME, ENABLE);//����������ж�
	CAN1->IER |= CAN_IT_TME;
}
void CANStopSend(void)
{
	CAN1->IER &= ~CAN_IT_TME;
}



/*******************************************************
**������:SendUCHARByCan/SendUSHORT16ByCan/SendSINT32ByCan/SendArrayUCHAR8ByCan/SendArrayUSHORT16ByCan
**����:����1��2��4�ֽ����ݡ���������
**ע������:���н��з���
********************************************************/
//����һ��1�ֽڵ�����
void SendUCHARByCan(u8 id , u8 dat)
{
	CanTxMsg TxMessage;
	TxMessage.StdId = id;
	TxMessage.IDE=CAN_ID_STD;
	TxMessage.RTR=CAN_RTR_DATA;
	TxMessage.DLC=1;
	TxMessage.Data[0] = dat;
	
	CAN_Transmit(CAN1,&TxMessage);
}
//����һ��2�ֽڵ�����
void SendUSHORT16ByCan(u8 id , u16 dat)
{
	CanTxMsg TxMessage;
	u16 *p;
	TxMessage.StdId = id;			 //���ͱ�ʶ��
	TxMessage.IDE=CAN_ID_STD;		 //ʹ�ñ�׼��ʶ��
	TxMessage.RTR=CAN_RTR_DATA;		 //֡���ͣ�����֡
	TxMessage.DLC=2;
	p = (u16*)TxMessage.Data;
	*p = dat;
	
	CAN_Transmit(CAN1,&TxMessage);
}
//����һ��4�ֽڵ�����
void SendSINT32ByCan(u8 id , s32 dat)
{
	CanTxMsg TxMessage;
	s32 *p;
	TxMessage.StdId = id;
	TxMessage.IDE=CAN_ID_STD;
	TxMessage.RTR=CAN_RTR_DATA;
	TxMessage.DLC=4;
	p = (s32*)TxMessage.Data;
	*p = dat;
	
	CAN_Transmit(CAN1,&TxMessage);
}
//����һ��u8���͵�����
void SendArrayUCHAR8ByCan(u8 id , u8 dat[] , u8 dlc)
{
	u8 i = 0 ;
	CanTxMsg TxMessage;
	TxMessage.StdId = id;
	TxMessage.IDE=CAN_ID_STD;
	TxMessage.RTR=CAN_RTR_DATA;
	TxMessage.DLC=dlc;
	for(i = 0; i<dlc; i++)
	{
		TxMessage.Data[i] = dat[i]	;
	}
	CAN_Transmit(CAN1,&TxMessage);
}

//����һ��u16���͵�����,dlcΪ�����ֽ�����dlc=sizeof(dat)/sizeof(u16)
void SendArrayUSHORT16ByCan(u8 id , u16 dat[],u8 dlc)
{
	u8 i = 0 ;
	u16 *p;
	
	CanTxMsg TxMessage;
	TxMessage.StdId = id;
	TxMessage.IDE=CAN_ID_STD;
	TxMessage.RTR=CAN_RTR_DATA;
	TxMessage.DLC=dlc;

	p = (u16*)TxMessage.Data;	
	for(i = 0; i<dlc; i++)
	{
		*(p+i) = dat[i]	;
	}
	CAN_Transmit(CAN1,&TxMessage);
}
//DT35ר�÷��ͺ���
void SendDT35(s16 id , u16 k1, u16 k2, u16 k3, u16 k4)
{
	CanTxMsg TxMessage;
	TxMessage.StdId = id;
	TxMessage.IDE=CAN_ID_STD;
	TxMessage.RTR=CAN_RTR_DATA;
	TxMessage.DLC=8;           //�����ֽ�
	TxMessage.Data[0] = k1;
	TxMessage.Data[1] = k1>>8;
	TxMessage.Data[2] = k2;
	TxMessage.Data[3] = k2>>8;
	TxMessage.Data[4] = k3;
	TxMessage.Data[5] = k3>>8;
	TxMessage.Data[6] = k4;
	TxMessage.Data[7] = k4>>8;
	CAN_Transmit(CAN1,&TxMessage);
}
