#include "STM32Lib\\stm32f10x.h"
#include "main.h"

static CanRxMsg CAN_RX_Buf[CAN_RX_BUF_MAX];		//发送队列
static CanTxMsg CAN_TX_Buf[CAN_TX_BUF_MAX];		//接收队列

volatile u32 CAN_Rx_Head = 0;					//队列头-接收的时候移动
volatile u32 CAN_Rx_Tail = 0;					//队列尾-读取的时候移动

volatile u32 CAN_Tx_Head = 0;					//队列头-接收的时候移动
volatile u32 CAN_Tx_Tail = 0;					//队列尾-读取的时候移动

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

	GPIO_PinRemapConfig(GPIO_Remap1_CAN1,ENABLE);    //端口重映射到PB8,PB9
}


/*******************************************************************************
**CAN中断
*******************************************************************************/
void CAN_Interrupt(void)
{
	CAN_InitTypeDef        CAN_InitStructure;
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
//	CanTxMsg TxMessage;
	
	/* CAN register init */
	CAN_DeInit(CAN1);			  //将CAN的外设设为初始值
	CAN_StructInit(&CAN_InitStructure);	   //把CAN_InitStruct中的每一个参数按缺省值填入		

	/* CAN cell init */
	CAN_InitStructure.CAN_TTCM=ENABLE;		//使能时间触发模式
	CAN_InitStructure.CAN_ABOM=ENABLE;		//使能自动离线管理
	CAN_InitStructure.CAN_AWUM=ENABLE;		//使能自动唤醒
	CAN_InitStructure.CAN_NART=DISABLE;		//ENABLE:错误不自动重传 DISABLE:重传
	CAN_InitStructure.CAN_RFLM=DISABLE;		//失能FIFO锁定
	CAN_InitStructure.CAN_TXFP=DISABLE;		//失能发送FIFO优先级
	CAN_InitStructure.CAN_Mode=CAN_Mode_Normal;		//正常传输模式
	CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;			//1-4  同步跳跃宽度(SJW)，即在每位中可以延长或缩短多少个时间单位的上限
	CAN_InitStructure.CAN_BS1=CAN_BS1_2tq;			//1-16 设定了时间段 1 的时间单位数
	CAN_InitStructure.CAN_BS2=CAN_BS2_1tq;			//1-8  设定了时间段 2 的时间单位数目
	CAN_InitStructure.CAN_Prescaler=9;				//波特率为 36/(9*(1+2+1))=1M
	CAN_Init(CAN1,&CAN_InitStructure);				//根据 CAN_InitStruct 中指定的参数初始化外设 CAN 的寄存器
	//CAN_InitStruct：指向结构 CAN_InitTypeDef 的指针，包含了指定外设 CAN的配置信息

	
	/* CAN 过滤器设置，匹配CANRXSERVOID */
	CAN_FilterInitStructure.CAN_FilterNumber=0;			   //指定了待初始化的过滤器
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;	//标识符屏蔽位模式 
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;	   //过滤器位宽:1 个 32 位过滤器 

	CAN_FilterInitStructure.CAN_FilterIdHigh=(((u32)CANRXSERVOID<<21)&0xFFFF0000)>>16;	   //设定过滤器标识符
	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;//设定过滤器标识符

	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0xffe0;   //设定过滤器屏蔽标识符  
	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;	   //设定过滤器屏蔽标识符
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_FIFO0;	//指向过滤器的 FIFO（0 或 1）
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;		//使能过滤器
	CAN_FilterInit(&CAN_FilterInitStructure);					//根据 CAN_FilterInitStruct 中指定的参数初始化外设 CAN 的寄存器
												//CAN_InitStruct：指向结构 CAN_InitTypeDef 的指针，包含了指定外设 CAN的配置信息
	

	/* CAN 过滤器设置,匹配CANRXVALVEID */
	CAN_FilterInitStructure.CAN_FilterNumber=1;			   //指定了待初始化的过滤器
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;	//标识符屏蔽位模式 
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;	   //过滤器位宽:1 个 32 位过滤器 

	CAN_FilterInitStructure.CAN_FilterIdHigh=(((u32)CANRXVALVEID<<21)&0xFFFF0000)>>16;	   //设定过滤器标识符
	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;//设定过滤器标识符

	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0xffe0;   //设定过滤器屏蔽标识符  
	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;	   //设定过滤器屏蔽标识符
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_FIFO0;	//指向过滤器的 FIFO（0 或 1）
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;		//使能过滤器
	CAN_FilterInit(&CAN_FilterInitStructure);					//根据 CA N_FilterInitStruct 中指定的参数初始化外设 CAN 的寄存器
												//CAN_InitStruct：指向结构 CAN_InitTypeDef 的指针，包含了指定外设 CAN的配置信息
		
	/* 允许FMP0中断*/ 
	CAN_ITConfig(CAN1,CAN_IT_FMP0, ENABLE);		  //使能或者失能指定的 CAN 中断

}



/*------------------------------------------------------------------
                    以下函数与接收有关
------------------------------------------------------------------*/

/**************************************************************
** 函数名:CANPutDatatoRxBuf
** 功能:把数据放进队列中
** 注意事项:应该在串口接收中断中调用此函数
***************************************************************/
void CANPutDatatoRxBuf(CanRxMsg *pRxMessage)
{
	u32 tmphead;
	tmphead = ( CAN_Rx_Head + 1 ) & CAN_RX_BUF_MARK;//队列头的最大值判断,到达最大,则变回0
	CAN_Rx_Head = tmphead; 					// 每收一次数据,队列头增加1 
	CAN_RX_Buf[tmphead] = *pRxMessage; 		// 把接收到的数据放进队列中 		
}
/*************************************************
**函数名:CANIsDataInRxBuf
**功能:获知缓冲中是否有数据
**注意事项:当队列的头和尾不相等的时候,就代表了有数据在缓冲中
*************************************************/
bool CANIsDataInRxBuf( void )
{
	if( CAN_Rx_Head == CAN_Rx_Tail )
		return FALSE;
	else 
		return TRUE; 
}
/*************************************************
**函数名:CANGetRxBufLen
**功能:获取缓冲中有效数据的长度
**注意事项:获取的值必然为最小值,因为真实长度会不断变化.由于是32位的ARM系统,获取32位数据不存在临界问题,所以可以不考虑关中断
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
**函数名:CANGetRxBufDat
**功能:从队列中获取数据
**注意事项:调用此函数前请先确保队列中有数据!!否则会陷入硬等待
**************************************************/
CanRxMsg *CANGetRxBufDat( void )
{
	u32 tmptail;
	while ( CAN_Rx_Head == CAN_Rx_Tail );//为防止数据混乱而弄上的硬等待
	tmptail = ( CAN_Rx_Tail + 1 ) & CAN_RX_BUF_MARK;
	CAN_Rx_Tail = tmptail;
	return &CAN_RX_Buf[tmptail];		 //返回队列中等待处理数据的地址
}


/*------------------------------------------------------------------
                    以下函数与发送有关
------------------------------------------------------------------*/

/**************************************************************
** 函数名:CANPutDatatoTxBuf
** 功能:把数据放进发送队列中
** 注意事项:用户需要有数据发的时候使用
***************************************************************/
bool CANPutDatatoTxBuf(CanTxMsg *pTxMessage)
{
	u32 tmphead;
	tmphead = ( CAN_Tx_Head + 1 ) & CAN_TX_BUF_MARK;//队列末端判断,到达末端,则变回0
	if(tmphead == CAN_Tx_Tail)
		return FALSE;
	
	CAN_Tx_Head = tmphead; 	// 每入列,队列头增加1 
	CAN_TX_Buf[tmphead] = *pTxMessage; 	
	return TRUE;			
}

/*************************************************
**函数名:CANIsDataInTxBuf
**功能:获知缓冲中是否有数据
**注意事项:当队列的头和尾不相等的时候,就代表了有数据在缓冲中
*************************************************/
bool CANIsDataInTxBuf( void )
{
	if( CAN_Tx_Head == CAN_Tx_Tail )
		return FALSE;
	else 
		return TRUE; 
}

/*************************************************
**函数名:CANGetTxBufLen
**功能:获取缓冲中有效数据的长度
**注意事项:获取的值必然为最小值,因为真实长度会不断变化.由于是32位的ARM系统,获取32位数据不存在临界问题,所以可以不考虑关中断
**			所谓有效,是指剩余的可用长度
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
**函数名:CANGetTxBufDat
**功能:从队列中获取数据
**注意事项:调用此函数前请先确保队列中有数据!!
**************************************************/
CanTxMsg *CANGetTxBufDat( void )
{
	u32 tmptail;
	while ( CAN_Tx_Head == CAN_Tx_Tail );//为防止数据混乱而弄上的硬等待
	tmptail = ( CAN_Tx_Tail + 1 ) & CAN_TX_BUF_MARK;
	CAN_Tx_Tail = tmptail;
	return &CAN_TX_Buf[tmptail];
}

/*******************************************************
**函数名:CANBeginSend/StopSend
**功能:启动发送
**注意事项:这里使用空中断方式发送,只要发送寄存器为空,则会进入发送空中断,系统再在中断中进行发送
********************************************************/
void CANBeginSend(void)
{
//	CAN_ITConfig(CAN1,CAN_IT_TME, ENABLE);//发送邮箱空中断
	CAN1->IER |= CAN_IT_TME;
}
void CANStopSend(void)
{
	CAN1->IER &= ~CAN_IT_TME;
}



/*******************************************************
**函数名:SendUCHARByCan/SendUSHORT16ByCan/SendSINT32ByCan/SendArrayUCHAR8ByCan/SendArrayUSHORT16ByCan
**功能:发送1、2、4字节数据、发送数组
**注意事项:断中进行发送
********************************************************/
//发送一个1字节的数据
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
//发送一个2字节的数据
void SendUSHORT16ByCan(u8 id , u16 dat)
{
	CanTxMsg TxMessage;
	u16 *p;
	TxMessage.StdId = id;			 //发送标识符
	TxMessage.IDE=CAN_ID_STD;		 //使用标准标识符
	TxMessage.RTR=CAN_RTR_DATA;		 //帧类型：数据帧
	TxMessage.DLC=2;
	p = (u16*)TxMessage.Data;
	*p = dat;
	
	CAN_Transmit(CAN1,&TxMessage);
}
//发送一个4字节的数据
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
//发送一个u8类型的数组
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

//发送一个u16类型的数组,dlc为发送字节数，dlc=sizeof(dat)/sizeof(u16)
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
//DT35专用发送函数
void SendDT35(s16 id , u16 k1, u16 k2, u16 k3, u16 k4)
{
	CanTxMsg TxMessage;
	TxMessage.StdId = id;
	TxMessage.IDE=CAN_ID_STD;
	TxMessage.RTR=CAN_RTR_DATA;
	TxMessage.DLC=8;           //两个字节
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
