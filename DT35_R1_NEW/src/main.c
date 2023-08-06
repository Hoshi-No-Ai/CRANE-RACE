#include "STM32Lib\\stm32f10x.h"
#include "main.h"
#include "math.h"

CanRxMsg st_RxMessage;

volatile u16  ucLedCnt = 0 ;		//LED灯闪烁计数
volatile u16  ucTimDlyCntms = 0 ;	//systick中断毫秒计数	
volatile u16  ucTimDlyCntus = 0 ;	//systick中断微秒计数	
volatile u16  count_thousand = 0;   //systick中断最低计数位，每到1000清零，systick中断1us触发一次
volatile u8   ucKeyScanFlag = 0 ;   //按键检测标志
volatile u16  usSendKeyPer = 0 ;    //周期发送按键标识

u8   aucKeyScanCnt[12] = {0} ;		//按键扫描计数
u8   aucDisShakeCnt[12] = {0} ;		//消抖扫描计数
u8   aucDisShakeFlag[12] = {0};		//消抖标志位
u8   aucKeyStatTemp[12]={0};		//按键状态临时存储
u16  usKeyStat = 0 ;				//按键状态值

u16  ucValvePreStat = 0x0000 ;					//上一次的气动状态
u8   aucServoPreStat[4] = {100,100,100,100}; 	//上一次的舵机状态
 
u8	aucCanRxData[8] = {0};			    //接收到的数据

u8 PulseTemp;

///////////////////////////////////
float dtvol1;
float dtvol2;
float dtvol3;
float dtvol4;
u16 dtv1,dtv2,dtv3,dtv4;
u8 tempv1=0;
u8 tempv2=0;
u8 tempv3=0;
u8 tempv4=0;
u8 tempv5=0;
u8 tempv6=0;
u8 tempv7=0;
u8 tempv8=0;

u32 tempave1=0;
u32 tempave2=0;
u32 tempave3=0;
u32 tempave4=0;	
			
u32 tempsum1=0;
u32 tempsum2=0;
u32 tempsum3=0;
u32 tempsum4=0;	

bool westle_calibration;
u8 measure_flag=0;
u16 ket1,ket2,ket3,ket4,ket5;
s16 ZKID=0x210;
int flag=0;
long double CESHI1;
long double CESHI2;
long double CESHI3;
long double CESHI4;
long double CESHI5;
long double CESHI3_BUC;

int m = 0;
///////////////////////////////////////

int main(void)
{
	
	ChipHalInit();			//片内硬件初始化
//	ValveAllStart();//关闭所有气动
	SysTickDelayms(500);      //延时100ms再开始所有动作
//	ValveAllClose();
	while(1)
	{
		/*亮闪烁,证明程序运行*/
 	  if(ucLedCnt >= LEDROLLING_TIME)
	 	{
			LED_ROLLING;
			ucLedCnt = 0;
	  }			
			
			ket1 = CESHI1 = 1000 * GetDistance(1);
			SysTickDelayus(8);
//			ket2 = CESHI2= 1000 * GetDistance(2);
//			SysTickDelayus(8);
			ket3 = CESHI3= 1000 *GetDistance(3);
			SysTickDelayus(8);
			ket4 = CESHI4= 1000 * GetDistance(4);//4
			SysTickDelayus(8);
			ket5 = CESHI5= 1000 * GetDistance(5);
			SysTickDelayus(8);
			
			SendDT35(ZKID, ket5, ket3, ket1, ket4);				//填入主控被设置的ID
//			SysTickDelayms(1);
//			SendDT35(ZKID + 1, ket1, 0, 0, 0);
		
			m++;
	 }			
}


u8 ucKeyNum;		//按键编号0-15
void Scan_key(void)
{
	if( aucDisShakeFlag[ucKeyNum] == 0 )				//正常扫描状态下
		{
			if( aucKeyScanCnt[ucKeyNum] >= KEYSCAN_TIME )	   	//到了一次扫描的时间
			{
				if( ((usKeyStat>>ucKeyNum)&1)!= ScanKey((ucKeyNum)) )	 	//如果原来状态和新状态不一样
				{
					aucKeyStatTemp[ucKeyNum] = ScanKey(ucKeyNum) ;			//临时存对应按键值
					aucDisShakeFlag[ucKeyNum] = 1;				   			//消抖标志位设为1
				}
				else								 						//如果原来状态和新状态一致
				{															//什么都不做
				}
				aucKeyScanCnt[ucKeyNum] = 0;	   					//计数值归零，重新开始计数，等待下一次扫描
			}
			aucKeyScanCnt[ucKeyNum] ++;			   				//按键检测扫描时间计数								
		}
		else												//消抖状态下
		{
	   		if(aucDisShakeCnt[ucKeyNum] >= DISSHAKE_TIME)	   	//到了消抖时间
			{
				if( aucKeyStatTemp[ucKeyNum] == ScanKey(ucKeyNum) )		//上一状态和新状态一致
				{
					usKeyStat ^= (1<<ucKeyNum) ;				   		//对应位的状态翻转				 	ucKeyStat
					SendUSHORT16ByCan(CANTXKEYID,usKeyStat);			//经消抖处理后检测到不同值，通过CAN发送
				}
				else	   												//上一状态和新状态不一致，表示为抖动
				{		   												//什么都不做
				}
				aucDisShakeFlag[ucKeyNum] = 0;						//标志位置0，重新开始正常扫描
				aucDisShakeCnt[ucKeyNum] = 0;						//计数值归零，重新开始计数，等待下一次消抖						
			}
			aucDisShakeCnt[ucKeyNum] ++;			   			//消抖时间计数			
		}
}

u8 shiyan;
void SendKeyStat(void)
{
//	/*检测不同值 就通过can发送*/
////  if( ucKeyScanFlag == 1 )
////  {
////		ucKeyNum = shiyan;
////		Scan_key();
//		ucKeyNum = 1;
//		Scan_key();
//		ucKeyNum = 2;
//		Scan_key();
//		ucKeyNum = 3;
//		Scan_key();
////		ucKeyScanFlag = 0 ;	 //标志位置为0，下次进入中断后才可执行此函数
////   }

	/*检测不同值 就通过can发送*/
  if( ucKeyScanFlag == 1 )
  {
	for( ucKeyNum = 0 ; ucKeyNum < 7 ; ucKeyNum++ )
	{
		if( aucDisShakeFlag[ucKeyNum] == 0 )				//正常扫描状态下
		{
			if( aucKeyScanCnt[ucKeyNum] >= KEYSCAN_TIME )	   	//到了一次扫描的时间
			{
				if( ((usKeyStat>>ucKeyNum)&1)!= ScanKey((ucKeyNum)) )	 	//如果原来状态和新状态不一样
				{
					aucKeyStatTemp[ucKeyNum] = ScanKey(ucKeyNum) ;			//临时存对应按键值
					aucDisShakeFlag[ucKeyNum] = 1;				   			//消抖标志位设为1
				}
				else								 						//如果原来状态和新状态一致
				{															//什么都不做
				}
				aucKeyScanCnt[ucKeyNum] = 0;	   					//计数值归零，重新开始计数，等待下一次扫描
			}
			aucKeyScanCnt[ucKeyNum] ++;			   				//按键检测扫描时间计数								
		}
		else												//消抖状态下
		{
	   		if(aucDisShakeCnt[ucKeyNum] >= DISSHAKE_TIME)	   	//到了消抖时间
			{
				if( aucKeyStatTemp[ucKeyNum] == ScanKey(ucKeyNum) )		//上一状态和新状态一致
				{
					usKeyStat ^= (1<<ucKeyNum) ;				   		//对应位的状态翻转				 	ucKeyStat
					SendUSHORT16ByCan(CANTXKEYID,usKeyStat);			//经消抖处理后检测到不同值，通过CAN发送
				}
				else	   												//上一状态和新状态不一致，表示为抖动
				{		   												//什么都不做
				}
				aucDisShakeFlag[ucKeyNum] = 0;						//标志位置0，重新开始正常扫描
				aucDisShakeCnt[ucKeyNum] = 0;						//计数值归零，重新开始计数，等待下一次消抖						
			}
			aucDisShakeCnt[ucKeyNum] ++;			   			//消抖时间计数			
		}
	 }
	 ucKeyScanFlag = 0 ;	 //标志位置为0，下次进入中断后才可执行此函数
}
}

