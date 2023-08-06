#ifndef _MAIN_H
#define _MAIN_H

#include "stdio.h"
#include "hal.h"
#include "GPIO.h"
#include "RCC.h"
#include "CAN.h"
#include "TIMER.h"
#include "PositionSwitch.h"
#include "Valve.h"
#include "NVIC.h"
#include "SPI.h"
#include "ADS1256.h"

extern CanRxMsg st_RxMessage;

void SendKeyStat(void);
void RcvValveCtrl_new(void);
void CAN_RX_Manage(void);

#define SENDKEYENABLE   1		   			//是否使能100ms发数功能，0为屏蔽，1为使能
#define LEDROLLING_TIME 500		   			//灯翻转时间
#define KEYSCAN_TIME    1 		   			//按键扫描时间设置为APPSCAN_TIME ms 
#define DISSHAKE_TIME   0//10		   			//消抖时间设置为DISSHAKE_TIME ms
#define SENDKEY_TIME    1		   			//周期发送KEY状态时间为SENDKEY_TIME ms

#define CANTXKEYID   	0x20				//按键检测发送ID
#define CANTEKID_PLUS 0X21
#define CANRXSERVOID	0x40	   			//舵机控制ID
#define CANRXVALVEID    0x30	  	 		//气动控制ID
#define CANRXDT35ID     0X50                //DT35控制ID


extern volatile u16  ucLedCnt ;				//LED灯闪烁计数
extern volatile u16  ucTimDlyCntms ;		//SysTick硬件中断计数
extern volatile u16  ucTimDlyCntus ;		//SysTick硬件中断计数
extern volatile u8   ucKeyScanFlag  ;   	//按键检测标志
extern volatile u16  usSendKeyPer  ; 		//定期发送按键状态计数


extern u8   aucKeyScanCnt[12]  ;			//按键扫描计数
extern u8   aucDisShakeCnt[12] ;			//消抖扫描计数
extern u8   aucDisShakeFlag[12];			//消抖标志位
extern u8   aucKeyStatTemp[12];				//按键临时存储值


extern u16   usKeyStat ;					//按键本次的值

extern u16   ucValvePreStat ;				//气动上次的值

extern u8    aucServoPreStat[4] ; 			//上一次的舵机状态

extern u8    aucCanRxData[8] ;				//接收到的数据

#endif

