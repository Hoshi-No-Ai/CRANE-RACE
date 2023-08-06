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

#define SENDKEYENABLE   1		   			//�Ƿ�ʹ��100ms�������ܣ�0Ϊ���Σ�1Ϊʹ��
#define LEDROLLING_TIME 500		   			//�Ʒ�תʱ��
#define KEYSCAN_TIME    1 		   			//����ɨ��ʱ������ΪAPPSCAN_TIME ms 
#define DISSHAKE_TIME   0//10		   			//����ʱ������ΪDISSHAKE_TIME ms
#define SENDKEY_TIME    1		   			//���ڷ���KEY״̬ʱ��ΪSENDKEY_TIME ms

#define CANTXKEYID   	0x20				//������ⷢ��ID
#define CANTEKID_PLUS 0X21
#define CANRXSERVOID	0x40	   			//�������ID
#define CANRXVALVEID    0x30	  	 		//��������ID
#define CANRXDT35ID     0X50                //DT35����ID


extern volatile u16  ucLedCnt ;				//LED����˸����
extern volatile u16  ucTimDlyCntms ;		//SysTickӲ���жϼ���
extern volatile u16  ucTimDlyCntus ;		//SysTickӲ���жϼ���
extern volatile u8   ucKeyScanFlag  ;   	//��������־
extern volatile u16  usSendKeyPer  ; 		//���ڷ��Ͱ���״̬����


extern u8   aucKeyScanCnt[12]  ;			//����ɨ�����
extern u8   aucDisShakeCnt[12] ;			//����ɨ�����
extern u8   aucDisShakeFlag[12];			//������־λ
extern u8   aucKeyStatTemp[12];				//������ʱ�洢ֵ


extern u16   usKeyStat ;					//�������ε�ֵ

extern u16   ucValvePreStat ;				//�����ϴε�ֵ

extern u8    aucServoPreStat[4] ; 			//��һ�εĶ��״̬

extern u8    aucCanRxData[8] ;				//���յ�������

#endif

