#include "STM32Lib\\stm32f10x.h"
#include "main.h"
#include "math.h"

CanRxMsg st_RxMessage;

volatile u16  ucLedCnt = 0 ;		//LED����˸����
volatile u16  ucTimDlyCntms = 0 ;	//systick�жϺ������	
volatile u16  ucTimDlyCntus = 0 ;	//systick�ж�΢�����	
volatile u16  count_thousand = 0;   //systick�ж���ͼ���λ��ÿ��1000���㣬systick�ж�1us����һ��
volatile u8   ucKeyScanFlag = 0 ;   //��������־
volatile u16  usSendKeyPer = 0 ;    //���ڷ��Ͱ�����ʶ

u8   aucKeyScanCnt[12] = {0} ;		//����ɨ�����
u8   aucDisShakeCnt[12] = {0} ;		//����ɨ�����
u8   aucDisShakeFlag[12] = {0};		//������־λ
u8   aucKeyStatTemp[12]={0};		//����״̬��ʱ�洢
u16  usKeyStat = 0 ;				//����״ֵ̬

u16  ucValvePreStat = 0x0000 ;					//��һ�ε�����״̬
u8   aucServoPreStat[4] = {100,100,100,100}; 	//��һ�εĶ��״̬
 
u8	aucCanRxData[8] = {0};			    //���յ�������

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
	
	ChipHalInit();			//Ƭ��Ӳ����ʼ��
//	ValveAllStart();//�ر���������
	SysTickDelayms(500);      //��ʱ100ms�ٿ�ʼ���ж���
//	ValveAllClose();
	while(1)
	{
		/*����˸,֤����������*/
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
			
			SendDT35(ZKID, ket5, ket3, ket1, ket4);				//�������ر����õ�ID
//			SysTickDelayms(1);
//			SendDT35(ZKID + 1, ket1, 0, 0, 0);
		
			m++;
	 }			
}


u8 ucKeyNum;		//�������0-15
void Scan_key(void)
{
	if( aucDisShakeFlag[ucKeyNum] == 0 )				//����ɨ��״̬��
		{
			if( aucKeyScanCnt[ucKeyNum] >= KEYSCAN_TIME )	   	//����һ��ɨ���ʱ��
			{
				if( ((usKeyStat>>ucKeyNum)&1)!= ScanKey((ucKeyNum)) )	 	//���ԭ��״̬����״̬��һ��
				{
					aucKeyStatTemp[ucKeyNum] = ScanKey(ucKeyNum) ;			//��ʱ���Ӧ����ֵ
					aucDisShakeFlag[ucKeyNum] = 1;				   			//������־λ��Ϊ1
				}
				else								 						//���ԭ��״̬����״̬һ��
				{															//ʲô������
				}
				aucKeyScanCnt[ucKeyNum] = 0;	   					//����ֵ���㣬���¿�ʼ�������ȴ���һ��ɨ��
			}
			aucKeyScanCnt[ucKeyNum] ++;			   				//�������ɨ��ʱ�����								
		}
		else												//����״̬��
		{
	   		if(aucDisShakeCnt[ucKeyNum] >= DISSHAKE_TIME)	   	//��������ʱ��
			{
				if( aucKeyStatTemp[ucKeyNum] == ScanKey(ucKeyNum) )		//��һ״̬����״̬һ��
				{
					usKeyStat ^= (1<<ucKeyNum) ;				   		//��Ӧλ��״̬��ת				 	ucKeyStat
					SendUSHORT16ByCan(CANTXKEYID,usKeyStat);			//������������⵽��ֵͬ��ͨ��CAN����
				}
				else	   												//��һ״̬����״̬��һ�£���ʾΪ����
				{		   												//ʲô������
				}
				aucDisShakeFlag[ucKeyNum] = 0;						//��־λ��0�����¿�ʼ����ɨ��
				aucDisShakeCnt[ucKeyNum] = 0;						//����ֵ���㣬���¿�ʼ�������ȴ���һ������						
			}
			aucDisShakeCnt[ucKeyNum] ++;			   			//����ʱ�����			
		}
}

u8 shiyan;
void SendKeyStat(void)
{
//	/*��ⲻֵͬ ��ͨ��can����*/
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
////		ucKeyScanFlag = 0 ;	 //��־λ��Ϊ0���´ν����жϺ�ſ�ִ�д˺���
////   }

	/*��ⲻֵͬ ��ͨ��can����*/
  if( ucKeyScanFlag == 1 )
  {
	for( ucKeyNum = 0 ; ucKeyNum < 7 ; ucKeyNum++ )
	{
		if( aucDisShakeFlag[ucKeyNum] == 0 )				//����ɨ��״̬��
		{
			if( aucKeyScanCnt[ucKeyNum] >= KEYSCAN_TIME )	   	//����һ��ɨ���ʱ��
			{
				if( ((usKeyStat>>ucKeyNum)&1)!= ScanKey((ucKeyNum)) )	 	//���ԭ��״̬����״̬��һ��
				{
					aucKeyStatTemp[ucKeyNum] = ScanKey(ucKeyNum) ;			//��ʱ���Ӧ����ֵ
					aucDisShakeFlag[ucKeyNum] = 1;				   			//������־λ��Ϊ1
				}
				else								 						//���ԭ��״̬����״̬һ��
				{															//ʲô������
				}
				aucKeyScanCnt[ucKeyNum] = 0;	   					//����ֵ���㣬���¿�ʼ�������ȴ���һ��ɨ��
			}
			aucKeyScanCnt[ucKeyNum] ++;			   				//�������ɨ��ʱ�����								
		}
		else												//����״̬��
		{
	   		if(aucDisShakeCnt[ucKeyNum] >= DISSHAKE_TIME)	   	//��������ʱ��
			{
				if( aucKeyStatTemp[ucKeyNum] == ScanKey(ucKeyNum) )		//��һ״̬����״̬һ��
				{
					usKeyStat ^= (1<<ucKeyNum) ;				   		//��Ӧλ��״̬��ת				 	ucKeyStat
					SendUSHORT16ByCan(CANTXKEYID,usKeyStat);			//������������⵽��ֵͬ��ͨ��CAN����
				}
				else	   												//��һ״̬����״̬��һ�£���ʾΪ����
				{		   												//ʲô������
				}
				aucDisShakeFlag[ucKeyNum] = 0;						//��־λ��0�����¿�ʼ����ɨ��
				aucDisShakeCnt[ucKeyNum] = 0;						//����ֵ���㣬���¿�ʼ�������ȴ���һ������						
			}
			aucDisShakeCnt[ucKeyNum] ++;			   			//����ʱ�����			
		}
	 }
	 ucKeyScanFlag = 0 ;	 //��־λ��Ϊ0���´ν����жϺ�ſ�ִ�д˺���
}
}

