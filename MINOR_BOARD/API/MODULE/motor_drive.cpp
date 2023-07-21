#include "motor_drive.h"

/*************************************************************************
�� �� ����GetEncoderNumber
�������ܣ�����6623��C620/C610���صĻ�е�Ƕ�ֵ������ʽ������ֵ��
��    ע����е�Ƕ�ֵ��Χ��0~8191��0x1FFF��
*************************************************************************/
int32_t Get_Encoder_Number(CanRxMsg* rx_message)
{
  int32_t encoder_temp;
	encoder_temp = rx_message->Data[0]<<8 | rx_message->Data[1];
	return encoder_temp;
}

/************************************************************************
�� �� ����Get_Speed
�������ܣ�����C620/C610���ص�ת�٣���λ��r/min
��    ע��RM3508������ٱ�Ϊ1��19��M2006������ٱ�Ϊ1��36
*************************************************************************/
int32_t Get_Speed(CanRxMsg* rx_message)
{
  int32_t speed_temp;
	if(rx_message->Data[2] & 0x01<<7)
  {	
     speed_temp = 0xFFFF0000 | (rx_message->Data[2]<<8 | rx_message->Data[3]);
  }
  else
     speed_temp = (rx_message->Data[2]<<8 | rx_message->Data[3]);//rpm
	return speed_temp;
}


/*************************************************************************
�� �� ����Encoder_Process
�������ܣ���ˢ�������ʽ��������RM3510�������ʽ���������ݴ����õ�ת��
��    ע��type:0--����ʽ��������1--����ʽ������
*************************************************************************/
void cEncoder::Encoder_Process(uint8_t type)
{
  static fp32 fpVeltCoff;


	this->siDiff = this->siRawValue - this->siPreRawValue;		
	
	if(this->siDiff < -this->siNumber/2)//���α������ķ���ֵ���̫��,��ʾ����ʽ������Ȧ�������˸ı������ʽ�������Ķ�ʱ���������������
	{
		(type == 0)? (this->siDiff += (this->siNumber+1)):(this->siDiff += 65536);//��ʱ��16λ������������Χ0-65535��65536����
	}
	else if(this->siDiff > this->siNumber/2)//���α������ķ���ֵ���̫��,��ʾ����ʽ������Ȧ�������˸ı������ʽ�������Ķ�ʱ���������������
	{
		(type == 0)? (this->siDiff -= (this->siNumber+1)):(this->siDiff -= 65536);
	}

	if(type == 0)
	{
	    fpVeltCoff = 60.0f / this->siGearRatio / this->siNumber / 0.001f;//0.001��ָ���β������1ms
	}
	else
	{
	    fpVeltCoff = 60.0f / this->siGearRatio / this->siNumber / 0.001f / 4.0f;//0.001��ָ���β������1ms,4��ָ��ʱ��������ģʽ��4��Ƶ
	}		
	this->siPreRawValue = this->siRawValue;
	this->fpSpeed = fpVeltCoff*this->siDiff;//��λ��r/min
	this->siSumValue += this->siDiff;//��¼��������������λ�ñջ���	 
  if(this->motor_flag==0)
	
	{
		this->siSumValue=0;
		this->motor_flag=1;
		
	}		
}




/*can���ͺ���*/
void can_send_data(CAN_TypeDef* CANx, uint32_t StdID, int16_t ssMotor1, int16_t ssMotor2, 
															int16_t ssMotor3, int16_t ssMotor4)
{
	CanTxMsg tx_message;
	
	tx_message.StdId = StdID;
	tx_message.IDE = CAN_Id_Standard;
	tx_message.RTR = CAN_RTR_Data;
	tx_message.DLC = 0x08;
	tx_message.Data[0] = ssMotor1>>8; 	             	
	tx_message.Data[1] = ssMotor1;
	tx_message.Data[2] = ssMotor2>>8;               
	tx_message.Data[3] = ssMotor2;        
	
	tx_message.Data[4] = ssMotor3>>8;               
	tx_message.Data[5] = ssMotor3; 
	tx_message.Data[6] = ssMotor4>>8;               
	tx_message.Data[7] = ssMotor4; 
	
	CAN_Transmit(CANx, &tx_message);
	

}

/*************************************************************************
�� �� ����Get_HT-04_Position
�������ܣ����HT-04�ı������ۼ�ֵ
��    ע��
*************************************************************************/
float Get_HT_04_Position(CanRxMsg* rx_message,bool type)
{
	u16 encoder_tp;
	float encoder_temp;
	encoder_tp = rx_message->Data[1]<<8 | rx_message->Data[2];
	if( type == HT_03)
	{
		encoder_temp = uint_to_float(encoder_tp, P_MIN, P_MAX, 16);
	}
	else
	{
		encoder_temp = uint_to_float(encoder_tp, P_MIN2, P_MAX2, 16);
	}
	return encoder_temp;
}

/*************************************************************************
�� �� ����Get_HT-04_Speed
�������ܣ����HT-04���ٶȷ���
��    ע��
*************************************************************************/
float Get_HT_04_Speed(CanRxMsg* rx_message,bool type)
{
	u16 speed_tp;
  float speed_temp;
	speed_tp = (rx_message->Data[3]<<4) | (rx_message->Data[4]>>4);
	if(type == HT_03)
	{
			speed_temp = uint_to_float(speed_tp, V_MIN, V_MAX, 12);
	}
	else
	{
			speed_temp = uint_to_float(speed_tp, V_MIN2, V_MAX2, 12);
	}

	return speed_temp;
}
/*************************************************************************
�� �� ����Get_HT-Current
�������ܣ����HT-04�ĵ�������
��    ע��
*************************************************************************/
int16_t Get_HT_04_Current(CanRxMsg* rx_message)
{
  int16_t Current_tp;
	float Current_temp;
	Current_tp = 0x0FFF & (rx_message->Data[4]<<8 | rx_message->Data[5]);
	Current_temp = uint_to_float(Current_tp, -40, 40, 12);
	return Current_temp;
}
/*************************************************************************
�� �� ����CanComm_SendControlPara
�������ܣ����Ͳ���
��    ע��
*************************************************************************/

void CanComm_SendControlPara(CAN_TypeDef* CANx, float f_p, float f_v, float f_kp, float f_kd, float f_t,uint32_t StdId,bool type)
{
    uint16_t p, v, kp, kd, t;
		CanTxMsg tx_message;
    if(type == HT_03)
		{
			/* ��������Ĳ����ڶ���ķ�Χ�� */
    LIMIT_MIN_MAX(f_p,  P_MIN,  P_MAX);
    LIMIT_MIN_MAX(f_v,  V_MIN,  V_MAX);
    LIMIT_MIN_MAX(f_kp, KP_MIN, KP_MAX);
    LIMIT_MIN_MAX(f_kd, KD_MIN, KD_MAX);
    LIMIT_MIN_MAX(f_t,  T_MIN,  T_MAX);
    
    /* ����Э�飬��float��������ת�� */
    p = float_to_uint(f_p,      P_MIN,  P_MAX,  16);            
    v = float_to_uint(f_v,      V_MIN,  V_MAX,  12);
    kp = float_to_uint(f_kp,    KP_MIN, KP_MAX, 12);
    kd = float_to_uint(f_kd,    KD_MIN, KD_MAX, 12);
    t = float_to_uint(f_t,      T_MIN,  T_MAX,  12);
		}
    else
		{
			 /* ��������Ĳ����ڶ���ķ�Χ�� */
    LIMIT_MIN_MAX(f_p,  P_MIN2,  P_MAX2);
    LIMIT_MIN_MAX(f_v,  V_MIN2,  V_MAX2);
    LIMIT_MIN_MAX(f_kp, KP_MIN2, KP_MAX2);
    LIMIT_MIN_MAX(f_kd, KD_MIN2, KD_MAX2);
    LIMIT_MIN_MAX(f_t,  T_MIN2,  T_MAX2);
    
    /* ����Э�飬��float��������ת�� */
    p = float_to_uint(f_p,      P_MIN2,  P_MAX2,  16);            
    v = float_to_uint(f_v,      V_MIN2,  V_MAX2,  12);
    kp = float_to_uint(f_kp,    KP_MIN2, KP_MAX2, 12);
    kd = float_to_uint(f_kd,    KD_MIN2, KD_MAX2, 12);
    t = float_to_uint(f_t,      T_MIN2,  T_MAX2,  12);
		}
    
    /* ���ݴ���Э�飬������ת��ΪCAN���������ֶ� */
    tx_message.Data[0] = p>>8;
    tx_message.Data[1] = p&0xFF;
    tx_message.Data[2] = v>>4;
    tx_message.Data[3] = ((v&0xF)<<4)|(kp>>8);
    tx_message.Data[4] = kp&0xFF;
    tx_message.Data[5] = kd>>4;
    tx_message.Data[6] = ((kd&0xF)<<4)|(t>>8);
    tx_message.Data[7] = t&0xff;
    
    /* ͨ��CAN�ӿڰ�buf�е����ݷ��ͳ�ȥ */
		tx_message.StdId = StdId;
		tx_message.IDE = CAN_Id_Standard;
		tx_message.RTR = CAN_RTR_Data;
		tx_message.DLC = sizeof(tx_message.Data);
    CAN_Transmit(CANx, &tx_message);
}




void CanComm_ControlCmd(CAN_TypeDef* CANx, uint8_t cmd,uint32_t StdId)
{
	CanTxMsg tx_message;
	
	for(int i=0;i<7;i++)
	{
		tx_message.Data[i] = 0xFF;
	}

	switch(cmd)
	{
			case CMD_MOTOR_MODE:
					tx_message.Data[7] = 0xFC;
					break;
			
			case CMD_RESET_MODE:
					tx_message.Data[7] = 0xFD;
			break;
			
			case CMD_ZERO_POSITION:
					tx_message.Data[7] = 0xFE;
			break;
			
			default:
			return; /* ֱ���˳����� */
	}
	
	tx_message.StdId = StdId;
	tx_message.IDE = CAN_Id_Standard;
	tx_message.RTR = CAN_RTR_Data;
	tx_message.DLC = sizeof(tx_message.Data);
  CAN_Transmit(CANx, &tx_message);
}






void zero_position_command(CAN_TypeDef* CANx, uint8_t motorId)
{
	CanTxMsg tx_message;
		OS_ERR err;
	
	tx_message.StdId = DEVICE_STD_ID + motorId;
	tx_message.IDE = CAN_Id_Standard;
	tx_message.RTR = CAN_RTR_Data;
	tx_message.DLC = 0x08;
	tx_message.Data[0] = 0x19; 	             	
	tx_message.Data[1] = 0x00;
	tx_message.Data[2] = 0x00;               
	tx_message.Data[3] = 0x00;        
	
	tx_message.Data[4] = 0x00;               
	tx_message.Data[5] = 0x00; 
	tx_message.Data[6] = 0x00;               
	tx_message.Data[7] = 0x00; 
	
	CAN_Transmit(CANx, &tx_message);
	OSTimeDly_ms(200);
	
}


void multi_position_command(CAN_TypeDef* CANx, uint8_t motorId, int32_t  angleControl)
{
	CanTxMsg tx_message;
		OS_ERR err;
	tx_message.StdId = DEVICE_STD_ID + motorId;
	tx_message.IDE = CAN_Id_Standard;
	tx_message.RTR = CAN_RTR_Data;
	tx_message.DLC = 0x08;
	tx_message.Data[0] = 0xA3; 	             	
	tx_message.Data[1] = 0x00;
	tx_message.Data[2] = 0x00;               
	tx_message.Data[3] = 0x00;        
	
	tx_message.Data[4] =  *(uint8_t *)(&angleControl);               
	tx_message.Data[5] =  *((uint8_t *)(&angleControl)+1); 
	tx_message.Data[6] =  *((uint8_t *)(&angleControl)+2);               
	tx_message.Data[7] = *((uint8_t *)(&angleControl)+3); 
	
	CAN_Transmit(CANx, &tx_message);
	OSTimeDly_ms(200);
  rate_monitor.temp_rate[4]++;
	
}

void run_command(CAN_TypeDef* CANx, uint8_t motorId)
{
	CanTxMsg tx_message;
		OS_ERR err;
	
	tx_message.StdId = DEVICE_STD_ID + motorId;
	tx_message.IDE = CAN_Id_Standard;
	tx_message.RTR = CAN_RTR_Data;
	tx_message.DLC = 0x08;
	tx_message.Data[0] = 0x88; 	             	
	tx_message.Data[1] = 0x00;
	tx_message.Data[2] = 0x00;               
	tx_message.Data[3] = 0x00;        
	
	tx_message.Data[4] = 0x00;               
	tx_message.Data[5] = 0x00; 
	tx_message.Data[6] = 0x00;               
	tx_message.Data[7] = 0x00; 
	
	CAN_Transmit(CANx, &tx_message);
	OSTimeDly_ms(200);
	 rate_monitor.temp_rate[3]++;
	
}

void clear_command(CAN_TypeDef* CANx, uint8_t motorId)
{
	CanTxMsg tx_message;
		OS_ERR err;
	
	tx_message.StdId = DEVICE_STD_ID + motorId;
	tx_message.IDE = CAN_Id_Standard;
	tx_message.RTR = CAN_RTR_Data;
	tx_message.DLC = 8;
	tx_message.Data[0] = 0x9B; 	             	
	tx_message.Data[1] = 0x00;
	tx_message.Data[2] = 0x00;               
	tx_message.Data[3] = 0x00;        
	
	tx_message.Data[4] = 0x00;               
	tx_message.Data[5] = 0x00; 
	tx_message.Data[6] = 0x00;               
	tx_message.Data[7] = 0x00; 
	
	CAN_Transmit(CANx, &tx_message);
	OSTimeDly_ms(200);
  rate_monitor.temp_rate[2]++;
	
}



void get_position_command(CAN_TypeDef* CANx, uint8_t motorId)
{
	CanTxMsg tx_message;
		OS_ERR err;
	
	tx_message.StdId = DEVICE_STD_ID + motorId;
	tx_message.IDE = CAN_Id_Standard;
	tx_message.RTR = CAN_RTR_Data;
	tx_message.DLC = 0x08;
	tx_message.Data[0] = 0x88; 	             	
	tx_message.Data[1] = 0x00;
	tx_message.Data[2] = 0x00;               
	tx_message.Data[3] = 0x00;        
	
	tx_message.Data[4] = 0x00;               
	tx_message.Data[5] = 0x00; 
	tx_message.Data[6] = 0x00;               
	tx_message.Data[7] = 0x00; 
	
	CAN_Transmit(CANx, &tx_message);
	OSTimeDly_ms(200);

	
}



 int64_t get_position(CanRxMsg* rx_message)
{
	 int64_t encoder_pos;
	 encoder_pos = rx_message->Data[1] | rx_message->Data[2]<<8 | rx_message->Data[3]<<16 | rx_message->Data[4]<<24 | rx_message->Data[5]<<40 |rx_message->Data[6]<<48 | rx_message->Data[7]<< 56;
	 return encoder_pos;
	 
}



