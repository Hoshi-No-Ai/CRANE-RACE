#include "motor_drive.h"

/*************************************************************************
函 数 名：GetEncoderNumber
函数功能：接收6623、C620/C610返回的机械角度值（绝对式编码器值）
备    注：机械角度值范围：0~8191（0x1FFF）
*************************************************************************/
int32_t Get_Encoder_Number(CanRxMsg* rx_message)
{
  int32_t encoder_temp;
	encoder_temp = rx_message->Data[0]<<8 | rx_message->Data[1];
	return encoder_temp;
}

/************************************************************************
函 数 名：Get_Speed
函数功能：接收C620/C610返回的转速，单位：r/min
备    注：RM3508电机减速比为1：19；M2006电机减速比为1：36
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
函 数 名：Encoder_Process
函数功能：有刷电机增量式编码器及RM3510电机绝对式编码器数据处理，得到转速
备    注：type:0--绝对式编码器；1--增量式编码器
*************************************************************************/
void cEncoder::Encoder_Process(uint8_t type)
{
  static fp32 fpVeltCoff;


	this->siDiff = this->siRawValue - this->siPreRawValue;		
	
	if(this->siDiff < -this->siNumber/2)//两次编码器的反馈值差别太大,表示绝对式编码器圈数发生了改变或增量式编码器的定时器计数器向上溢出
	{
		(type == 0)? (this->siDiff += (this->siNumber+1)):(this->siDiff += 65536);//定时器16位计数器计数范围0-65535共65536个数
	}
	else if(this->siDiff > this->siNumber/2)//两次编码器的反馈值差别太大,表示绝对式编码器圈数发生了改变或增量式编码器的定时器计数器向下溢出
	{
		(type == 0)? (this->siDiff -= (this->siNumber+1)):(this->siDiff -= 65536);
	}

	if(type == 0)
	{
	    fpVeltCoff = 60.0f / this->siGearRatio / this->siNumber / 0.001f;//0.001是指两次采样间隔1ms
	}
	else
	{
	    fpVeltCoff = 60.0f / this->siGearRatio / this->siNumber / 0.001f / 4.0f;//0.001是指两次采样间隔1ms,4是指定时器编码器模式的4倍频
	}		
	this->siPreRawValue = this->siRawValue;
	this->fpSpeed = fpVeltCoff*this->siDiff;//单位：r/min
	this->siSumValue += this->siDiff;//记录编码器的总数，位置闭环用	 
  if(this->motor_flag==0)
	
	{
		this->siSumValue=0;
		this->motor_flag=1;
		
	}		
}




/*can发送函数*/
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
函 数 名：Get_HT-04_Position
函数功能：获得HT-04的编码器累加值
备    注：
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
函 数 名：Get_HT-04_Speed
函数功能：获得HT-04的速度反馈
备    注：
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
函 数 名：Get_HT-Current
函数功能：获得HT-04的电流反馈
备    注：
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
函 数 名：CanComm_SendControlPara
函数功能：发送参数
备    注：
*************************************************************************/

void CanComm_SendControlPara(CAN_TypeDef* CANx, float f_p, float f_v, float f_kp, float f_kd, float f_t,uint32_t StdId,bool type)
{
    uint16_t p, v, kp, kd, t;
		CanTxMsg tx_message;
    if(type == HT_03)
		{
			/* 限制输入的参数在定义的范围内 */
    LIMIT_MIN_MAX(f_p,  P_MIN,  P_MAX);
    LIMIT_MIN_MAX(f_v,  V_MIN,  V_MAX);
    LIMIT_MIN_MAX(f_kp, KP_MIN, KP_MAX);
    LIMIT_MIN_MAX(f_kd, KD_MIN, KD_MAX);
    LIMIT_MIN_MAX(f_t,  T_MIN,  T_MAX);
    
    /* 根据协议，对float参数进行转换 */
    p = float_to_uint(f_p,      P_MIN,  P_MAX,  16);            
    v = float_to_uint(f_v,      V_MIN,  V_MAX,  12);
    kp = float_to_uint(f_kp,    KP_MIN, KP_MAX, 12);
    kd = float_to_uint(f_kd,    KD_MIN, KD_MAX, 12);
    t = float_to_uint(f_t,      T_MIN,  T_MAX,  12);
		}
    else
		{
			 /* 限制输入的参数在定义的范围内 */
    LIMIT_MIN_MAX(f_p,  P_MIN2,  P_MAX2);
    LIMIT_MIN_MAX(f_v,  V_MIN2,  V_MAX2);
    LIMIT_MIN_MAX(f_kp, KP_MIN2, KP_MAX2);
    LIMIT_MIN_MAX(f_kd, KD_MIN2, KD_MAX2);
    LIMIT_MIN_MAX(f_t,  T_MIN2,  T_MAX2);
    
    /* 根据协议，对float参数进行转换 */
    p = float_to_uint(f_p,      P_MIN2,  P_MAX2,  16);            
    v = float_to_uint(f_v,      V_MIN2,  V_MAX2,  12);
    kp = float_to_uint(f_kp,    KP_MIN2, KP_MAX2, 12);
    kd = float_to_uint(f_kd,    KD_MIN2, KD_MAX2, 12);
    t = float_to_uint(f_t,      T_MIN2,  T_MAX2,  12);
		}
    
    /* 根据传输协议，把数据转换为CAN命令数据字段 */
    tx_message.Data[0] = p>>8;
    tx_message.Data[1] = p&0xFF;
    tx_message.Data[2] = v>>4;
    tx_message.Data[3] = ((v&0xF)<<4)|(kp>>8);
    tx_message.Data[4] = kp&0xFF;
    tx_message.Data[5] = kd>>4;
    tx_message.Data[6] = ((kd&0xF)<<4)|(t>>8);
    tx_message.Data[7] = t&0xff;
    
    /* 通过CAN接口把buf中的内容发送出去 */
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
			return; /* 直接退出函数 */
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



