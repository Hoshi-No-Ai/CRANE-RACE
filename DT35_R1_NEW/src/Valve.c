#include "STM32Lib\\stm32f10x.h"
#include "main.h"
u16 TempValveStat;
void ValveCtrl(void)
{
	if((TempValveStat&VALVE1) == VALVE1)
	{
		VALVE_1_OPEN;
	}
	else
	{
		VALVE_1_CLOSE;
	}
	
	if((TempValveStat&VALVE2) == VALVE2)
	{
		VALVE_2_OPEN;
	}
	else
	{
		VALVE_2_CLOSE;
	}	
	
	if((TempValveStat&VALVE3) == VALVE3)
	{
		VALVE_3_OPEN;
	}
	else
	{
		VALVE_3_CLOSE;
	}	
	
	if((TempValveStat&VALVE4) == VALVE4)
	{
		VALVE_4_OPEN;
	}
	else
	{
		VALVE_4_CLOSE;
	}	
	
	if((TempValveStat&VALVE5) == VALVE5)
	{
		VALVE_5_OPEN;
	}
	else
	{
		VALVE_5_CLOSE;
	}	
	
	if((TempValveStat&VALVE6) == VALVE6)
	{
		VALVE_6_OPEN;
	}
	else
	{
		VALVE_6_CLOSE;
	}	
	
	if((TempValveStat&VALVE7) == VALVE7)
	{
		VALVE_7_OPEN;
	}
	else
	{
		VALVE_7_CLOSE;
	}	
	
	if((TempValveStat&VALVE8) == VALVE8)
	{
		VALVE_8_OPEN;
	}
	else
	{
		VALVE_8_CLOSE;
	}
	/*今年没有用到*/
//if((TempValveStat&VALVE9) == VALVE9)
//	{
//		VALVE_9_OPEN;
//	}
//	else
//	{
//		VALVE_9_CLOSE;
//	}
//	
//		if((TempValveStat&VALVE10) == VALVE10)
//	{
//		VALVE_10_OPEN;
//	}
//	else
//	{
//		VALVE_10_CLOSE;
//	}	
//	
//		if((TempValveStat&VALVE11) == VALVE11)
//	{
//		VALVE_11_OPEN;
//	}
//	else
//	{
//		VALVE_11_CLOSE;
//	}	
//	
//		if((TempValveStat&VALVE12) == VALVE12)
//	{
//		VALVE_12_OPEN;
//	}
//	else
//	{
//		VALVE_12_CLOSE;
//	}	
//	
//		if((TempValveStat&VALVE13) == VALVE13)
//	{
//		VALVE_13_OPEN;
//	}
//	else
//	{
//		VALVE_13_CLOSE;
//	}	
//	
//		if((TempValveStat&VALVE14) == VALVE14)
//	{
//		VALVE_14_OPEN;
//	}
//	else
//	{
//		VALVE_14_CLOSE;
//	}	
//	
//		if((TempValveStat&VALVE15) == VALVE15)
//	{
//		VALVE_15_OPEN;
//	}
//	else
//	{
//		VALVE_15_CLOSE;
//	}	
//	
//		if((TempValveStat&VALVE16) == VALVE16)
//	{
//		VALVE_16_OPEN;
//	}
//	else
//	{
//		VALVE_16_CLOSE;
//	}				
}

void ValveAllStart(void)
{
	VALVE_1_OPEN;
	VALVE_2_OPEN;
	VALVE_3_OPEN;
	VALVE_4_OPEN;
	VALVE_5_OPEN;
	VALVE_6_OPEN;
	VALVE_7_OPEN;
	VALVE_8_OPEN;
	VALVE_9_OPEN;
	VALVE_10_OPEN;
	VALVE_11_OPEN;
	VALVE_12_OPEN;
	VALVE_13_OPEN;
	VALVE_14_OPEN;
	VALVE_15_OPEN;
	VALVE_16_OPEN;
}
void ValveAllClose(void)
{
	VALVE_1_CLOSE;
	VALVE_2_CLOSE;
	VALVE_3_CLOSE;
	VALVE_4_CLOSE;
	VALVE_5_CLOSE;
	VALVE_6_CLOSE;
	VALVE_7_CLOSE;
	VALVE_8_CLOSE;
	VALVE_9_CLOSE;
	VALVE_10_CLOSE;
	VALVE_11_CLOSE;
	VALVE_12_CLOSE;
	VALVE_13_CLOSE;
	VALVE_14_CLOSE;
	VALVE_15_CLOSE;
	VALVE_16_CLOSE;
}
