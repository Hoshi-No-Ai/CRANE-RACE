#include "STM32Lib\\stm32f10x.h"
#include "main.h"


u8 ScanKey(u8 KeyNum)
{
	u8 Stat;
	switch(KeyNum)
	{
	 	case 0:
			Stat = GET_KEY1;
		break;
	 	case 1:
			Stat = GET_KEY2;
		break;
	 	case 2:
			Stat = GET_KEY3;
		break;
	 	case 3:
			Stat = GET_KEY4;
		break;
	 	case 4:
			Stat = GET_KEY5;
		break;
	 	case 5:
			Stat = GET_KEY6;
		break;
	 	case 6:
			Stat = GET_KEY7;
		break;
	 	case 7:
			Stat = GET_KEY8;
		break;
	 	case 8:
			Stat = GET_KEY9;
		break;
	 	case 9:
			Stat = GET_KEY10;
		break;
	 	case 10:
			Stat = GET_KEY11;
		break;
	 	case 11:
			Stat = GET_KEY12;
		break;
		default:
		break;	
	}
	return Stat;
}


