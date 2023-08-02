#include "flash.h"

u8 g_ucFlashFlag;
u8 g_ucFlashWrongNum;
s32 g_siFlashValue[MAX_FLASH_LEN];

union FLASHSAVE
{
	fp32 g_fpFlashValue[MAX_FLASH_LEN];
	u8 g_ucFlashValue[1600];
} FlashSave;

union FLASHREAD
{
	fp32 fpData;
	u8 ucData[4];
} FlashRead;

fp32 GetOneFP32(u32 uiReadAddr)
{
	s32 i;
	u8 ucTemp;
	for (i = 0; i < 4; i++)
	{
		ucTemp = (u8)(*(__IO u8 *)(ADDR_FLASH_SECTOR_11 + (uiReadAddr << 2) + i));
		FlashRead.ucData[i] = ucTemp;
	}
	return FlashRead.fpData;
}

// 读取一个16位数据
SSHORT16 GetOneShort16(u32 uiReadAddr)
{
	SSHORT16 ssTemp;
	ssTemp = (SSHORT16)(*(__IO s32 *)(ADDR_FLASH_SECTOR_11 + (uiReadAddr << 2)));
	return ssTemp;
}

// 读取一个32位数据
s32 GetOneInt32(u32 uiReadAddr)
{
	s32 ssTemp;
	ssTemp = *(__IO s32 *)(ADDR_FLASH_SECTOR_11 + (uiReadAddr << 2));
	return ssTemp;
}

// 向指定地址写入一个32位数据
u8 SaveOneWord32(s32 siData, u32 uiWriteAddr)
{
	u8 ucRet = 0;
	u32 i = 0;
	/* Unlock the Flash to enable the flash control register access *************/
	FLASH_Unlock();
	// 写之前先读取数据
	for (i = 0; i < MAX_FLASH_LEN; i++)
	{
		g_siFlashValue[i] = *(__IO s32 *)(ADDR_FLASH_SECTOR_11 + (i << 2));
	}

	/* Clear pending flags (if any) */
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
					FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

	/* Device voltage range supposed to be [2.7V to 3.6V], the operation will
			be done by word */
	if (FLASH_EraseSector(FLASH_Sector_11, VoltageRange_3) != FLASH_COMPLETE)
	{
		/* Error occurred while sector erase.
			User can add here some code to deal with this error  */
		return 0;
	}

	g_siFlashValue[uiWriteAddr] = siData;

	for (i = 0; i < MAX_FLASH_LEN; i++)
	{
		if (FLASH_ProgramWord(ADDR_FLASH_SECTOR_11 + (i << 2), g_siFlashValue[i]) == FLASH_COMPLETE)
		{
			ucRet = 0x01;
		}
		else
		{
			ucRet = 0x00;
			g_ucFlashWrongNum++;
		}
	}

	/* Lock the Flash to disable the flash control register access (recommended
	  to protect the FLASH memory against possible unwanted operation) *********/
	FLASH_Lock();
	g_ucFlashFlag = ucRet;
	return ucRet;
}

// 从指定地址开始写入一个32位数据的数组
u8 SaveParaWord32(u32 uiLength, u32 uiFlashAddr, s32 *p)
{
	u8 ucRet = 0;
	u32 i = 0;

	/* Unlock the Flash to enable the flash control register access *************/
	FLASH_Unlock();
	// 写入之前先读flash
	for (i = 0; i < MAX_FLASH_LEN; i++)
	{
		g_siFlashValue[i] = *(__IO s32 *)(ADDR_FLASH_SECTOR_11 + (i << 2));
	}

	/* Clear pending flags (if any) */
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
					FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

	/* Device voltage range supposed to be [2.7V to 3.6V], the operation will
	   be done by word */
	if (FLASH_EraseSector(FLASH_Sector_11, VoltageRange_3) != FLASH_COMPLETE)
	{
		/* Error occurred while sector erase.
		   User can add here some code to deal with this error  */
		return 0;
	}

	for (i = 0; i < uiLength; i++)
	{
		g_siFlashValue[uiFlashAddr + i] = p[i];
	}

	for (i = 0; i < MAX_FLASH_LEN; i++)
	{
		if (FLASH_ProgramWord(ADDR_FLASH_SECTOR_11 + (i << 2), g_siFlashValue[i]) == FLASH_COMPLETE)
		{

			ucRet = 0x01;
		}
		else
		{
			ucRet = 0x00;
			g_ucFlashWrongNum++;
		}
	}

	/* Lock the Flash to disable the flash control register access (recommended
	  to protect the FLASH memory against possible unwanted operation) *********/
	FLASH_Lock();
	g_ucFlashFlag = ucRet;
	return ucRet;
}

// 将数组中的数据全部写入Flash
u8 SaveAllWord32(void)
{
	u8 ucRet = 0;
	u32 i = 0;
	/* Unlock the Flash to enable the flash control register access *************/
	FLASH_Unlock();

	/* Clear pending flags (if any) */
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
					FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

	/* Device voltage range supposed to be [2.7V to 3.6V], the operation will
	   be done by word */
	if (FLASH_EraseSector(FLASH_Sector_11, VoltageRange_3) != FLASH_COMPLETE)
	{
		/* Error occurred while sector erase.
		   User can add here some code to deal with this error  */
		return 0;
	}

	for (i = 0; i < MAX_FLASH_LEN; i++)
	{
		if (FLASH_ProgramWord(ADDR_FLASH_SECTOR_11 + (i << 2), g_siFlashValue[i]) == FLASH_COMPLETE)
		{

			ucRet = 0x01;
		}
		else
		{
			ucRet = 0x00;
			g_ucFlashWrongNum++;
		}
	}

	/* Lock the Flash to disable the flash control register access (recommended
	  to protect the FLASH memory against possible unwanted operation) *********/
	FLASH_Lock();
	g_ucFlashFlag = ucRet;
	return ucRet;
}

// 从指定地址开始写入一个32位浮点的数组
u8 SaveParaFP32(u32 uiLength, u32 uiFlashAddr, fp32 *p)
{
	u8 ucRet = 0;
	u32 i = 0;

	/* Unlock the Flash to enable the flash control register access *************/
	FLASH_Unlock();

	/* Clear pending flags (if any) */
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
					FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

	/* Device voltage range supposed to be [2.7V to 3.6V], the operation will
	   be done by word */
	if (FLASH_EraseSector(FLASH_Sector_11, VoltageRange_3) != FLASH_COMPLETE)
	{
		/* Error occurred while sector erase.
		   User can add here some code to deal with this error  */
		return 0;
	}

	for (i = 0; i < uiLength; i++)
	{
		FlashSave.g_fpFlashValue[uiFlashAddr + i] = p[i];
	}

	for (i = 0; i < MAX_FLASH_LEN * 4; i++)
	{
		if (FLASH_ProgramByte(ADDR_FLASH_SECTOR_11 + i, FlashSave.g_ucFlashValue[i]) == FLASH_COMPLETE)
		{

			ucRet = 0x01;
		}
		else
		{
			ucRet = 0x00;
			g_ucFlashWrongNum++;
		}
	}

	/* Lock the Flash to disable the flash control register access (recommended
	  to protect the FLASH memory against possible unwanted operation) *********/
	FLASH_Lock();
	g_ucFlashFlag = ucRet;
	return ucRet;
}
