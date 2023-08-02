#ifndef __E103_W06_H__
#define __E103_W06_H__

#include "stm32f4xx.h"
#include "basic_type.h"
#include "string.h"
#include "stdarg.h"
#include "stdio.h"
#include "basic_function.h"

extern union float2char uart6_tx_buf[40];
extern union float2char uart3_tx_buf[40];

void wifi_printf(char *fmt, ...);
void wifi_init(void);

#endif
