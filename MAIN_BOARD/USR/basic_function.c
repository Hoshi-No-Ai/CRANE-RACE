#include "basic_function.h"

void delay_ms(int t)
{
    uint32_t i;
    for (; t > 0; t--)
        for (i = 0; i < 42000; i++)
            ;
}

typedef uint32_t u32;

void delay_us(u32 t)
{
    u32 i = 0;
    for (i = 0; i < t; i++)
    {
        u32 a = 40;
        while (a--)
            ;
    }
}
