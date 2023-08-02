#ifndef __BASIC_TYPE__
#define __BASIC_TYPE__

#include <stdbool.h>

#define NULL 0

#define FALSE false
#define TRUE true

typedef unsigned char UCHAR8;    /* defined for unsigned 8-bits integer variable 	    无符号8位整型变量  */
typedef signed char SCHAR8;      /* defined for signed 8-bits integer variable		    有符号8位整型变量  */
typedef unsigned short USHORT16; /* defined for unsigned 16-bits integer variable 	  无符号16位整型变量 */
typedef signed short SSHORT16;   /* defined for signed 16-bits integer variable 		  有符号16位整型变量 */
typedef unsigned int UINT32;     /* defined for unsigned 32-bits integer variable 	  无符号32位整型变量 */
typedef int SINT32;              /* defined for signed 32-bits integer variable 		  有符号32位整型变量 */
typedef float FP32;              /* single precision floating point variable (32bits) 单精度浮点数（32位长度） */
typedef double DB64;             /* double precision floating point variable (64bits) 双精度浮点数（64位长度） */

typedef UCHAR8 u8;    /* defined for unsigned 8-bits integer variable 	    无符号8位整型变量  */
typedef USHORT16 u16; /* defined for unsigned 16-bits integer variable 	  无符号16位整型变量 */
typedef UINT32 u32;   /* defined for unsigned 32-bits integer variable 	  无符号32位整型变量 */

typedef float fp32;
typedef double fp64;

#endif
