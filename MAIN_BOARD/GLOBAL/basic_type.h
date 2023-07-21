#ifndef __BASIC_TYPE__
#define __BASIC_TYPE__

#include <stdbool.h>

#define NULL 0 

#define FALSE false
#define TRUE true
	
typedef unsigned char  		UCHAR8;                  /* defined for unsigned 8-bits integer variable 	    �޷���8λ���ͱ���  */
typedef signed   char  		SCHAR8;                  /* defined for signed 8-bits integer variable		    �з���8λ���ͱ���  */
typedef unsigned short 		USHORT16;                /* defined for unsigned 16-bits integer variable 	  �޷���16λ���ͱ��� */
typedef signed   short 		SSHORT16;                /* defined for signed 16-bits integer variable 		  �з���16λ���ͱ��� */
typedef unsigned int   		UINT32;                  /* defined for unsigned 32-bits integer variable 	  �޷���32λ���ͱ��� */
typedef int   				SINT32;                      /* defined for signed 32-bits integer variable 		  �з���32λ���ͱ��� */
typedef float          		FP32;                    /* single precision floating point variable (32bits) �����ȸ�������32λ���ȣ� */
typedef double         		DB64;                    /* double precision floating point variable (64bits) ˫���ȸ�������64λ���ȣ� */

typedef UCHAR8            u8;                      /* defined for unsigned 8-bits integer variable 	    �޷���8λ���ͱ���  */
typedef USHORT16          u16;                     /* defined for unsigned 16-bits integer variable 	  �޷���16λ���ͱ��� */
typedef UINT32            u32;                     /* defined for unsigned 32-bits integer variable 	  �޷���32λ���ͱ��� */

typedef float fp32;
typedef double fp64;

#endif
