/*
	Copyright © 2010 senseFly
	Author: Antoine Beyeler
	
	All rights reserved. Do not distribute.
*/

#ifndef __SF_DEMO_H__
#define __SF_DEMO_H__

#ifdef __cplusplus
extern "C" {
#endif


enum
{
	OUTPUT_ID = 0x01,
	INPUT_ID = 0x02
};



typedef struct
{
	unsigned long counter;
} __attribute((packed)) OutputData;


typedef struct
{
	unsigned char activateLed;
} __attribute((packed)) InputData;




#ifdef __cplusplus
}
#endif


#endif
