/*
	Copyright © 2010 senseFly
	Author: Antoine Beyeler
	
	All rights reserved. Do not distribute.
*/

#ifndef __SF_METEO_H__
#define __SF_METEO_H__

#ifdef __cplusplus
extern "C" {
#endif


enum
{
	OUTPUT_ID = 0x01,
//	INPUT_ID = 0x02
};



typedef struct
{
	unsigned long counter;
	float tempSRS34;
	float tempHC2;
	float humidityHC2;
} __attribute((packed)) OutputData;


/*typedef struct
{
	unsigned char activateLed;
	float windSpeed;
	float windDir;
} __attribute((packed)) InputData;
*/



#ifdef __cplusplus
}
#endif


#endif
