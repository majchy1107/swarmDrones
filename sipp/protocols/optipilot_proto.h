/*
	Copyright © 2010 senseFly
	Author: Yannick Gasser
	
	All rights reserved. Do not distribute.
*/

#ifndef __SF_OPTIPILOT_H__
#define __SF_OPTIPILOT_H__

#ifdef __cplusplus
extern "C" {
#endif


/* Optipilot protocol:
* 
* Protocol for the communication of the the gyroscopes integration values to the BlackFin processor
* and the reception of the navigation control from the BlackFin processor to the Aeropic.
*/


enum
{
	OPTIPILOT_ID = 0x02
};

typedef struct
{
	signed short ofDiffHor;
	signed short ofDiffVert;
	signed short ofAdns1X;
	signed short ofAdns1Y;
	unsigned char adns1Squal;
	signed short ofAdns2X;
	signed short ofAdns2Y;
	unsigned char adns2Squal;
	unsigned long itow;
} __attribute((packed)) OptipilotProto;




#ifdef __cplusplus
}
#endif


#endif
