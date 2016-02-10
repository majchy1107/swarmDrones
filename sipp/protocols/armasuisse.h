/*
	Copyright © 2010 senseFly
	Author: Antoine Beyeler
	
	All rights reserved. Do not distribute.
*/

#ifndef __SF_ARMASUISSE_H__
#define __SF_ARMASUISSE_H__

#ifdef __cplusplus
extern "C" {
#endif


/* Armasuisse protocol:
* 
* Protocol for the communication of the target position and camera control from
* the Gumstix computer to the Aeropic.
*/


enum
{
	ARMASUISSE_ID = 0x01
};


enum CameraFlags
{
	trigShot = 1,
};

typedef struct
{
	signed long longitude;			//!< 1e-7 degrees
	signed long latitude; 			//!< 1e-7 degrees
	unsigned short cameraFlags;		//!< camera flags (see enum CameraFlags)

} __attribute((packed)) ArmasuisseProto;




#ifdef __cplusplus
}
#endif


#endif
