/*
	Copyright (c) 2013 senseFly
	Author: Alexandre Habersaat

	All rights reserved. Do not distribute.
*/

#ifndef __GEO_COORDINATE_H__
#define __GEO_COORDINATE_H__

#ifdef __cplusplus
extern "C" {
#endif

//----------
// Includes
//----------

#include <math.h>
#include <stdint.h>
#include <inttypes.h>
#include <stm32.h>

//--------------------
// Public definitions
//--------------------

//! The radius of the earth in meters
#define PHYS_EARTH_RADIUS 6367449

//! Conversion of GPS longitude and latitude degrees to radians
#define GPS_LONLAT_TO_RAD	(M_PI /180.0 / 10000000.0)
//! Conversion of radian to GPS longitude and latitude degrees
#define GPS_RAD_TO_LONLAT	(10000000.0 * 180.0 / M_PI)


//--------------------------
// Public types definitions
//--------------------------

//! The definition for the geographic coordinate.
typedef struct
{
	int32_t lon; //!< The longitude, in [°]*1e7.
	int32_t lat; //!< The latitude, in [°]*1e7.
}
geo_LonLat;

#ifdef __cplusplus
}
#endif

#endif /* __GEO_COORDINATE_H__ */
