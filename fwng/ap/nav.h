/*
	Copyright (C) 2008-2011. LIS Laboratory, EPFL, Lausanne
	Licensed to senseFly LLC in 2011
*/
/*
	Copyright (c) 2012 senseFly
	Author: Antoine Beyeler

	All rights reserved. Do not distribute.
*/
/*!
*	\file nav.h
*	\brief Header file of the waypoint navigation module
*/

#ifndef __NAV_H__
#define __NAV_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stm32.h>
#include "nav_waypoint.h"
#include "nav_circle.h"
#include "nav_flightplan.h"


//--------------------
// Public definitions
//--------------------

#ifndef NAV_DEFAULT_K_COURSE
	//! Proportional factor linking course error and turn rate
	#define NAV_DEFAULT_K_COURSE		0.5
#endif

#ifndef NAV_DEFAULT_MAX_TURNRATE
	//! Maximum turnrate allowed to be used by navigation controller
	#define NAV_DEFAULT_MAX_TURNRATE	35.0
#endif

#ifndef NAV_DEFAULT_AIRSPEED
	//! Default airspeed for normal waypoints.
	#define NAV_DEFAULT_AIRSPEED		12.0
#endif


#ifndef NAV_DEFAULT_ALTITUDE
	//! Default height above ground for normal waypoints.
	#define NAV_DEFAULT_ALTITUDE		50.0
#endif


#ifndef NAV_DEFAULT_RADIUS
	//! Default radius for normal waypoints.
	#define NAV_DEFAULT_RADIUS			30.0
#endif


//-------------------------
// Public type definitions
//-------------------------

//! Structure for the navigation parameters
typedef struct
{
	float kCourse;			//!< the translation factor between course error and commanded turnrate
	float maxTurnRate;		//!< the maximum turnrate to be commanded by the navigation module
}
nav_Params;



//----------------------------
// Public function prototypes
//----------------------------

//! This function sets up the navigation
void nav_Init();

//! This function reset the nav module
void nav_Reset();

//! Global function for nav step. This function is usefull to detect if nav_Step was or not executed during last loop.
void nav_StepGlobal();
//! Run the navigation controller for the given flight plan. Return false if the flight plan is invalid.
bool nav_Step(nav_FlightPlan* fp);

//! Calculate the commands needed to go to waypoint "wpt".
void nav_CalculateCommands(float* desiredCourse, float* desiredTurnRate, nav_Waypoint* wpt, nav_CircleCache* cache, uint8_t flag);
//! This function calculates a turn rate based on desired course and turn rate.
float nav_CalculateTurnRate(float desiredCourse, float desiredTurnRate, nav_Waypoint* wpt);

//! This function returns the navigation commands
nav_Commands* nav_GetCommands();
//! This function returns the navigation parameters
nav_Params* nav_GetParams();
//! Return the current flight plan
nav_FlightPlan* nav_GetCurrentFlightPlan();

//! This function registers the variables to Ishtar
void nav_RegisterToIshtar();

#ifdef __cplusplus
}
#endif

#endif
