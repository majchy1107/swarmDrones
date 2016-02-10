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
*	\file nav_def.h
*	\brief Definition file of the waypoint navigation module.
*
*	This file contains the defintions and enumerations used by the navigation module.
*
*/

#ifndef __NAV_DEF_H__
#define __NAV_DEF_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stm32.h>
#include "geo_coordinate.h"


//--------------------
// Public definitions
//--------------------

//! Enumeration for waypoint types.
enum nav_wpt_WaypointType
{
	CIRCLE = 0, 	//!< makes a circle around the waypoint
	LINE = 2		//!< follow a line to the waypoint
};

//! Enumeration for turn directions.
enum nav_wpt_TurnDirection
{
	COUNTER_CLOCKWISE = 0, 					//! counter-clockwise rotation
	CLOCKWISE 								//! clockwise rotation
};


//! The possible reference point for the altitude.
enum nav_wpt_AltitudeReference
{
	ALTITUDE_ABOVE_TAKE_OFF = 0, //!< Altitude above take-off point.
	ALTITUDE_WGS84, //!< Altitude above WGS-84 elipsoid.
	ALTITUDE_AMSL, //!< Altitude above mean sea level.
	ALTITUDE_AGL //!< Altitude above ground level.
};

//! Enumeration for the altitude transition.
enum nav_wpt_AltitudeTransitionType
{
	ALTITUDE_TRANSITION_MAX_VSPEED = 0,
	ALTITUDE_TRANSITION_LINEAR,
	ALTITUDE_TRANSITION_BEFORE_DEPARTURE,
	ALTITUDE_TRANSITION_AFTER_ARRIVAL
};


//-------------------------
// Public type definitions
//-------------------------

/*!
 * The type definition for a waypoint ID.
 * Positive value indicates that the waypoint is part of the route, while negative
 * value indicates that this is a special purpose waypoint.
 */
typedef int8_t nav_WaypointId;

//! The id for invalid waypoint.
#define NAV_INVALID_WAYPOINT_ID -1

//! Undefined heading value.
#define NAV_UNDEF_HEADING 1234

//! Structure for the waypoints (15 bytes long)
typedef struct
{
	geo_LonLat pos;					//!< The 2D position.
	int16_t height;					//!< The height, in [m].
	union
	{
		struct
		{
			uint16_t course;			//!< For line waypoints, the course to have when passing over the waypoint, in [°] * 100.
		} __attribute((packed)) line;	//!< The parameters of the line waypoint.

		struct
		{
			uint16_t minAngularSweep;	//!< For circular waypoints, the minimum angular sweep before transitions are allowed, in [°].
			uint8_t radius;				//!< Circle radius around waypoint, in [m] (0-255 m).
		} __attribute((packed)) circle;	//!< The parameters of the circle waypoint.
	} __attribute((packed));
	union
	{
		struct
		{
			unsigned type					: 2; //!< The waypoint type.
			unsigned isActive				: 1; //!< If the waypoint is active.
			unsigned autoNext				: 1; //!< If the plane is going automatically to the next waypoint once the current one is achieved.
			unsigned action					: 4; //!< The action to perform at the waypoint, if defined.

			unsigned turnDirection			: 1; //!< The direction to turn over the waypoint if in circle (CLOCKWISE or COUNTER_CLOCKWISE).
			unsigned holdHeight				: 1; //!< If the altitude must be held or not to achieve the waypoint.
			unsigned altitudeReference		: 2; //!< The reference for the altitude.
			unsigned altitudeTransitionFromAbove : 2; //!< The type of altitude transition when coming from above.
			unsigned altitudeTransitionFromBelow : 2; //!< The type of altitude transition when coming from below.
		} __attribute((packed));
		uint16_t flags;						//!< The flags.
	} __attribute((packed)) params;			//!< The parameters of the waypoint.
} __attribute((packed)) nav_Waypoint;


//! Structure for the status of the current waypoint
typedef struct
{
	float angularSweep;				//!< Number of "°"s turned around (circle) waypoint.
	float distanceToWpt;			//!< Distance to waypoint in meters.
	float currentHeadingOnCircle;	//!< The current heading of the aircraft with respect to the center of the circle waypoint.
	float lastHeadingOnCircle;		//!< The previous heading of the aircraft with respect to the center of the circle waypoint.
	union
	{
		struct
		{
			unsigned hasBeenReached	: 1;	//!< If the waypoint has already been reached at least once.
			unsigned isOver			: 1;	//!< If the plane is currently over the waypoint (e.g. on circle, past waypoint on line etc.).
			unsigned onLine			: 1;	//!< If the plane is on the line.
			unsigned heightOk		: 1;	//!< If the altitude at the waypoint is correct.
			unsigned isAchieved		: 1;	//!< The waypoint goals (position, altitude, speed, course) are achieved.
		};
		uint8_t flags;		//! the flags
	};
}
nav_Status;


//! This call-back function is called when a waypoint is achieved
typedef void (*nav_WaypointAchievedFunc)(nav_Waypoint* waypoint);

//! This call-back function is called when the current waypoint is changed, the status is the status of the previous waypoint.
typedef void (*nav_WaypointActivatedFunc)(nav_WaypointId prevWptId, nav_WaypointId nextWptId, nav_Status* status);

//! This call-back function is called when the flight plan achieves the last waypoint.
typedef void (*nav_FlightPlanFinishedFunc)();

//! This call-back function is called when a waypoint has been modified since the last check.
typedef void (*nav_WaypointModifiedFunc)(nav_WaypointId wptId);

//! Structure for a flight plan.
typedef struct
{
	//! A pointer to the waypoints.
	nav_Waypoint* waypoints;
	//! A pointer to the checksums.
	uint16_t* waypointChecksums;
	//! The number of waypoints in the plan.
	nav_WaypointId numberOfWaypoints;

	//! The current waypoint id.
	nav_WaypointId currentWaypointId;
	//! The status of the current waypoint.
	nav_Status status;

	// Call-back
	//! The function to call when a waypoint is achieved.
	nav_WaypointAchievedFunc waypointAchievedFunc;
	//! The function to call when a waypoint is activated.
	nav_WaypointActivatedFunc waypointActivatedFunc;
	//! The function to call when the flight plan is finished.
	nav_FlightPlanFinishedFunc flightPlanFinishedFunc;
	//! The function to call when a waypoint is modified.
	nav_WaypointModifiedFunc waypointModifiedFunc;
}
nav_FlightPlan;

//! Structure for the navigation commands
typedef struct
{
	// navigation commands
	float cmdAirSpeed;	//!< the command for the airspeed
	float cmdAltitude;	//!< the command for the altitude
	float cmdTurnRate;	//!< the command for the turn rate accounting both waypoint turn rate and course error
	float cmdCourse;	//!< the command for the course
}
nav_Commands;

//! Cache data for the circle navigation.
typedef struct
{
	nav_Waypoint lineWaypoint; //!< The line transit waypoint to the target position.
	nav_Waypoint circleWaypoint; //!< The circle waypoint to adjust the altitude, before or after.
	nav_Status status; //!< The status of the waypoint of the path.
	float slopeTangent; //! The slope to use to adjust the altitude.
	nav_Waypoint* currentWaypoint; //!< The current waypoint known.
	nav_Waypoint currentWaypointData; //!< The data of the current waypoint.
}
nav_Path;

#ifdef __cplusplus
}
#endif

#endif


