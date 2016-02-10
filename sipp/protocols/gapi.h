/*
	Copyright (c) 2010-2011 senseFly
	Author: Antoine Beyeler
	
	All rights reserved. Do not distribute.
*/

#ifndef __SF_GAPI_H__
#define __SF_GAPI_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "fwng/ap/sensor_hub.h"

/* Gumstix API protocol */

enum
{
	GAPI_STATE_ID = 0x01,
	/* gapi_StatePacket structure, sent at 50Hz from the sfPilot to the API */
	
	GAPI_WAYPOINT_ID = 0x02,
	/* nav_wpt_Waypoint structure, sent from the API to the sfPilot upon updateGWpt() */
	
	GAPI_COMMANDS_ID = 0x03,
	/* gapi_CommandPacket structure, sent from the API to the sfPilot upon related API call */
	
	GAPI_CONTROL_ID = 0x04
	/* gapi_ControlPacket structure, sent from the API to the sfPilot upon updateControl() */
};



typedef struct
{
	 int32_t longitude; //!< the longitude in deg E+7
    int32_t latitude; //!< the latitude in deg E+7

    // CAUTION: succession and order is assumed for vector Ishtar variable!
    float xPosition; //!< x position relative to starting position (sh_SyncMeasures), in global frame (pointing east)
    float yPosition; //!< y position relative to starting position (sh_SyncMeasures), in global frame (pointing north)

    // CAUTION: succession and order is assumed for vector Ishtar variable!
    float rollRate; //!< the roll rate in deg/s
    float pitchRate; //!< the pitch rate in deg/s
    float yawRate; //!< the yaw rate in deg/s

    // CAUTION: succession and order is assumed for vector Ishtar variable!
    float accX; //!< the acceleration on the X axis
    float accY; //!< the acceleration on the Y axis
    float accZ;     //!< the acceleration on the Z axis

    float airAcceleration; //!< the acceleration in the air

    float airSpeed; //!< the airspeed in m/s
    float groundSpeed; //!< the ground speed in m/s
    float speed3D; //!< the 3D speed (norm) of the plane in m/s
    float verticalSpeed; //!< the vertical speed in m/s (> 0 means climbing)
    float verticalSpeedAgl; //!< the vertical speed in m/s with respect to the ground (> 0 means climbing)

    // CAUTION: succession and order is assumed for vector Ishtar variable!
    float magX; //!< magnetic field measure on the x axis
    float magY; //!< magnetic field measure on the y axis
    float magZ;     //!< magnetic field measure on the z axis

    float heading; //!< the heading where the nose of the aircraft is pointing, in deg
    float course; //!< the course or track, i.e. the direction where the aircraft is moving, in deg

    float altitude; //!< the altitude above the sea level
    float height; //!< the height above the starting point (depends on calibration)

    // CAUTION: succession and order is assumed for vector Ishtar variable!
    float rollAngle; //!< the roll angle
    float pitchAngle; //!< the pitch angle
    float yawAngle; //!< the yaw angle

    float batteryVoltage; //!< the voltage of the battery
    float batteryCurrent; //!< the current draw
    float batteryOCVoltage; //!< the open circuit voltage of the battery
    float batteryIntegratedCharge; //!< sum of current drawn
    float batteryRemainingCharge; //!< Remaining charge estimated
    uint8_t batteryRemainingChargePercent; //!< Remaining charge in percent

    float dt; //!< the time delta since previous measurements
    float dtReciprocal; //!< the reciprocal of the time delta
    float time; //!< elapsed time in second

    float magDeclinaison; //! the magnetique declinaison
    float magInclinaison; //! the magnetique declinaison
    float magIntensity; //! the magnetique declinaison
    float heightAgl; //! the height above ground from the optic flow sensor

    float temperature; //! the temperature on the autopilot

	unsigned long itow;
	unsigned char enableFlag;
} __attribute((packed)) gapi_StatePacket;



typedef struct
{
	unsigned goHomeFlag			: 1;
	unsigned goToStartFlag		: 1;
	unsigned goLandFlag			: 1;
	unsigned startMissionFlag	: 1;
	unsigned holdPositionFlag	: 1;
	unsigned resumeFlag			: 1;
	unsigned reserved			: 2;
} __attribute((packed)) gapi_CommandPacket;



enum gapi_ControlModes
{
	GAPI_NO_CONTROL = 0x0,
	
	GAPI_TURN_RATE_CONTROL = 0x1,
	GAPI_ALTITUDE_CONTROL = 0x1,
	
	GAPI_ANGLE_CONTROL = 0x2,
	GAPI_SPEED_CONTROL = 0x2,
	
	GAPI_ACTUATOR_CONTROL = 0x3,
	GAPI_THRUST_CONTROL = 0x3
};

typedef struct
{
	float aileronCommand;
	float elevatorCommand;
	float thrustCommand;
	
	struct
	{
		unsigned aileronControlMode		: 2;
		unsigned elevatorControlMode	: 2;
		unsigned thrustControlMode		: 2;
		unsigned reserved				: 2;
	} __attribute((packed));
} __attribute((packed)) gapi_ControlPacket;


#ifdef __cplusplus
}
#endif


#endif
