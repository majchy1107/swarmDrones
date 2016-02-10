/*
	Copyright (C) 2009-2011. LIS Laboratory, EPFL, Lausanne
	Licensed to senseFly LLC in 2011
*/
/*
	Copyright (c) 2012 senseFly
	Author: Antoine Beyeler

	All rights reserved. Do not distribute.
*/


#ifndef __SENSOR_HUB_FWNG_H__
#define __SENSOR_HUB_FWNG_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stm32.h>
#include "geo_coordinate.h"

#ifndef SH_INITIAL_BATTERY_VOLTAGE
	/*! Control the initialisation value of the battery voltage low-pass filther, to avoid
		inelegent behaviour when initialized at 0.
	*/
	#define SH_INITIAL_BATTERY_VOLTAGE	12.6
#endif


//! The status of state variable
enum StateStatus
{
	// The order is taken into account, 0 for valid, then the greater the worse.
	VALID = 0, 	//!< the measure is valid
	DEGRADED, 	//!< the measure is degraded, maybe another sensor is used to approximate
	UNAVAILABLE //!< the measure is unavailable
};

//! The structure for status estimation
typedef struct
{
	geo_LonLat pos;

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
	float accZ;	 //!< the acceleration on the Z axis

	float airAcceleration; //!< the acceleration in the air

	float airSpeed; //!< the airspeed in m/s
	float groundSpeed; //!< the ground speed in m/s
	float speed3D; //!< the 3D speed (norm) of the plane in m/s
	float verticalSpeed; //!< the vertical speed in m/s (> 0 means climbing)
	float verticalSpeedAgl; //!< the vertical speed in m/s with respect to the ground (> 0 means climbing)

	// CAUTION: succession and order is assumed for vector Ishtar variable!
	float magX; //!< magnetic field measure on the x axis
	float magY; //!< magnetic field measure on the y axis
	float magZ;	 //!< magnetic field measure on the z axis

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

	float headingFromWind; //! the heading resulting from the ground speed and the new wind estimation

	float angleOfAttack; //! The angle of attack of the drone.
	uint32_t gpsTime;


	union
	{
		struct
		{
			unsigned longitude : 2; 		//!< status of the longitude (offset: 0)
			unsigned latitude : 2; 			//!< status of the latitude (offset: 2)

			unsigned xPosition : 2; 		//!< status of the x position relative to starting position (offset: 4)
			unsigned yPosition : 2;			//!< status of the y position relative to starting position (offset: 6)

			unsigned pitchRate : 2;			//!< status of the pitch rate (offset: 8)
			unsigned yawRate : 2;			//!< status of the yaw rate (offset: 10)
			unsigned rollRate : 2;			//!< status of the roll rate (offset: 12)

			unsigned accX : 2;				//!< status of the x acceleration (offset: 14)
			unsigned accY : 2;				//!< status of the y acceleration (offset: 16)
			unsigned accZ : 2;				//!< status of the z acceleration (offset: 18)
			unsigned airAcceleration : 2;	//!< status of the air acceleration (offset: 20)

			unsigned airSpeed : 2; 			//!< status of the air speed (offset: 22)
			unsigned groundSpeed : 2;		//!< status of the ground speed (offset: 24)
			unsigned speed3D : 2;			//!< status of the 3D speed (offset: 26)
			unsigned verticalSpeed : 2;		//!< status of the vertical speed (offset: 28)

			unsigned magX : 2;				//!< status of the x magnetic field (offset: 30)
			unsigned magY : 2;				//!< status of the y magnetic field (offset: 0)
			unsigned magZ : 2;				//!< status of the z magnetic field (offset: 2)

			unsigned heading : 2;			//!< status of the heading (offset: 4)
			unsigned course : 2;			//!< status of the course (offset: 6)

			unsigned altitude : 2;			//!< status of the altitude (offset: 8)
			unsigned height : 2;			//!< status of the height (offset: 10)

			unsigned pitchAngle : 2;		//!< status of the pitch angle (offset: 12)
			unsigned yawAngle : 2;			//!< status of the yaw angle (offset: 14)
			unsigned rollAngle : 2;			//!< status of the roll angle (offset: 16)

			unsigned batteryVoltage : 2;				//!< status of the battery voltage (offset: 18)
			unsigned batteryCurrent : 2;				//!< status of the battery current (offset: 20)
			unsigned batteryOCVoltage : 2;				//!< status of the battery open circuit voltage (offset: 22)
			unsigned batteryIntegratedCharge : 2; 		//!< status of the battery integrated charge (offset: 24)
			unsigned batteryRemainingChargePercent : 2; //!< status of the battery remaining charge (offset: 26)

			unsigned heightAGL : 2;			//!< status of the height from the optic flow (offset: 28)

			unsigned temperature : 2;		//!< status of the temperature (offset 30)
		} __attribute((packed));

		uint32_t status[2];			//!< raw status data, used for ishtar variables
	} __attribute((packed)) status;
} __attribute((packed)) sh_State;


#if SF_LOG_HIGH_RATE
	//! The structure for raw data.
	typedef struct
	{
		// IMU
		struct
		{
			float rollRate; //!< the roll rate in deg/s
			float pitchRate; //!< the pitch rate in deg/s
			float yawRate; //!< the yaw rate in deg/s
			float accX; //!< the acceleration on the X axis
			float accY; //!< the acceleration on the Y axis
			float accZ; //!< the acceleration on the Z axis
			int32_t accTemp; //!< the raw temperature of the accelero
			int32_t gyroTemp; //!< the raw temperature of the gyro
			//float magnX; //!< the magnetometer on the X axis
			//float magnY; //!< the magnetometer on the Y axis
			//float magnZ; //!< the magnetometer on the Z axis
			int32_t rollRateRaw; //!< the roll rate in deg/s      RAW
			int32_t pitchRateRaw; //!< the pitch rate in deg/s    RAW
			int32_t yawRateRaw; //!< the yaw rate in deg/s        RAW
			int32_t accXRaw; //!< the acceleration on the X axis  RAW
			int32_t accYRaw; //!< the acceleration on the Y axis  RAW
			int32_t accZRaw; //!< the acceleration on the Z axis  RAW
		}
		imu;
		float airSpeed;
		float altHeight;
		//float dt; //!< the time delta since previous measurements
		//float dtReciprocal; //!< the reciprocal of the time delta
		uint16_t dynamicPressure;
		uint16_t dynamicPressureTemp;
		float time; //!< elapsed time in second

	} __attribute((packed)) sh_RawState;
#endif

void sh_Init();

//! This function gathers the data from all sensors except gyros, which are given in parameters
void sh_Step();

//! This function returns a pointer to the state
sh_State* sh_GetState();

//! Return true if all sensors are valid
bool sh_SensorsValid();

// Prints the current state of the imu on the console.
void sh_PrintCurrentState(uint8_t* args, uint8_t len);

#ifdef __cplusplus
}
#endif

#endif
