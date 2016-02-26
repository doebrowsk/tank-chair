#include "harliealloc.h"
#include "sensor.h"
#include "harlielog.h"
#include "MainCRIO.h"
#include <math.h>
#include <string.h>
#include "differentialSteering.h"
#include "compass.h"

#define PIOVER180 0.01745329251994329577
#define TWOPI 6.28318530717958647693
#define PI 3.14159265358979323846

float COMPASS_Variance;
float COMPASS_Declination;

/**
 * Converts the heading to radians
 */
float _ConvertToRadians(float heading)
{
	return heading*(PIOVER180);
}

/**
 * Initializes the Compass with magic numbers and sets the boot and update function pointers
 */
Sensor * Compass_Init(void)
{
    // TODO: change the magic numbers
	Sensor* sensor = hMalloc(sizeof(Sensor));
	sensor->boot = &Compass_Boot;
	sensor->update = &Compass_Update;
	Sensor_Init(sensor);
	COMPASS_Declination = -7;
	COMPASS_Variance = 0.523598776; /*30 Degrees*/
	return sensor;
}

/**
 * Updates the compass values 
 */
void Compass_Update(Sensor * sensor, const SensorVariable state)
{
	LOG.COMPASS("Starting Compass Sensor Update");
	if(Microprocessor->isCompassNew()==0)
	{
		LOG.COMPASS("No new compass data");
		return;
	}
	float heading = Microprocessor->getCompassHeading()+COMPASS_Declination;
	LOG.COMPASS("Heading in degrees = %f",heading);
	float rads = _ConvertToRadians(heading);
	float  currentHeading = Matrix_GetTheta(state.State);
	float residual = rads - currentHeading;

	if(residual < -PI)
	{
		residual = residual + TWOPI;
	}
	else if	(residual > PI)
	{
		residual = residual - TWOPI;
	}

	/*
	LOG.COMPASS("Heading in rads found was %f",rads);
	float  currentHeading = Matrix_GetTheta(state.State);
	LOG.COMPASS("Current heading = %f",currentHeading);
	float rotations = currentHeading/TWOPI;
	LOG.COMPASS("Rotations = %f",rotations);
	float realitive = currentHeading - TWOPI*((int)rotations);
	LOG.COMPASS("Realitive = %f",realitive);
	float minAngle = calculateMinimalSteeringAngle(realitive,TWOPI*((int)rotations)+rads);
	LOG.COMPASS("Calculate Minimal SteeringAngle = %f",minAngle);
	float correctDiffHeading = currentHeading-minAngle;
	LOG.COMPASS("correctDiffHeading was %f",correctDiffHeading);
	*/
	sensor->state->State = Matrix_SetValue(sensor->state->State,1,1,residual);
	sensor->state->Variance = Matrix_SetValue(sensor->state->Variance,1,1,COMPASS_Variance);
	sensor->hasBeenUpdated=1;
}

/**
 * Initliazes the Compass with the .ini file 
 */
void Compass_Boot(Sensor * sensor,SensorVariable * start)
{
	LOG.COMPASS("Compassing Booting");
	if(fileExists("config/compass.ini"))
	{
		dictionary* config = iniparser_load("config/compass.ini");
		COMPASS_Variance = iniparser_getdouble(config,"Variance:Variance",0.523598776);
		COMPASS_Declination= iniparser_getdouble(config,"Declination:Declination",-7.5);
		iniparser_freedict(config);
		LOG.COMPASS("compass.ini file loaded");
	}
	else
	{
		LOG.COMPASS("compass.ini was not found");
	}
	sensor->H = Matrix_Create(1,5);
	sensor->H = Matrix_SetValue(sensor->H,1,3,1);
	sensor->state->State= Matrix_Create(1,1);
	sensor->state->Variance = Matrix_Create(1,1);
	LOG.COMPASS("Compassing Booted");
}

