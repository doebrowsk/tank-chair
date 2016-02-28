#include "yaw.h"
#include "odometry.h"
#include "harliealloc.h"
#include "harlielog.h"
#include "iniparser.h"
#include "sysutils.h"
#include "MainCRIO.h"
#include <math.h>

/* Make these next 2 numbers configurable */
float Yaw_cYawNoiseStdDegree = .025;
float Yaw_cYawConversionStdinVolts = .0000878;

float Yaw_rateConstant;
float Yaw_offsetGain;

int32_t Yaw_previousYawRateSum;

float Yaw_PI;
float Yaw_poleDt;
float Yaw_PIover180;
float Yaw_ConstantVar;
float Yaw_dVar;

/**
 * Initializes the yaw functionality
 */
Sensor * Yaw_Init()
{
	Sensor* sensor = hMalloc(sizeof(Sensor));
	sensor->boot = &Yaw_Boot;
	sensor->update = &Yaw_Update;

	/*
	Setup the system constants
	*/
	Yaw_previousYawRateSum = 0;
	Yaw_cYawConversionStdinVolts = .0000878;
	Yaw_cYawNoiseStdDegree = 0.25;
	Yaw_PI = acos(-1);
	Yaw_PIover180 = ((float)acos(-1))/((float)(180.0));
	Yaw_poleDt = 1;
	Yaw_ConstantVar = 1;
	Yaw_dVar = 1;
	Sensor_Init(sensor);
	return sensor;
}

/**
 * Creates and ini file for the yaw and prepares the yaw sensor
 */
void Yaw_Boot(Sensor * sensor,SensorVariable * start)
{
	/*
		Polldt should be one millisecond as per spec of Chad
	*/

	Yaw_rateConstant = 0;
	int32_t slope = 47057;
	int32_t offset = 2347992;
	if(fileExists("config/yaw.ini"))
	{
		dictionary* config = iniparser_load("config/yaw.ini");
		Yaw_rateConstant = (float)iniparser_getdouble(config,"Constant:VoltsPerDegreePerSecond",0);
		Yaw_poleDt = (float) iniparser_getdouble(config,"Constant:PoleDtInMicroSeconds",0);
		Yaw_ConstantVar =(float) iniparser_getdouble(config,"Variance:Constant",1);
		Yaw_dVar = (float) iniparser_getdouble(config,"Variance:dTheta",1);
		slope = (int32_t) iniparser_getint(config,"Constant:Slope",47057);
		offset = (int32_t) iniparser_getint(config,"Constant:Offset",2347992);
		iniparser_freedict(config);
		LOG.YAW("Yaw Ini file loaded");
	}
	else
	{
		LOG.ERR("Yaw.ini was not found");
	}

	/*To restart the yaw rate sensor at 0*/
	Microprocessor->getYawRate();

	sensor->H = Matrix_Create(1,6);
	sensor->H = Matrix_SetValue(sensor->H,1,5,1);
        sensor->H = Matrix_SetValue(sensor->H,1,6,1);

	sensor->state->State= Matrix_Create(1,1);


	sensor->state->Variance = Matrix_Create(1,1);
	sensor->state->State = Matrix_Create(1,1);

	Yaw_previousYawRateSum = Microprocessor->getYawRate();
	LOG.YAW("Yaw Rate Sensor has been started.");
}

/**
 * Updates the yaw value
 */
void Yaw_Update(Sensor * sensor, const SensorVariable state)
{
	LOG.YAW("Starting the yaw sensor update");
	sensor->hasBeenUpdated = 1;

	int32_t yawRateSample = Microprocessor->getYawRate();
	int32_t sampleVoltageDt = yawRateSample;
        LOG.YAW("Previous yaw sample = %d ", Yaw_previousYawRateSum);

	Yaw_previousYawRateSum = yawRateSample;
	LOG.YAW("Yaw sample dt = %d ",sampleVoltageDt);
	/*
	deltaInAngles = sampleVoltageDt / 12.5mV/degree (whatever that value is) * yawlooptimeinus/10^6us
	*/
	/* TODO FIX THIS SO ITS A PRE DETERMINED CONSTANT */
	/* Chad Comment:: I think this is fine as is.  I might take out the pollDt value eventually, but we'll see.*/
	float dDegree = ((sampleVoltageDt / 1000.0) - Matrix_GetValue(state.State, 6,1)) /Yaw_rateConstant;


	/*OMG WE SHOULD GET A REALL NUMBER FOR UPDATE RATE*/
	LOG.YAW("Ddegree = %f",dDegree);
	float dTheta = dDegree * Yaw_PIover180;

	float Omega = dTheta;

	float yaw_var = (Yaw_ConstantVar + Yaw_dVar * Omega * Omega);
	LOG.YAW("yaw_var = %0.10f",yaw_var); 

	sensor->state->Variance = Matrix_SetValue(sensor->state->Variance,1,1,yaw_var);
	/*Must subtract from state to find the resdiual*/
	sensor->state->State = Matrix_SetValue(sensor->state->State,1,1,Omega-Matrix_GetOmega(state.State));

}

