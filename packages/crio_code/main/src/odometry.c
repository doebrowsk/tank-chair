#include "odometry.h"
#include "harliealloc.h"
#include "harlielog.h"
#include "sysutils.h"
#include "iniparser.h"
#include <math.h>

/*From the INI file*/
float    Odometry_leftWheelMeterPerTick;
float    Odometry_rightWheelMeterPerTick;
float    Odometry_leftMotorMeterPerTick;
float    Odometry_rightMotorMeterPerTick;
/* Chad Comment: Eric and I measured the track as .561975 meters. */
float    Odometry_trackInMeters; /*0.5677*/

float Odometry_cvarL;
float Odometry_cvarR;
/* Make track variance a configurable setting as well. */
float Odometry_ctrackVar;
const float Odometry_small_const_variance = 1e-8;


/*Looping Variables*/
int32_t Odometry_rightWheelTicks;
int32_t Odometry_leftWheelTicks;

/*Todo verfiy that this value is still required */
float Odometry_dTheta=0;

Sensor * Odometry_Init(void)
{
	Sensor* sensor = hMalloc(sizeof(Sensor));
	sensor->boot = &Odometry_Boot;
	sensor->update = &Odometry_Update;
	Sensor_Init(sensor);
	Odometry_leftMotorMeterPerTick = 0;
	Odometry_rightMotorMeterPerTick = 0;
	Odometry_leftWheelMeterPerTick = 0;
	Odometry_rightWheelMeterPerTick = 0;
	Odometry_rightWheelTicks = 0;
	Odometry_leftWheelTicks = 0;
	Odometry_cvarL = .000049871;
	Odometry_cvarR = .000073805;
	Odometry_ctrackVar = .00016129;
	Odometry_trackInMeters = 1;
	return sensor;
}


/**
 * Updates the odometry values
 */
void Odometry_Update(Sensor * sensor,const SensorVariable state)
{
	LOG.ODOMETRY("Starting Odomentry Sensor Update");
	/*Set to true*/
	sensor->hasBeenUpdated = 1;

	/*Get New data and figure out the Dticks in each wheel*/
	int32_t newLeftWheelTicks = Microprocessor->getLeftWheelTicks();
	int32_t newRightWheelTicks = Microprocessor->getRightWheelTicks();

	LOG.ODOMETRY("LEFT WHEEL TICKS = %d",newLeftWheelTicks);
	LOG.ODOMETRY("RIGHT WHEEL TICKS = %d",newRightWheelTicks);

	int32_t dticksLeftWheel = newLeftWheelTicks -  Odometry_leftWheelTicks;
	int32_t dticksRightWheel = newRightWheelTicks -  Odometry_rightWheelTicks;

	Odometry_leftWheelTicks = newLeftWheelTicks;
	Odometry_rightWheelTicks = newRightWheelTicks;

	float dmLeftWheel = (float)dticksLeftWheel * Odometry_leftWheelMeterPerTick;
	float dmRightWheel = (float)dticksRightWheel * Odometry_rightWheelMeterPerTick;

	float YleftEnc = dmLeftWheel - 1.0/2.0 * PSOUpdateRateInS *(2.0 * Matrix_GetVel(state.State)+Odometry_trackInMeters*Matrix_GetOmega(state.State));
	float YrightEnc = dmRightWheel - 1.0/2.0 * PSOUpdateRateInS *(2.0 * Matrix_GetVel(state.State)-Odometry_trackInMeters*Matrix_GetOmega(state.State));

	float VarLeftEnc = dmLeftWheel*dmLeftWheel *  Odometry_cvarL + Odometry_small_const_variance;
	float VarRightEnc = dmRightWheel*dmRightWheel * Odometry_cvarR + Odometry_small_const_variance;

	/*Set the variances.
	These should be the addition of the previous variance and the variance created by this sensor*/
	/* If you want to make the variances larger, it should be done in the config file, not as this multiply */
	matrix Variance = Matrix_Create(2,2);
	Variance = Matrix_SetValue(Variance,1,1,VarLeftEnc);
	Variance = Matrix_SetValue(Variance,2,2,VarRightEnc);
	sensor->state->Variance = Variance;
	/* Set the new location based on the changes in the odometry sensor*/
	matrix Residual = Matrix_Create(2,1);
	Residual = Matrix_SetValue(Residual,1,1,YleftEnc);
	Residual = Matrix_SetValue(Residual,2,1,YrightEnc);
	sensor->state->State=Residual;
}

/**
 * Creates the ini file for the odom and initializes the odom values to initial values
 */
void Odometry_Boot(Sensor * sensor,SensorVariable * start)
{
	/* MAKE THESE GLOBAL*/

	LOG.ODOMETRY("BOOTING THE ODOMETERY SENSOR");
	if(fileExists("config/odometry.ini"))
	{
		dictionary* config = iniparser_load("config/odometry.ini");
		Odometry_leftWheelMeterPerTick = iniparser_getdouble(config,"WheelEncoderConstant:MeterPerTickLeft",0);
		Odometry_rightWheelMeterPerTick = iniparser_getdouble(config,"WheelEncoderConstant:MeterPerTickRight",0);
		Odometry_trackInMeters = iniparser_getdouble(config,"DistanceBetweenWheels:track",0);
		Odometry_cvarL = iniparser_getdouble(config,"Variance:LeftWheel",.000049871);
		Odometry_cvarR = iniparser_getdouble(config,"Variance:RightWheel",.000073805);
		iniparser_freedict(config);
		LOG.ODOMETRY("Odometry Ini file loaded");
	}
	else
	{
		LOG.ERR("odomentry.ini was not found");
	}

	matrix Variance = Matrix_Create(2,2);
	Variance = Matrix_SetValue(Variance,1,1,.01);
	Variance = Matrix_SetValue(Variance,2,2,.01);
	sensor->state->Variance=Variance;
	sensor->state->State= Matrix_Create(3,1);
	sensor->H = Matrix_Create(2,6);
	sensor->H = Matrix_SetValue(sensor->H,1,4,PSOUpdateRateInS);
	sensor->H = Matrix_SetValue(sensor->H,1,5,Odometry_trackInMeters*PSOUpdateRateInS/2.0);
	sensor->H = Matrix_SetValue(sensor->H,2,4,PSOUpdateRateInS);
	sensor->H = Matrix_SetValue(sensor->H,2,5,-Odometry_trackInMeters*PSOUpdateRateInS/2.0);

	Odometry_rightWheelTicks=Microprocessor->getRightWheelTicks();
	Odometry_leftWheelTicks=Microprocessor->getLeftWheelTicks();
}

/**
 * returns the theta
 */
float Odometry_GetdTheta()
{
	return Odometry_dTheta;
}
