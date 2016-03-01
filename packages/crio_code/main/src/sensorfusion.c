#include "sensorfusion.h"
#include "sysutils.h"
#include "odometry.h"
#include "yaw.h"
#include "iniparser.h"
#include "harliealloc.h"
#include "harlielog.h"
#include "sensor.h"
#include "microprocessorinterface.h"
#include "statevariable.h"
#include "packets.h"
#include "gps.h"
#include "compass.h"

void _SensorFusion_AddSensor(SensorFusion * fusion, Sensor * sensor);

/**
 * Add a sensor to the sensor list.  Cannot add more than 4 sensors to the list
 */
void _SensorFusion_AddSensor(SensorFusion * fusion, Sensor * sensor)
{
	if(fusion->sensorCount>= NUMBER_OF_SENSORS)
	{
		LOG.ERR("Too many sensors have been declared. Allowed to declare %i",NUMBER_OF_SENSORS);
	}
	fusion->sensorList[fusion->sensorCount] = sensor;
	LOG.INFO("Added sensor number : %i",fusion->sensorCount);
	fusion->sensorCount++;
}

/**
 * Initializes a list of sensors, enough for 4 different sensors
 */
void SensorFusion_Init(SensorFusion * fusion)
{
        gps_enabled = FALSE;
	LOG.INFO("Initializing SensorFusion");
	fusion->sensorList = hMalloc(sizeof(Sensor*)*NUMBER_OF_SENSORS);
	fusion->sensorCount=0;
}


/*
  TODO: Allow for changing between a few different fusion types so that the odometry counting and other such things
	  are simply configureation options and nothing else
	  */

/**
 * Loads the SensorFusion.ini file and adds the sensors to the SensorFusion list
 */
void SensorFusion_LoadSensors(SensorFusion * fusion)
{
	if(fileExists("config/sensorFusion.ini"))
	{
		dictionary* config = iniparser_load("config/sensorFusion.ini");

        // Initialize the odometry  and adds it to the sensor fusion
		if(iniparser_getboolean(config,"Sensor:odometry",0))
		{
			LOG.INFO("Loading odometry sensor");
			/*Make microInterface part of the consrtuctor*/
			Sensor * Odometry = Odometry_Init();
			_SensorFusion_AddSensor(fusion,Odometry);
			LOG.INFO("Finished Adding Obometry");
		}
		else
		{
			LOG.INFO("Not Loading odometry sensor");
		}

        // Initialize the yaw and adds it to the sensor fusion
		if(iniparser_getboolean(config,"Sensor:yaw",0))
		{
			LOG.INFO("Loading yaw sensor");
			/*Make microInterface part of the consrtuctor*/
			Sensor * yaw = Yaw_Init();
			_SensorFusion_AddSensor(fusion,yaw);
			LOG.INFO("Finished Adding Yaw");
		}
		else
		{
			LOG.INFO("Not Loading yaw sensor");
		}

        // Initialize the GPS and adds it to the sensor fusion
		if(iniparser_getboolean(config,"Sensor:GPS",0))
		{
			LOG.INFO("Loading GPS sensor");
			/*Make microInterface part of the consrtuctor*/
			Sensor * gps = GPS_Init();
			_SensorFusion_AddSensor(fusion,gps);
			LOG.INFO("Finished Adding GPS");
                        gps_enabled = TRUE;
		}
		else
		{
			LOG.INFO("Not Loading GPS sensor");
                        gps_enabled = FALSE;
		}

        // Initialize the compass and adds it to the sensor fusion
		if(iniparser_getboolean(config,"Sensor:compass",0))
		{
			LOG.INFO("Loading Compass sensor");
			/*Make microInterface part of the consrtuctor*/
			Sensor * compass = Compass_Init();
			_SensorFusion_AddSensor(fusion,compass);
			LOG.INFO("Finished Adding Compass");
		}
		else
		{
			LOG.INFO("Not Loading Compass sensor");
		}

		iniparser_freedict(config);
	  }
	else
	{
		LOG.ERR("File sensorFusion.ini did not exist");
	}
}

/**
 * Boots up the sensor with it's current state.
 */
void SensorFusion_BootSensors(SensorFusion * fusion)
{
	int i;
	hasStartPosition = 0;
	for(i = 0; i < fusion->sensorCount; i++)
	{
        // Runs the void(*boot) function for easch sensor with it's sensor and current state as params
		fusion->sensorList[i]->boot(fusion->sensorList[i],&fusion->currentState);
	}
}

/**
 * Returns the pose from the sensors
 */
Pose SensorFusion_GetCurrentPose(SensorFusion * fusion) {
	Pose p;
	p.theta = limitTheta(Matrix_GetValue(fusion->currentState.State,3,1));
	p.x = Matrix_GetValue(fusion->currentState.State,1,1);
	p.y = Matrix_GetValue(fusion->currentState.State,2,1);

	p.vel = Matrix_GetValue(fusion->currentState.State,4,1);
	p.omega = Matrix_GetValue(fusion->currentState.State,5,1);
        p.yaw_bias = Matrix_GetValue(fusion->currentState.State, 6,1);
	p.x_variance = Matrix_GetValue(fusion->currentState.Variance,1,1);
	p.y_variance = Matrix_GetValue(fusion->currentState.Variance,2,2);
	p.theta_variance = Matrix_GetValue(fusion->currentState.Variance,3,3);
	p.vel_variance = Matrix_GetValue(fusion->currentState.Variance,4,4);
	p.omega_variance = Matrix_GetValue(fusion->currentState.Variance,5,5);
        p.yaw_bias_variance = Matrix_GetValue(fusion->currentState.Variance, 6,6);
	return p;
}
