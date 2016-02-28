/*This holds the header info for the compass sensor*/

/*#include <stdint.h>*/
#include <math.h>
#include "sensor.h"
#include "sysutils.h"
#include "iniparser.h"

#ifndef COMPASS_H
#define COMPASS_H

#define SIZE_OF_COMPASS_FIFO 4096

// Struct holding the GPS sensor values
typedef struct GPS_Packet_BestVel_CL{
	int32_t solStat;
	int32_t posType;
	float latency;
	float age;
	double horizontalVel;
	double heading;
	double verticalVel;
	int8_t CRCstart;
	// Next think is the crc check
}GPS_Packet_BestVel;

// Initializes the compass sensor
Sensor * Compass_Init();

// Updates the sensor values 
void Compass_Update(Sensor * sensor,const SensorVariable state);

//Loads the compass.ini file and creates matrices for the sensor state
void Compass_Boot(Sensor * sensor,SensorVariable * start);

#endif
