
#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "sensor.h"
#include "statevariable.h"
#include <vxWorks.h>



typedef struct OdometryCL{
	/*From the INI file*/
	float    leftWheelMeterPerTick;
	float    rightWheelMeterPerTick;
	float    leftMotorMeterPerTick;
	float    rightMotorMeterPerTick;
	/* Chad Comment: Eric and I measured the track as .561975 meters. */
	float    trackInMeters; /*0.5677*/
	/*Looping Variables*/
	int32_t rightWheelTicks;
	int32_t leftWheelTicks;
}Odometry;


Sensor * Odometry_Init();
void Odometry_Boot(Sensor * sensor,SensorVariable * start);
void Odometry_Update(Sensor * sensor, const SensorVariable state);
float  Odometry_GetdTheta();
float Odometry_GetRightWheelVelocity();
float Odometry_GetLeftWheelVelocity();

#endif
