#include "sensor.h"
#include <time.h>

#ifndef YAW_H
#define YAW_H

typedef struct YawCL{
	// From the INI file
	uint16_t poleDt;
	float samples; // How many to sum at a time
	int32_t bootupCycles;
	float offset;
	float rateConstant;
	float offsetGain;
	float stopTime;
	float stopOffsetGain;
	struct timespec sinceLastOdometeryChange;
}Yaw;

Sensor * Yaw_Init();
void Yaw_Boot(Sensor * sensor,SensorVariable * start);
void Yaw_Update(Sensor * sensor, const SensorVariable state);

#endif
