#include "sensor.h"
#include "microprocessorinterface.h"
#include "statevariable.h"
#include "packets.h"


#ifndef SENSORFUSION_H
#define SENSORFUSION_H
#define NUMBER_OF_SENSORS 4

typedef struct {
	Sensor ** sensorList;
	int sensorCount;
	SensorVariable currentState;
	void (*updateFilter)();
	void * data;
}SensorFusion;

float startX;
float startY;
float startTheta;
int hasStartPosition;

uint8_t gps_enabled;

void SensorFusion_Init(SensorFusion * fusion);
void SensorFusion_LoadSensors(SensorFusion * fusion);
void SensorFusion_BootSensors(SensorFusion * fusion);
Pose SensorFusion_GetCurrentPose(SensorFusion * fusion);

#endif
