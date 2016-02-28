#include"microprocessorinterface.h"
#include"statevariable.h"
#include"matrixLibStack.h"
#include"MainCRIO.h"
#ifndef SENSOR_H
#define SENSOR_H
typedef struct SensorCL Sensor;

struct SensorCL {
	SensorVariable * state;
	matrix H;
	void (*boot)(Sensor*,SensorVariable *);
	void (*update)(Sensor*,SensorVariable);
	/* if 1 true 0 false*/
	int hasBeenUpdated;
};

void Sensor_Init(Sensor *sensor);
#endif
