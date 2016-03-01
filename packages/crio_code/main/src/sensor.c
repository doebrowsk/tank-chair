#include "sensor.h"
#include "harliealloc.h"

void Sensor_Init(Sensor *sensor)
{
	sensor->state = hMalloc(sizeof(SensorVariable));
}



