#ifndef BASICFUSION_H
#define BASICFUSION_H

#include "sensorfusion.h"
#include "differentialSteering.h"

// Initializes the senor fusion struct
SensorFusion * BasicFusion_Init();
// Updates the sensor values 
void BasicFusion_UpdateFilter(SensorFusion * fusion,WheelSpeedCommand speeds);

#endif
