/* 
 * File:   kalman.h
 * Author: ben
 *
 * Created on November 21, 2009, 9:16 PM
 */

#ifndef _KALMAN_H
#define	_KALMAN_H

#include "sensorfusion.h"

// Initializes the Kalman Sensor
SensorFusion * Kalman_Init();

// Updates the state and the values of all the Matricies
void Kalman_UpdateFilter(SensorFusion * fusion,WheelSpeedCommand speeds);

#endif	/* _KALMAN_H */
