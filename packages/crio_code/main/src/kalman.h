/* 
 * File:   kalman.h
 * Author: ben
 *
 * Created on November 21, 2009, 9:16 PM
 */

#ifndef _KALMAN_H
#define	_KALMAN_H

#include "sensorfusion.h"

SensorFusion * Kalman_Init();
void Kalman_UpdateFilter(SensorFusion * fusion,WheelSpeedCommand speeds);

#endif	/* _KALMAN_H */
