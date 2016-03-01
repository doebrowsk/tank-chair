/* 
 * File:   differential_steering.h
 * Author: Eric Perko
 *
 * Created on October 4, 2009, 9:06 PM
 */

#ifndef _DIFFERENTIAL_STEERING_H
#define	_DIFFERENTIAL_STEERING_H

#include "packets.h"
#include <vxWorks.h>


#ifdef	__cplusplus
extern "C" {
#endif
    #define MAX_TURNING 100

    // Struct to hold gain values
    typedef struct _gain_t {
        float proportionalGain;
        float integralGain;
        float derivativeGain;
    } PIDGains;

    // Struct to hold error values
    typedef struct _PID_errors_t {
        float propotionalError;
        float integralError;
        float derivativeError;
    } PIDErrors ;

    // Struct for wheel speed command
    typedef struct s_speed_t {
        float rightWheelSpeed;
        float leftWheelSpeed;
    } WheelSpeedCommand;

    // Initializes the differential steerer
    STATUS initDifferentialSteering();

    // Calculates the error and steers accounting for the error
    STATUS differentialSteering(const HeadingSpeedCommand *commands, WheelSpeedCommand * const speeds, const float currentHeading);

    // Takes a twist command and assigns the wheel speeds according
    STATUS twistSteering(const AngularRateSpeedCommand *twist_commands, WheelSpeedCommand * const speeds, const float currentHeading);
    
    // Calculates the next minimal steering angle 
    float calculateMinimalSteeringAngle(float desiredHeading, float currentHeading);

#ifdef	__cplusplus
}
#endif

#endif	/* _DIFFERENTIAL_STEERING_H */
