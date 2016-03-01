/* 
 * File:   sharedVariables.h
 * Author: eric
 *
 * Created on October 22, 2009, 3:19 PM
 */
#include "packets.h"
#include "vxWorks.h"
#include <time.h>

#ifndef _SHAREDVARIABLES_H
#define	_SHAREDVARIABLES_H

#ifdef	__cplusplus
extern "C" {
#endif

/* TODO we need mutexs for this variable */
struct timespec last_command_received;
HeadingSpeedCommand heading_speed_commands;
AngularRateSpeedCommand angular_rate_commands;
WaypointCommand waypoint;
WaypointCommand lastWaypoint;
int8_t steering_mode;

#define MODE_WAGON 1
#define MODE_VECTOR 2
#define MODE_TWIST 3
#define MODE_OFF 0

STATUS initSharedVariables();

#ifdef	__cplusplus
}
#endif

#endif	/* _SHAREDVARIABLES_H */

