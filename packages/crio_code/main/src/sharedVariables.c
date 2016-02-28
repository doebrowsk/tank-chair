/* 
 * File:   initSharedVariables.c
 * Author: eric
 *
 * Created on October 22, 2009, 3:20 PM
 */

#include "vxWorks.h"
#include "sharedVariables.h"

STATUS initSharedVariables() {
    heading_speed_commands.desiredHeading = 0.0;
    heading_speed_commands.desiredSpeed = 0.0;

    waypoint.heading = 0.0;
    waypoint.max_speed = 0.0;
    waypoint.p.x = 0.0;
    waypoint.p.y = 0.0;

    lastWaypoint.heading = 0.0;
    lastWaypoint.max_speed = 0.0;
    lastWaypoint.p.x = 0.0;
    lastWaypoint.p.y = 0.0;

    steering_mode = MODE_OFF;
    return(OK);
}
