/* 
 * File:   wagonSteering.h
 * Author: ericperk
 *
 * Created on November 2, 2009, 4:47 PM
 */
#include "packets.h"

#ifndef _WAGONSTEERING_H
#define	_WAGONSTEERING_H

#ifdef	__cplusplus
extern "C" {
#endif

    // Steers the robot given the last waypoint, the current waypoint, heading and pose
    STATUS wagonSteering(const WaypointCommand * lastWaypoint, const WaypointCommand * waypoint, HeadingSpeedCommand * const hscommand, const Pose * pose);

#ifdef	__cplusplus
}
#endif

#endif	/* _WAGONSTEERING_H */

