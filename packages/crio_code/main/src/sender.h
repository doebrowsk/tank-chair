/* 
 * File:   sender.h
 * Author: Eric Perko
 *
 * Created on August 31, 2009, 6:26 PM
 */

#include <vxWorks.h>
#include "packets.h"

#ifndef _SENDER_H
#define	_SENDER_H

#define PSO_PACKET_READY 3

#ifdef	__cplusplus
extern "C" {
#endif

Pose poseToBroadcast;
GPSNetworkingPacket gpsToBroadcast;
int gps_is_new;

STATUS startSenders();

STATUS sendPSOPacket();
STATUS sendDiagnosticsPacket();
STATUS sendGPSPacket();

STATUS initSenders();

#ifdef	__cplusplus
}
#endif

#endif	/* _SENDER_H */

