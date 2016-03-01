/* 
 * File:   receiver.h
 * Author: ericperk
 *
 * Created on September 25, 2009, 12:08 PM
 */

#ifndef _RECEIVER_H
#define	_RECEIVER_H

#ifdef	__cplusplus
extern "C" {
#endif

    STATUS initReceivers();

    STATUS startReceivers();

    STATUS receiveCommandPacket();

    char command_should_estop;


#ifdef	__cplusplus
}
#endif

#endif	/* _RECEIVER_H */

