/* 
 * File:   timerutils.h
 * Author: ericperk
 *
 * Created on September 17, 2009, 9:21 PM
 */

#ifndef _TIMERUTILS_H
#define	_TIMERUTILS_H

#ifdef	__cplusplus
extern "C" {
#endif

// Starts the timer for the given amount of time
STATUS startTimer(uint8_t ms, VOIDFUNCPTR handler);

#ifdef	__cplusplus
}
#endif

#endif	/* _TIMERUTILS_H */

