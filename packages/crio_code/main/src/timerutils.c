/* 
 * File:   timerutils.c
 * Author: Eric Perko
 *
 * Created on September 17, 2009, 9:16 PM
 */
#include <time.h>
#include <stdio.h>
#include "harlielog.h"

// Starts the timer for the given amount of time in ms. Possibly runs the handler after the timer ends.
STATUS startTimer(uint8_t ms, VOIDFUNCPTR handler) 
{
    timer_t timerid;
    struct itimerspec value;

    value.it_value.tv_nsec = 1000 * 1000 * ms;
    value.it_value.tv_sec = 0;

    value.it_interval.tv_nsec = 1000 * 1000 * ms;
    value.it_interval.tv_sec = 0;

    if (timer_create(CLOCK_REALTIME, NULL, &timerid) == ERROR) 
    {
        LOG.ERR("Error creating timer");
        return(ERROR);
    }
    STATUS temp = timer_connect(timerid, handler, 0);
    STATUS temp2 = timer_settime(timerid, 0, &value, NULL);
    if((temp == ERROR) || (temp2 == ERROR)) 
    {
        LOG.ERR("Error connecting or setting the timer");
        return(ERROR);
    }
    return(OK);
}

