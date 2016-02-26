/*
 * File:   MainCRIO.h
 * Author: igvc
 *
 * Created on September 2, 2009, 1:09 AM
 */

#include <vxWorks.h>
#include "microprocessorinterface.h"

#ifndef _MAINCRIO_H
#define	_MAINCRIO_H

#define MAIN_EXIT 1
#define PSO_EXIT 2

int PSOSenderTask;
int CommandReceiverTask;
int PSOTask;
int PSOUpdateRateInMs;
float PSOUpdateRateInS;
int PIDUpdateRateInMs;
const MicroprocessorInterface * Microprocessor;
float trackInMeters;
UINT32 waitUntilExit();

#endif	/* _MAINCRIO_H */

