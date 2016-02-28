/* 
 * File:   hardwarediagnostics.h
 * Author: chad
 *
 * Created on November 22, 2009, 7:46 AM
 */

#include "fpga.h"
#include <vxWorks.h>
#include "harlielog.h"

#ifndef _HARDWAREDIAGNOSTICS_H
#define _HARDWAREDIAGNOSTICS_H

STATUS checkVoltages();

STATUS checkESTOP();

typedef struct HARLIESTATUS_t {
    int32_t strlen;
    char * string;
    int8_t status_code;
} HarlieStatus;

#endif /* _HARDWAREDIAGNOSTICS_H */
