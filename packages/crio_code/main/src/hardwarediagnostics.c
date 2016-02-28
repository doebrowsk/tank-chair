#include "fpga.h"
#include <vxWorks.h>
#include "harlielog.h"

// TODO:determine why everything is commented out

STATUS checkVoltages()
{
/*
    char status = 'T';

    float V24 = FPGA_Get24VLineRead();

    float V24L = 23.5;
    float V24H = 29;
    if( V24 < V24L){
        LOG.ERR("BATT LOW: 24V (%fV) Line Reads Below %fV!",V24,V24L);
        status = 'F';
    } else if (V24 > V24H) {
        LOG.ERR("BATT HIGH: 24V (%fV) Line Reads Above %fV!",V24,V24H);
        status = 'F';
    } else {
        LOG.VERBOSE("BATT: 24V (%fV) Line Reads OK!",V24);
    }

    float VcRIO = FPGA_GetcRIOLineRead();

    float VcRIOL = 20;
    float VcRIOH = 28;
    if( VcRIO < VcRIOL){
        LOG.ERR("VOLT LOW: cRIO (%fV) Line Reads Below %fV!",VcRIO,VcRIOL);
        status = 'F';
    } else if (VcRIO > VcRIOH) {
        LOG.ERR("VOLT HIGH: cRIO (%fV) Line Reads Above %fV!",VcRIO,VcRIOH);
        status = 'F';
    } else {
        LOG.VERBOSE("VOLT: cRIO (%fV) Line Reads OK!",VcRIO);
    }

    float V14 = FPGA_Get14VLineRead();

    float V14L = 13.6;
    float V14H = 14;
    if( V14 < V14L){
        LOG.ERR("VOLT LOW: 13.8V (%fV) Line Reads Below %fV!",V14,V14L);
        status = 'F';
    } else if (V14 > V14H) {
        LOG.ERR("VOLT HIGH: 13.8V (%fV) Line Reads Above %fV!",V14,V14H);
        status = 'F';
    } else {
        LOG.VERBOSE("VOLT: 13.8V (%fV) Line Reads OK!",V14);
    }

    float V5 = FPGA_Get5VLineRead();

    float V5L = 4.9;
    float V5H = 5.2;
    if( V5 < V5L){
        LOG.ERR("VOLT LOW: 5V (%fV) Line Reads Below %fV!",V5,V5L);
        status = 'F';
    } else if (V5 > V5H) {
        LOG.ERR("VOLT HIGH: 5V (%fV) Line Reads Above %fV!",V5,V5H);
        status = 'F';
    } else {
        LOG.VERBOSE("VOLT: 5V (%fV) Line Reads OK!",V5);
    }

    if(status == 'F'){
        return (ERROR);
    } else {
        return (OK);
    }
}
*/
    return (OK);
}
/*
2. Switches

    * Check for Both Down (0V) (Loop until condition Met)
    * Check for Left Up (Loop until Condition Met)
    * Check for Right Up (Loop until Condition Met)
*/

/*
 3. ESTOP

    * Check for Estop disabled (24V or True)
    * Test cRIO Estop control (Line will drop to 0V or False)
    * Return ESTOP to disabled (Line will return to 24V)
      X No need to test wireless and button in code since they make nice audible clicks
*/

/*STATUS checkESTOP(){
    char status = 'T';

    if(status == 'F'){
        return (ERROR);
    } else {
        return (OK);
    }
}*/
