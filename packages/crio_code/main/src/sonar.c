#include "sonar.h"
#include "fpga.h"
#include "microprocessorinterface.h"
#include "harlielog.h"
#include "MainCRIO.h"

const float frontSonarMperUS = 0.0001728;
const float frontSonarMOffset = -0.04445; 

// Returns the sonar reading and limits the reading into the reasonable range
float getSonarPing(int sonar) {
    uint32_t ping_us=0;
    if(sonar==1)
    {
        ping_us = Microprocessor->getSonarPing_1();
    }
    else if(sonar==2)
    {
        ping_us = Microprocessor->getSonarPing_2();
    }
    else if(sonar==3)
    {
        ping_us = Microprocessor->getSonarPing_3();
    }
    else if(sonar==4)
    {
        ping_us = Microprocessor->getSonarPing_4();
    }
    else if(sonar==5)
    {
        ping_us = Microprocessor->getSonarPing_5();
    }
    else
    {
        ping_us = Microprocessor->getSonarPing_1();
    }
    if(ping_us == 0) 
    {
        ping_us = 735; /* corresponds to a minimum reading */
    }
    if(ping_us > 300000) 
    {
        ping_us = 300000; /* corresponds to a maximum reading */
    }
    float ping_m = ping_us * frontSonarMperUS;
    LOG.DATA("SonarPing %d = %f",sonar, ping_m);
    return ping_m;
}
