#include "harlielog.h"
#include <stdio.h>
#include <vxWorks.h>
#include <taskLib.h>
#include <strings.h>
#include "basicfusion.h"
#include "harliealloc.h"
#include "timerutils.h"
#include "sender.h"
#include "MainCRIO.h"
#include <eventLib.h>
#include "fpga.h"
#include "dkalman.h"
#include "differentialSteering.h"
#include "sharedVariables.h"
#include "wagonSteering.h"
#include "sonar.h"
#include "sysutils.h"

SensorFusion *fusion;
WheelSpeedCommand speeds;
struct timespec temp_time;

STATUS _steer();
UINT32 _PSOWaitUntilExit();
int _mainLoop();

#define CMD_TIMEOUT_IN_SEC (0.1)

// Creates tasks that start
int taskCreate (
        char *  name,
        int     priority,
        int     options,
        int     stackSize,
        FUNCPTR entry,
        int     arg1,
        int     arg2,
        int     arg3,
        int     arg4,
        int     arg5,
        int     arg6,
        int     arg7,
        int     arg8,
        int     arg9,
        int     arg10
)
{
        int ret;
        char *memArea;
        char *pStackBase;

        // initialize TCB and stack space 

        memArea = (char *) malloc (STACK_ROUND_UP(stackSize)+STACK_ROUND_UP(stackSize));
		char *tcbPt = (char *)malloc(sizeof(WIND_TCB) + sizeof(WIND_TCB));
		tcbPt = (char *)((int)tcbPt + sizeof(WIND_TCB));
        if (memArea == NULL)
        {
            return ERROR;
        }

        // Calculate the base of the stack. Direction of stack growth depends on architecture.

        #if (_STACK_DIR == _STACK_GROWS_DOWN)
        // We Use This one
        pStackBase = (char *)(memArea + STACK_ROUND_UP(stackSize));
        #else
        pStackBase = memArea + STACK_ROUND_UP (sizeof (WIND_TCB) + 16);
        #endif

        // Initialize task 

        ret = taskInit ((WIND_TCB *) tcbPt,
                name,
                priority,
                options,
                pStackBase,
                stackSize,
                entry,
                arg1, arg2, arg3, arg4, arg5,
                arg6, arg7, arg8, arg9, arg10);
		
        if (ret == ERROR)
        {
                return ERROR;
        }
		ret=taskActivate(((int)(tcbPt)));
        if (ret == ERROR)
        {
                return ERROR;
        }
		else
        {
				return((int)(tcbPt));
        }
}

/* This is the signature for anything that the timer should call*/
void _doLoop(timer_t timerid, int arg) 
{
/*
 * TODO This needs to be broken into lots of little functions
*/

        TASK_DESC info;
        if((taskInfoGet(PSOTask, &info)) == ERROR) 
        {
            LOG.ERR("PSOTask: error getting task info");
        }
        else
        {
            LOG.MEMORY("current stack size %d", info.td_stackCurrent);
            LOG.MEMORY("stack size - current stack usage: %d", info.td_stackSize - info.td_stackCurrent);
        }
	LOG.INFO("Running filter");
	fusion->updateFilter(fusion,speeds);
	poseToBroadcast = SensorFusion_GetCurrentPose(fusion);
	poseToBroadcast.sonar_ping_1 = getSonarPing(1);
	poseToBroadcast.sonar_ping_2 = getSonarPing(2);
	poseToBroadcast.sonar_ping_3 = getSonarPing(3);
	poseToBroadcast.sonar_ping_4 = getSonarPing(4);
	poseToBroadcast.sonar_ping_5 = getSonarPing(5);
	
	eventSend(PSOSenderTask, PSO_PACKET_READY);
	_steer();
	Microprocessor->setLeftWheelVelocity(speeds.leftWheelSpeed);
	Microprocessor->setRightWheelVelocity(speeds.rightWheelSpeed);
	LOG.VERBOSE("Tried to execute commands: heading: %f speed: %f\nMade speeds left: %f right: %f", 
                    heading_speed_commands.desiredHeading, heading_speed_commands.desiredSpeed, 
                    speeds.leftWheelSpeed, speeds.rightWheelSpeed);
}

// Steers the robot using differential steering or twists
STATUS _steer() 
{
	if(steering_mode == MODE_WAGON) 
    {
			LOG.VERBOSE("About to do wagon steering");
			WaypointCommand currentPositionwaypoint;
			currentPositionwaypoint.p.x = poseToBroadcast.x;
			currentPositionwaypoint.p.y = poseToBroadcast.y;
			currentPositionwaypoint.heading = poseToBroadcast.theta;
			currentPositionwaypoint.max_speed = waypoint.max_speed;
			wagonSteering(&lastWaypoint, &waypoint, &heading_speed_commands, &poseToBroadcast);
			LOG.VERBOSE("About to differential steer");
			differentialSteering(&heading_speed_commands, &speeds, poseToBroadcast.theta);
	}
    else if(steering_mode == MODE_VECTOR)
    {      
        LOG.VERBOSE("About to differential steer");
        differentialSteering(&heading_speed_commands, &speeds, poseToBroadcast.theta);
    }
    else if(steering_mode == MODE_TWIST) 
    {
        LOG.VERBOSE("About to do twist steering");
        clock_gettime(CLOCK_REALTIME, &temp_time);
        if(timespec_diff(&temp_time, &last_command_received) > CMD_TIMEOUT_IN_SEC) 
        {
            LOG.ERR("Command timeout. No command received in the last %f seconds", CMD_TIMEOUT_IN_SEC);
            angular_rate_commands.desiredAngularRate = 0.0;
            angular_rate_commands.desiredSpeed = 0.0;
        }             
        twistSteering(&angular_rate_commands, &speeds, poseToBroadcast.theta);
    }
	return (OK);
}

//  Creates the PSO task which starts at the main loop.
STATUS startMainLoop() 
{
    int task = taskCreate("PSO", 100, VX_FP_TASK, 0xF0000, (FUNCPTR) &_mainLoop, 0,0,0,0,0,0,0,0,0,0);
	if(task == ERROR) 
    {
		LOG.ERR("Error spawning PSO task");
		return(ERROR);
	} 
    else 
    {
		PSOTask = task;
		printf("PSOTask %d\n", task);
	}
	return(OK);
}

// Initializes the Kalman, loads then boots the sensors and starts the timers
int _mainLoop()
{

	LOG.INFO("Init fuison (Kalman)");
	fusion = Kalman_Init();
	LOG.INFO("Loading Sensors");
	SensorFusion_LoadSensors(fusion);
	LOG.INFO("Booting Sensors");
	SensorFusion_BootSensors(fusion);
	LOG.INFO("Starting the timer Sensors");
	startTimer(PSOUpdateRateInMs, (VOIDFUNCPTR) &_doLoop);

	// so that the timers don't go poof
	_PSOWaitUntilExit();
	Microprocessor->close();
	return 0;
}

// A blocking function that will wait till exit event is recieved
UINT32 _PSOWaitUntilExit() 
{
	UINT32 events;
	while(eventReceive(PSO_EXIT, EVENTS_WAIT_ALL, WAIT_FOREVER, &events) != OK) {}
	return events;
}

STATUS stopMainLoop() 
{
	eventSend(PSOTask, PSO_EXIT);
	PSOTask = -1;
	return (OK);
}
