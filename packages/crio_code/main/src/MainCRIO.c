#include <eventLib.h>
#include <stdio.h>
#include <time.h>
#include <unistd.h>
#include <vxWorks.h>
#include "sysutils.h"
#include "iniparser.h"
#include "sender.h"
#include "MainCRIO.h"
#include "timerutils.h"
#include "PSOmain.h"
#include "receiver.h"
#include "differentialSteering.h"
#include "sharedVariables.h"
#include "harlielog.h"
#include "matrixlibtestppc.h"

void _triggerPSOSender(timer_t timerid, int arg);
void _initCRIOConstants(void);
STATUS _initCRIO();
STATUS _runTests();

/**
 * Starts senders and receivers and then goes to the main loop
 */
void MainCRIO(void) {
	_initCRIO();
    LOG.INFO("Running tests");
    /*_runTests();*/
	LOG.INFO("Starting Senders");
	startSenders();
	LOG.INFO("Starting Receivers");
	startReceivers();
	LOG.INFO("Starting Main Loop");
	/*The main loop starts and runs the PSO*/
	startMainLoop();
	/*
	   startTimer(10, &_triggerPSOSender);
	 */
	printf("Exiting main loop...\n");
}

/**
 * Performs matrix tests
 */
STATUS _runTests() {
    LOG.INFO("Running Matrix tests");
    if(runMatrixTests() != OK) {
        LOG.ERR("Matrix tests failed");
    }
    return (OK);
}

/**
 * Initializes configuration files and log files
 */
STATUS _initCRIO() {
	PSOSenderTask = -1;
	CommandReceiverTask = -1;
	PSOTask = -1;
	HarlieLogInit();
	LOG.INFO("Starting the file");
	_initCRIOConstants();
	LOG.INFO("Init Microprocessor");
	Microprocessor = MicroprocessorInit();
	LOG.INFO("Booting the Microprocessor");
	Microprocessor->boot();
	LOG.INFO("Init Senders");
	initSenders();
	LOG.INFO("Init Receivers");
	initReceivers();
	LOG.INFO("Init Differential Steering");
	initDifferentialSteering();
	LOG.INFO("Init Shared Variables");
	initSharedVariables();
	return(OK);
}

/**
 * Triggers an event send to the PSO sender task
 */
void _triggerPSOSender(timer_t timerid, int arg) {
	eventSend(PSOSenderTask, PSO_PACKET_READY);
}

/**
 * Blocking function that waits for all events?? before returning them
 */
UINT32 waitUntilExit() {
	UINT32 events;
	while(eventReceive(MAIN_EXIT, EVENTS_WAIT_ALL, WAIT_FOREVER, &events) != OK) {
	}
	return events;
}

/**
 * Initializes the CRIO and odom by creating the ini file for the CRIO and for the odom as well
 */
void _initCRIOConstants(void)
{
	/*Create INI */
	if(fileExists("config/CRIO.ini"))
	{
		LOG.VERBOSE("Reading CRIO.ini");
		dictionary* config = iniparser_load("config/CRIO.ini");
		PSOUpdateRateInMs = iniparser_getint(config,"PSO:UpdateRateInMs",10);
		PIDUpdateRateInMs = iniparser_getint(config,"PID:UpdateRateInMs",10);
		PSOUpdateRateInS = PSOUpdateRateInMs/1000.0;
		LOG.VERBOSE("Reading CRIO.ini Finished");
	}
	else
	{
		LOG.VERBOSE("CRIO.ini could not be found.  Crash Crash!!!");
	}
	if(fileExists("config/odometry.ini")) {
		LOG.VERBOSE("Reading odometry.ini for MainCRIO");
		dictionary* config = iniparser_load("config/odometry.ini");
		trackInMeters = (float) iniparser_getdouble(config,"DistanceBetweenWheels:track",0.5);
		iniparser_freedict(config);
		LOG.VERBOSE("Reading odometry.ini for MainCRIO finished");
	} else {
		LOG.ERR("odometry.ini could not be found. MainCRIO needed it f     or the trackwidth");
	}
}
