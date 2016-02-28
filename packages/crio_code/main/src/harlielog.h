#ifndef HARLIELOG_H
#define HARLIELOG_H

#include <stdarg.h>


#define TIME_STR_LENGTH 128
#define MSG_BUFFER_LENGTH 256
#define OUT_BUFFER_LENGTH 512
// CURRENTLY SET TO 25 MB
#define MAX_LOG_SIZE 26214400

int LogOk;

// Struct containing pointers to all the logging functions
typedef struct {
	void (*INFO)(char *, ...);
    // So ERR is instead of ERROR. ERROR is some type of built in.
	void (*ERR)(char *, ...);
	void (*VERBOSE)(char *, ...);
	void (*MEMORY)(char *, ...);
	void (*DATA)(char *, ...);
	void (*MATRIX)(char *, ...);
	void (*KALMAN)(char *, ...);
	void (*YAW)(char *, ...);
	void (*GPS)(char *, ...);
	void (*ODOMETRY)(char *, ...);
	void (*CHADDATA)(char *, ...);
	void (*COMPASS)(char *, ...);
}LogStruct;

// Struct for logs
LogStruct LOG;

// Initializes the Logger
void HarlieLogInit();

// Closes the logger
void HarlieLogClose();
#endif

