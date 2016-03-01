#include "sysutils.h"
#include "harlielog.h"
#include <stdio.h>
#include <time.h>

// Checks if the file exists
int fileExists(char * filename)
{
	FILE * pFile = fopen (filename,"r");
	if (pFile != NULL)
	{
		//LOG.VERBOSE("File exists : %s",filename);
		fclose (pFile);
		return 1;
	}
	//LOG.VERBOSE("File does not exists : %s",filename;
	return 0;
}

// Sleeps in given ms
int msSleep(long ms)
{
	struct timespec sleep;
	struct timespec remaining;
	sleep.tv_nsec = ms % 1000 * 1000000;
	sleep.tv_sec = (int)(ms / 1000);
	/*
	printf("Sleeping for %d ns and %d second\n", sleep.tv_nsec,sleep.tv_sec);
	*/
	return nanosleep(&sleep,&remaining);
}

// calculates x-y in nanoseconds, then converts to seconds before returning the float 
double timespec_diff(const struct timespec *x, const struct timespec *y) 
{
    int64_t x_in_nano = (SEC_TO_NSEC(x->tv_sec) + x->tv_nsec);
    int64_t y_in_nano = (SEC_TO_NSEC(y->tv_sec) + y->tv_nsec);

    return ((x_in_nano - y_in_nano) / 1e9);
}
