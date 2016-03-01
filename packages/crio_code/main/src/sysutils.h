#ifndef SYSUTILS_H
#define SYSUTILS_H

#include <time.h>

#define SEC_TO_NSEC(s) ((s) * 1000 * 1000 * 1000)

// Checks if the file exists
int fileExists(char * filename);

// Sleeps in given ms
int msSleep(long ms);

// Calculates x-y in ns, then converts to seconds before returning the float
double timespec_diff(const struct timespec *x, const struct timespec *y);


#endif 
