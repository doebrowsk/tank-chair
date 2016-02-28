#include "harlielog.h"
#include "stringextra.h"
#include "sysutils.h"
#include "minigzip.h"
#include "iniparser.h"
#include <stdio.h>
#include <time.h>
#include <string.h>

void _HarlieLogWrite(char *pType,char *pMsg);
void _LOG_INFO(char * msg, ...);
void _LOG_ERROR(char * msg, ...);
void _LOG_VERBOSE(char * msg, ...);
void _LOG_MEMORY(char * msg, ...);
void _LOG_DATA(char* msg, ...);
void _LOG_MATRIX(char* msg, ...);
void _LOG_KALMAN(char * msg, ...);
void _LOG_YAW(char * msg, ...);
void _LOG_GPS(char * msg, ...);
void _LOG_ODOMETRY(char * msg, ...);
void _LOG_CHADDATA(char * msg, ...);
void _LOG_COMPASS(char * msg, ...);
void _LOG_NULL(char * msg, ...);

FILE *HarlieLogFile;
char timeStrFinal[TIME_STR_LENGTH];
char OutString[OUT_BUFFER_LENGTH];
char buffer[MSG_BUFFER_LENGTH];
char buffer2[MSG_BUFFER_LENGTH];
char INITbuffer[MSG_BUFFER_LENGTH];
long int _startSec;

// Sets the function pointer 
void * _SetLogFunction(dictionary* config, char * LogName, void (*in)(char *, ...))
{
	if(iniparser_getboolean(config,LogName,1))
	{
        // @TODO: check if \0 is needed
		sprintf(INITbuffer,"%s is Enabled\0",LogName);
		_HarlieLogWrite("INIT",INITbuffer);
		return in;
	}
	else
	{
        // @TODO: check if \0 is needed
		sprintf(INITbuffer,"%s is Disabled\0",LogName);
		_HarlieLogWrite("INIT",INITbuffer);
		return &_LOG_NULL;
	}
}

// Initializes the log.ini file with initial values
void _InitLogStruct()
{
	FILE * pFile = fopen ("config/log.ini","r");
	if (pFile!=NULL)
	{
		_HarlieLogWrite("INIT","Config found for log.ini\nUsing the config");
		dictionary* config = iniparser_load("config/log.ini");
		LOG.DATA	= _SetLogFunction(config,"LOG:DATA",_LOG_DATA);
		LOG.ERR		= _SetLogFunction(config,"LOG:ERROR",_LOG_ERROR);
		LOG.GPS		= _SetLogFunction(config,"LOG:GPS",_LOG_GPS);
		LOG.INFO	= _SetLogFunction(config,"LOG:INFO",_LOG_INFO);
		LOG.KALMAN	= _SetLogFunction(config,"LOG:KALMAN",_LOG_KALMAN);
		LOG.MATRIX	= _SetLogFunction(config,"LOG:MATRIX",_LOG_MATRIX);
		LOG.MEMORY	= _SetLogFunction(config,"LOG:MEMORY",_LOG_MEMORY);
		LOG.ODOMETRY= _SetLogFunction(config,"LOG:ODOMETRY",_LOG_ODOMETRY);
		LOG.VERBOSE	= _SetLogFunction(config,"LOG:VERBOSE",_LOG_VERBOSE);
		LOG.YAW		= _SetLogFunction(config,"LOG:YAW",_LOG_YAW);
		LOG.CHADDATA= _SetLogFunction(config,"LOG:CHADDATA",_LOG_CHADDATA);
		LOG.COMPASS = _SetLogFunction(config,"LOG:COMPASS",_LOG_COMPASS);
		iniparser_freedict(config);
	}
	else
	{
		_HarlieLogWrite("INIT","All Logging Turned On, No log.ini config file found.");
		LOG.DATA = &_LOG_DATA;
		LOG.ERR = &_LOG_ERROR;
		LOG.GPS = &_LOG_GPS;
		LOG.INFO = &_LOG_INFO;
		LOG.KALMAN = &_LOG_KALMAN;
		LOG.MATRIX = &_LOG_MATRIX;
		LOG.MEMORY = &_LOG_MEMORY;
		LOG.ODOMETRY = &_LOG_ODOMETRY;
		LOG.VERBOSE = &_LOG_VERBOSE;
		LOG.YAW = &_LOG_YAW;
		LOG.CHADDATA = &_LOG_CHADDATA;
		LOG.COMPASS = &_LOG_COMPASS;
	}
	if(fileExists("config/log.ini"))
	{
		LOG.INFO("config/log.ini file loaded");
	}
	else
	{
		LOG.ERR("config/log.ini was not found");
	}
}

// Initializes the Harlie log 
void HarlieLogInit()
{
	time_t rawtime;
	struct tm * timeinfo;
	time ( &rawtime );
	timeinfo = localtime ( &rawtime );
	strftime (timeStrFinal,TIME_STR_LENGTH,"%m.%d.%Y.at.%H.%M",timeinfo);

    // @TODO: check if \0 is needed
	char ralf[60] = "\0";
	sprintf(ralf,"LOG.Back.%s.gz\0",timeStrFinal);

	struct timespec RTime;
	clock_gettime(CLOCK_REALTIME,&RTime);
	_startSec = RTime.tv_sec;

	if(fileExists("PSO.LOG"))
	{
		if(0 == fileExists(ralf))
		{
			// Do Comprerssion 
			printf("Starting Compression\n");
			gzFile gFile = gzopen(ralf,"wb+");
			FILE * logFile = fopen("PSO.LOG","rb");
			gz_compress(logFile, gFile);
			gzclose(gFile);
			fclose(logFile);
			printf("Compression Done\n");
		}
		remove("PSO.LOG");
	}
	char pfileName[20] = "PSO.LOG";
	HarlieLogFile = fopen(&pfileName[0], "w+");

	strftime (timeStrFinal,TIME_STR_LENGTH,"%c",timeinfo);
	_HarlieLogWrite("STARTING","STARTING");
	_HarlieLogWrite("START_TIME",&timeStrFinal[0]);

	// Initalize the Log struct
	_InitLogStruct();
}

// Closes the log file
void HarlieLogClose()
{
	if(HarlieLogFile != NULL)
	{
		fclose(HarlieLogFile);
	}
	HarlieLogFile = NULL;
}

// Prints the time since start and the Harlie log file if it exists
void _HarlieLogWrite(char *pType, char *pMsg)
{
	struct timespec Time;
	clock_gettime(CLOCK_REALTIME,&Time);
	//  @TODO: FIX SO NOT CRAPPY!!!
	sprintf(timeStrFinal,"%li:%li",Time.tv_sec-_startSec,Time.tv_nsec/1000000);
	sprintf(&OutString[0], "%s : %s\t%s \r\n",pType,timeStrFinal, pMsg);

	//printf("%s",&OutString[0]);

	if(HarlieLogFile != NULL)
	{
		fprintf(HarlieLogFile, "%s",&OutString[0]);
		if(ftell(HarlieLogFile) > MAX_LOG_SIZE)
		{
			fseek(HarlieLogFile,1000,SEEK_SET);
			fprintf(HarlieLogFile,"\nLOOP\n");
		}
	}
	fflush(HarlieLogFile);
}

// Prints the LOG INFO
void _LOG_INFO(char * msg, ...)
{
    // For Variadic arguments
	va_list args;
	va_start (args, msg);
	vsprintf (buffer,msg, args);
	va_end (args);
	_HarlieLogWrite("INFO",buffer);
}

// Prints the LOG VERBOSE
void _LOG_VERBOSE(char * msg, ...)
{
	va_list args;
	va_start (args, msg);
	vsprintf (buffer,msg, args);
	va_end (args);
	_HarlieLogWrite("VERBOSE",buffer);
}
// Prints the LOG DATA
void _LOG_DATA(char * msg, ...)
{
	va_list args;
	va_start (args, msg);
	vsprintf (buffer,msg, args);
	va_end (args);
	_HarlieLogWrite("DATA",buffer);
}

// Prints the LOG MATRIX
void _LOG_MATRIX(char * msg, ...)
{
	va_list args;
	va_start (args, msg);
	vsprintf (buffer,msg, args);
	va_end (args);
	_HarlieLogWrite("MATRIX",buffer);
}

// PRINTS the KALMAN LOG
void _LOG_KALMAN(char * msg, ...)
{
	va_list args;
	va_start (args, msg);
	vsprintf (buffer,msg, args);
	va_end (args);
	_HarlieLogWrite("KALMAN",buffer);
}

// Prints the GPS LOG
void _LOG_GPS(char * msg, ...)
{
	va_list args;
	va_start (args, msg);
	vsprintf (buffer,msg, args);
	va_end (args);
	_HarlieLogWrite("GPS",buffer);
}

// Prints the yaw log
void _LOG_YAW(char * msg, ...)
{
	va_list args;
	va_start (args, msg);
	vsprintf (buffer,msg, args);
	va_end (args);
	_HarlieLogWrite("YAW",buffer);
}

// Prints the odom log
void _LOG_ODOMETRY(char * msg, ...)
{
	va_list args;
	va_start (args, msg);
	vsprintf (buffer,msg, args);
	va_end (args);
	_HarlieLogWrite("ODOMETRY ",buffer);
}

// Prints the error log
void _LOG_ERROR(char * msg, ...)
{
  va_list args;
  va_start (args, msg);
  vsprintf (buffer,msg, args);
  sprintf(buffer2, "%s: pERROR =  %s", buffer, strerror(errno));
  va_end (args);
  _HarlieLogWrite("ERROR",buffer);
  printf("Error: %s \n",buffer);
}

// Prints the memeory log
void _LOG_MEMORY(char * msg, ...)
{
	va_list args;
	va_start (args, msg);
	vsprintf (buffer,msg, args);
	va_end (args);
	_HarlieLogWrite("MEMORY",buffer);
}

// Prints the CHADDATA
void _LOG_CHADDATA(char * msg, ...)
{
	va_list args;
	va_start (args, msg);
	vsprintf (buffer,msg, args);
	va_end (args);
	_HarlieLogWrite("CHADDATA",buffer);
}

// Prints the compass log
void _LOG_COMPASS(char * msg, ...)
{
	va_list args;
	va_start (args, msg);
	vsprintf (buffer,msg, args);
	va_end (args);
	_HarlieLogWrite("COMPASS",buffer);
}

// Does nothing? why is this even written
void _LOG_NULL(char * msg, ...)
{
}
