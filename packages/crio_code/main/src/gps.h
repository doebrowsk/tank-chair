//#include <stdint.h>
#include <math.h>
#include "sensor.h"
#include "sysutils.h"
#include "iniparser.h"
#include "packets.h"

#ifndef GPS_H
#define GPS_H

#define SIZE_OF_GPS_ARRAY 100

//Word align to the byte :) FTW 
#pragma pack(push,1)
typedef struct GPS_Packet_Header_CL{
	int8_t sync1;
	int8_t sync2;
	int8_t sync3;
	uint8_t lengthHeader;
	uint16_t messageId;
	int8_t messageType;
	uint8_t portAddress;
	uint16_t lengthMessage; // This is the the size of the data inbetween the end of the header and the crc value... Does not include header of crc value
	int16_t sequence;
	int8_t idleTime;
	int8_t timeStatus;
	int16_t gpsWeek;
	int32_t gpsWeekMs;
	int32_t recieverStatus;
	int16_t reserved;
	int16_t recieverSWVVersion;
	int8_t  dataStart;
	/*
	DATA!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	*/
}GPS_Packet_Header;

typedef struct GPS_Packet_BestPos_CL{
	uint32_t solStat;
	uint32_t posType;
	double latitude;
	double longitude;
	double height;
	float undulation;
	int32_t datumId;
	float latitudeStdDev;
	float longitudeStdDev;
	float heightStdDev;
	int32_t stn_id;
	float differentialAge;
	float solutionAge;
	int8_t satellitesTracked; // this is how many can it see
	int8_t satellitesUsed; // this is how many were used in the solution creation
	int8_t GPSPlusGLONASSL1;
	int8_t GPSPLUSGLONASSL1L2;
	int8_t reserved1;
	int8_t extendedSolution;
	int8_t reserved2;
	int8_t signalMask;
	int8_t CRCStart;
	/*
	Next think is the crc check
	*/
}GPS_Packet_BestPos;

#pragma pack(pop)

Sensor * GPS_Init();
void GPS_Boot(Sensor * sensor,SensorVariable * start);
void GPS_Update(Sensor * sensor,const SensorVariable state);
void GPS_MakeGPSPacket(const GPS_Packet_BestPos *gps_pos, GPSNetworkingPacket * const gps_packet);
#endif
