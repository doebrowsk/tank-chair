#include "harliealloc.h"
#include "sensor.h"
#include "harlielog.h"
#include "gps.h"
#include "sender.h"
#include "MainCRIO.h"
#include <math.h>
#include <string.h>
#include "differentialSteering.h"

double GPS_LatitudeRatio;
double GPS_LongitudeRatio;
float GPS_longLowLimit;
float GPS_longHighLimit;
float GPS_latLowLimit;
float GPS_latHighLimit;
float GPS_angleDtLength;
float GPS_angleVarianceMult;
double GPS_latOffset;
double GPS_longOffset;
float GPS_angleLastLat;
float GPS_angleLastLong;
float GPS_anglePrevious;
float GPS_angleSum;
float GPS_velocityMin;
float GPS_velcityBeliefFactor;
int GPS_networkOutputOnly;

uint8_t GPS_data[SIZE_OF_GPS_ARRAY];

// Changes the endian of the 16 bit value
uint16_t _Convert16Int(const uint16_t A)
{
	uint16_t B;
	unsigned char *dst = (unsigned char *)&B;
	unsigned char *src = (unsigned char *)&A;
	dst[0] = src[1];
	dst[1] = src[0];
	return B;
}

// Changes the endian of the 32 bit value
uint32_t _Convert32Int(const uint32_t A)
{
	uint32_t B;
	unsigned char *dst = (unsigned char *)&B;
	unsigned char *src = (unsigned char *)&A;
	dst[0] = src[3];
	dst[1] = src[2];
	dst[2] = src[1];
	dst[3] = src[0];
	return B;
}

// Changes the endian of the 64 bit value
uint64_t _Convert64Int(const uint64_t A)
{
	uint64_t B;
	uint8_t *dst = (uint8_t *)&B;
	uint8_t * src = (uint8_t *)&A;
	dst[0] = src[7];
	dst[1] = src[6];
	dst[2] = src[5];
	dst[3] = src[4];
	dst[4] = src[3];
	dst[5] = src[2];
	dst[6] = src[1];
	dst[7] = src[0];
	return B;
}

// Converts a double reading into a double value with correct endian
double _Convert64Double(const double D)
{
	uint64_t * D64 = (uint64_t *)&D;
	uint64_t retInt = _Convert64Int(*D64);
	double * ret = (double *)&retInt;
	return *ret;
}

// Converts a float reading into a float value with correct endian
float _Convert32Float(const float D)
{
	uint32_t * D32 = (uint32_t * )&D;
	uint32_t retInt = _Convert32Int(*D32);
	float * ret = (float *)&retInt;
	return *ret;
}

// Initializes the GPS struct 
Sensor * GPS_Init(void)
{
	Sensor* sensor = hMalloc(sizeof(Sensor));
	sensor->boot = &GPS_Boot;
	sensor->update = &GPS_Update;
	Sensor_Init(sensor);
	return sensor;
}

/*
   see if packet exists
   returns pointer to start of a header packet

   Parse packets
   Have a function for each type of packet
*/
// Contants
const uint32_t pos_type_single = 16;
const uint32_t pos_type_omnistar = 20;
const uint32_t pos_type_omnistar_hp = 64;
const uint32_t sol_status_computed = 0;

// Creates a networking packet from the best gps position reading??
void GPS_MakeGPSPacket(const GPS_Packet_BestPos *gps_pos, GPSNetworkingPacket * const gps_packet) {
    gps_packet->type = GPS_t;
    gps_packet->satellites_computed = gps_pos->satellitesUsed;
    gps_packet->satellites_tracked = gps_pos->satellitesTracked;
    gps_packet->latitude = gps_pos->latitude;
    gps_packet->longitude = gps_pos->longitude;
    gps_packet->lat_std_dev = gps_pos->latitudeStdDev;
    gps_packet->long_std_dev = gps_pos->longitudeStdDev;
    gps_packet->solution_status = gps_pos->solStat;
    gps_packet->position_type = gps_pos->posType;
    gps_packet->solution_age = gps_pos->solutionAge;
    gps_packet->differential_age = gps_pos->differentialAge;
}

// Reads the GPS and stores the values 
void _ReadPosPacket(GPS_Packet_Header * header, Sensor * sensor,const SensorVariable state)
{
	GPS_Packet_BestPos * bestPosPt = (GPS_Packet_BestPos*)&header->dataStart;
	GPS_Packet_BestPos bestPos;
	memcpy(&bestPos,bestPosPt,sizeof(GPS_Packet_BestPos));
	bestPos.longitude = _Convert64Double(bestPos.longitude);
	bestPos.latitude = _Convert64Double(bestPos.latitude);
	bestPos.latitudeStdDev = _Convert32Float(bestPos.latitudeStdDev);
	bestPos.longitudeStdDev  = _Convert32Float(bestPos.longitudeStdDev);
	bestPos.solStat = _Convert32Int(bestPos.solStat);
	bestPos.posType = _Convert32Int(bestPos.posType);
	bestPos.differentialAge = _Convert32Float(bestPos.differentialAge);
    bestPos.solutionAge = _Convert32Float(bestPos.solutionAge);
	LOG.GPS("LAT %Lf   LONG %Lf",bestPos.latitude,bestPos.longitude);
	LOG.GPS("Satellites Used = %d ",bestPos.satellitesUsed);
	LOG.GPS("Satellites Tracked = %d ",bestPos.satellitesTracked);
        GPS_MakeGPSPacket(&bestPos, &gpsToBroadcast);
        gps_is_new = TRUE;
        if (GPS_networkOutputOnly) 
        {
            sensor->hasBeenUpdated = FALSE;
        } 
        else 
        {
            // Check for this 0 case due to funky ness
            if(bestPos.solStat == sol_status_computed) 
            {
                    LOG.GPS("Solution status computed");
                    if(bestPos.posType == pos_type_single || bestPos.posType == pos_type_omnistar || bestPos.posType == pos_type_omnistar_hp) 
                    {
                            LOG.GPS("Position type was single, omnistar or omnistarHP: %u", bestPos.posType);
                            if(bestPos.satellitesUsed<12 && bestPos.satellitesUsed > 0)
                            {
                                    LOG.GPS("Make satellites used PASS");
                                    if(bestPos.latitude<GPS_latHighLimit && bestPos.latitude > GPS_latLowLimit)
                                    {
                                            LOG.GPS("LAtitude LIMIT  PASS");
                                            if(bestPos.longitude < GPS_longHighLimit && bestPos.longitude > GPS_longLowLimit)
                                            {
                                                    LOG.GPS("LONGITDUE LIMIT PASS");
                                                    float small = 0.0001;
                                                    if(bestPos.longitudeStdDev > small && bestPos.latitudeStdDev > small)
                                                    {
                                                            LOG.GPS("Std are not equal to 0 PASS");
                                                            // Set up the matrix
                                                            sensor->H = Matrix_Create(2,6);
                                                            sensor->H = Matrix_SetValue(sensor->H,1,1,1);
                                                            sensor->H = Matrix_SetValue(sensor->H,2,2,1);

                                                            matrix Variance = Matrix_Create(2,2);
                                                            sensor->state->Variance=Variance;

                                                            matrix Location = Matrix_Create(2,1);
                                                            sensor->state->State = Location;

                                                            if(bestPos.differentialAge > 10.0)
                                                            {
                                                                    Variance = Matrix_SetValue(Variance,1,1,bestPos.latitudeStdDev * bestPos.latitudeStdDev + 4);
                                                                    Variance = Matrix_SetValue(Variance,2,2,bestPos.longitudeStdDev * bestPos.longitudeStdDev + 4);
                                                            }
                                                            else
                                                            {
                                                                    Variance = Matrix_SetValue(Variance,1,1,bestPos.latitudeStdDev * bestPos.latitudeStdDev + 2.);
                                                                    Variance = Matrix_SetValue(Variance,2,2,bestPos.longitudeStdDev * bestPos.longitudeStdDev + 2.);
                                                            }
                                                            sensor->state->Variance=Variance;

                                                            double newLat = (bestPos.latitude - GPS_latOffset) * GPS_LatitudeRatio;
                                                            double newLong = (bestPos.longitude-GPS_longOffset) * GPS_LongitudeRatio;

                                                            // Stores the longitude and the latitude readings
                                                            sensor->state->State = Matrix_SetValue(sensor->state->State,1,1,(float)newLat - Matrix_GetX(state.State));
                                                            sensor->state->State = Matrix_SetValue(sensor->state->State,2,1,(float)newLong - Matrix_GetY(state.State));

                                                            LOG.GPS("GPS meter in x  = %10.20lf",newLat);
                                                            LOG.GPS("GPS meter in y  = %10.20lf",newLong);
                                                            LOG.GPS("GPS meter in x stdDev  = %f",bestPos.latitudeStdDev);
                                                            LOG.GPS("GPS meter in y stdDev  = %f",bestPos.longitudeStdDev);

                                                            LOG.DATA("GPS meter in x  = %10.20lf",newLat);
                                                            LOG.DATA("GPS meter in y  = %10.20lf",newLong);
                                                            LOG.DATA("GPS meter in x stdDev  = %f",bestPos.latitudeStdDev);
                                                            LOG.DATA("GPS meter in y stdDev  = %f",bestPos.longitudeStdDev);

                                                            sensor->hasBeenUpdated = 1;

                                                            LOG.GPS("GPS used");
                                                            return;
                                                    }
                                            }
                                    }
                            }
                            else
                            {
                                LOG.GPS("UsedSatelites Failed");
                            }
                    }
                    else 
                    {
                        LOG.GPS("Position type was no good: %u", bestPos.posType);
                    }
            }
            else 
            {
                LOG.GPS("Solution Status not 'Computed'. It was: %u", bestPos.solStat);
            }
        }
}

// Reads the GPS value if there is new data
void GPS_Update(Sensor * sensor, const SensorVariable state)
{
	LOG.GPS("Starting GPS Sensor Update");

	if(Microprocessor->isGPSNew()==0)
	{
		LOG.GPS("No new GPS data");
		return;
	}

	uint32_t length = Microprocessor->getGPSArrayLength();
	Microprocessor->getGPSArrayData(&GPS_data[0],length);
	GPS_Packet_Header * header = (GPS_Packet_Header *) &GPS_data;
    uint16_t message_type = _Convert16Int(header->messageId);
	LOG.GPS("PRINT type %d", message_type);
	/*
	 * Check for bestpos packet
	 */
	if(message_type == 42 || message_type == 47)
	{
			LOG.GPS("BESTPOS or PSEUDORANGE Packet found");
			_ReadPosPacket(header,sensor,state);
	}
}

// Initialize the GPS sensor
void GPS_Boot(Sensor * sensor,SensorVariable * start)
{
	if(fileExists("config/GPS.ini"))
	{
		dictionary* config = iniparser_load("config/GPS.ini");
		GPS_LatitudeRatio = iniparser_getdouble(config,"Conversion:Latitude",0);
		GPS_LongitudeRatio= iniparser_getdouble(config,"Conversion:Longitude",0);
		GPS_longLowLimit= iniparser_getdouble(config,"Limit:LowLongitude",0);
		GPS_longHighLimit= iniparser_getdouble(config,"Limit:HighLongitude",360);
		GPS_latLowLimit= iniparser_getdouble(config,"Limit:LowLatitude",0);
		GPS_latHighLimit= iniparser_getdouble(config,"Limit:HighLatitude",360);
		GPS_angleDtLength = iniparser_getdouble(config,"Angle:DtLength",1.0);
		GPS_angleVarianceMult = iniparser_getdouble(config,"Angle:VarianceMultiplyer",1);
		GPS_latOffset = iniparser_getdouble(config,"Offset:Latitude",1.0);
		GPS_longOffset = iniparser_getdouble(config,"Offset:Longitude",1);
		GPS_velocityMin = iniparser_getdouble(config,"Limit:VelocityMin",.5);
		GPS_velcityBeliefFactor = iniparser_getdouble(config,"Conversion:VelocityBeliefFactor",.5);
        GPS_networkOutputOnly = iniparser_getboolean(config, "Output:NetworkOutputOnly", 0);
		iniparser_freedict(config);
		LOG.GPS("GPS Ini file loaded");
	}
	else
	{
		LOG.GPS("gps.ini was not found");
	}

	// CHAD: I am awsome DELETE ME 
	GPS_angleVarianceMult = 10;

	GPS_angleLastLat = 0.0;
	GPS_angleLastLong  = 0.0;
	GPS_anglePrevious = 0;

	int bootUpAttempt = 0;
	GPS_angleSum=10000;
    GPS_networkOutputOnly = 1;

    // TODO: figure out if this is required or not
	/*
	 * This may or may not be required!!!!!!
	 * It is required for initalization purposes:)
	 */
	/*
	StateVariable TestState;
	while(bootUpAttempt == 0)
	{
		GPS_Update(sensor,TestState);
		if(sensor->hasBeenUpdated)
		{
			WOOOOO Major HACK
			if(sensor->state->Location.columns==2)
			{
				Matrix_SetValue(start->Location,1,1,Matrix_GetValue(sensor->state->Location,1,1));
				Matrix_SetValue(start->Location,2,1,Matrix_GetValue(sensor->state->Location,2,1));
			}
			else
			{
				LOG.VERBOSE("Retrying to find gps corrdinates to get an inital offset.");
			}
		}
		else
		{
			LOG.VERBOSE("Retrying to find gps corrdinates to get an inital offset.");
		}
	}
	*/
}




