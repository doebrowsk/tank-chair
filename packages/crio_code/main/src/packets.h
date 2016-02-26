/*
 * File:   packets.h
 * Author: Eric Perko
 *
 * Created on August 31, 2009, 12:36 PM
 */

#ifndef _PACKETS_H
#define	_PACKETS_H

#include <vxWorks.h>
#include "vector_math.h"
#include "hardwarediagnostics.h"

#ifdef	__cplusplus
extern "C" {
#endif

	#define BUF_SIZE 1024

	/*
	 * These types are defined to numbers that match the type code in the packets
	 * themselves. This is why they are defined to numbers
	 *
	 * TODO switch these to enums
	 */
	#define HEADINGSPEEDCOMMAND_t 1
	#define REBOOTCOMMAND_t 0
	#define WAYPOINTCOMMAND_t 2
	#define STARTCOMMAND_t 3
	#define STOPCOMMAND_t 4
	#define QUERYSYSTEMSTATUS_t 5
	#define ESTOP_t 6
	#define ANGULARRATESPEEDCOMMAND_t 7

	#define POSE_t 0
	#define DIAGNOSTICS_t 1
	#define SONARS_t 2
        #define GPS_t 3
        #define COMPASS_t 4

	typedef struct poseToBroadcast {
		int8_t type;
		int8_t data1; /* padding */
		int8_t data2;
		int8_t data3;
		float x;
		float y;
		float theta;
		float vel;
		float omega;
                float yaw_bias;
		float x_variance;
		float y_variance;
		float theta_variance;
		float vel_variance;
		float omega_variance;
                float yaw_bias_variance;
		float sonar_ping_1;
		float sonar_ping_2;
		float sonar_ping_3;
		float sonar_ping_4;
		float sonar_ping_5;
	} Pose;

	typedef struct sonarToBroadcast {
		int8_t type;
		int8_t data1; /* padding */
		int8_t data2;
		int8_t data3;
		float front_sonar;
	} Sonars;

	typedef struct diagnosticsPacket_t {
		int8_t type;
		int8_t status;
        uint8_t eStopTriggered;
        uint8_t RCOn;
        int16_t FPGAVersion;
        int16_t VMonitor_cRIO_mV;

        int32_t LWheelTicks;
        int32_t RWheelTicks;
        int32_t LMotorTicks;
        int32_t RMotorTicks;

        int16_t VMonitor_24V_mV;
        int16_t VMonitor_13V_mv;
        int16_t VMonitor_5V_mV;
        int16_t VMonitor_eStop_mV;

        int16_t YawRate_mV;
        int16_t YawSwing_mV;
        int16_t YawTemp_mV;
        int16_t YawRef_mV;
        
        uint16_t C1Steering;
        uint16_t C2Throttle;
        uint16_t C3Mode;
        uint8_t RCeStop;
    } DiagnosticsPacket;

    typedef struct gpsNetworkingPacket_t {
        int8_t type;
        int8_t data1; /* padding */
        uint8_t satellites_computed;
        uint8_t satellites_tracked;
        double latitude;
        double longitude;
        float lat_std_dev;
        float long_std_dev;
        uint32_t solution_status;
        uint32_t position_type;
        float differential_age;
        float solution_age;
    } GPSNetworkingPacket;

	typedef struct Command_t {
		int8_t type;
		char commandData[BUF_SIZE - sizeof(int8_t)];
	} Command;

	typedef struct HeadingSpeedCommand_t {
		int8_t type;
		int8_t data1; /* the following are for padding purposes */
		int8_t data2;
		int8_t data3;
		float desiredHeading;
		float desiredSpeed;
	} HeadingSpeedCommand;

	typedef struct AngularRateSpeedCommand_t {
		int8_t type;
		int8_t data1; /* the following are for padding purposes */
		int8_t data2;
		int8_t data3;
		float desiredAngularRate;
		float desiredSpeed;
	} AngularRateSpeedCommand;

	typedef struct WaypointCommand_t {
		int8_t type;
		int8_t data1; /* the following are for padding purposes */
		int8_t data2;
		int8_t data3;
		Point2d p;
		float heading;
		float max_speed;
	} WaypointCommand;

	typedef struct QuerySystemStatus_t {
		int8_t type;
		int8_t verbose;
	} QuerySystemStatus;

	typedef struct StopCommand_t {
		int8_t type;
		int8_t force;
	} StopCommand;

#ifdef	__cplusplus
}
#endif
#endif	/* _PACKETS_H */

