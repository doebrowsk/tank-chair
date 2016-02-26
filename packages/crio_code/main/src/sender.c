#define DEFAULT_SEND_PORT 50000
#define BROADCAST_ADDR "255.255.255.255"
/* TODO: This port number and broadcast address should be moved to a config file later... */
/* TODO: Document this file */

#include <vxWorks.h>
#include <net/inet.h>
#include <sockLib.h>
#include <inetLib.h>
#include <stdioLib.h>
#include <strLib.h>
#include <hostLib.h>
#include <ioLib.h>
#include <taskLib.h>
#include <eventLib.h>
#include "sender.h"
#include "MainCRIO.h"
#include "harlielog.h"
#include "sensorfusion.h"

size_t socketAddressSize;
struct sockaddr_in broadcast;
int sendPSOsocketFD;

STATUS closeSocket(int socketFd);
int getSendingSocket();

/**
 * Initialize the socket for sending packets.
 */
STATUS initSenders() {
    socketAddressSize = sizeof(struct sockaddr_in);
    bzero((char *) &broadcast, socketAddressSize);
    broadcast.sin_len = (u_char) socketAddressSize;
    broadcast.sin_family = AF_INET;
    broadcast.sin_port = htons(DEFAULT_SEND_PORT);
    broadcast.sin_addr.s_addr = inet_addr(BROADCAST_ADDR);
    sendPSOsocketFD = getSendingSocket();
    gps_is_new = FALSE;
    return(OK);
}

/**
 * Spwans a new task to send data through the socket connection
 */
STATUS startSenders() {
    int task = taskSpawn("PSOPacketSender", 90, VX_FP_TASK, 0xF000, (FUNCPTR) &sendPSOPacket, 0,0,0,0,0,0,0,0,0,0);
    if(task == ERROR) {
        LOG.ERR("Error spawning PSOPacketSender task");
        return(ERROR);
    } else {
        PSOSenderTask = task;
    }
    return(OK);
}

/**
 * Returns the tocketFD? 
 */
int getSendingSocket() {
    int socketFd;
    socketFd = socket(AF_INET, SOCK_DGRAM, 0);
    if (ERROR == socketFd) {
        LOG.ERR("Error creating sending socket");
        return(ERROR);
    } else {
        int sockOpts = 1;
        if (setsockopt(socketFd, SOL_SOCKET, SO_BROADCAST, (char *) &sockOpts, sizeof(sockOpts)) == ERROR) {
            LOG.ERR("Error giving socket broadcast permissions");
            return(ERROR);
        } else {
            return socketFd;
        }
    }
}

/**
 * Closes the socket connection
 */
STATUS closeSocket(int socketFd) {
    if (close(socketFd) == ERROR) {
        LOG.ERR("Error closing socket.");
        return(ERROR);
    }
    else {
        return(OK);
    }
}

/**
 * Sends everything back 
 */
STATUS sendPSOPacket() {
/* Old stuff... just here as an example of setting the address IF WE WERE NOT using the broadcast address

    if (((broadcast.sin_addr.s_addr = inet_addr(sendToAddress)) == ERROR) &&
            ((broadcast.sin_addr.s_addr = hostGetByName(sendToAddress)) == ERROR)) {
        LOG.ERR("Error getting the address of to send to.");
        return(ERROR);
    }
*/
    while(1) {
        UINT32 events;
        if (eventReceive(PSO_PACKET_READY, EVENTS_WAIT_ALL, WAIT_FOREVER, &events) == OK) {
            Pose packet = poseToBroadcast;
            packet.type = POSE_t;
            packet.theta = htonl(packet.theta);
            packet.x = htonl(packet.x);
            packet.y = htonl(packet.y);
            packet.omega = htonl(packet.omega);
            packet.vel = htonl(packet.vel);
            packet.yaw_bias = htonl(packet.yaw_bias);
            packet.x_variance = htonl(packet.x_variance);
            packet.y_variance = htonl(packet.y_variance);
            packet.theta_variance = htonl(packet.theta_variance);
            packet.vel_variance = htonl(packet.vel_variance);
            packet.omega_variance = htonl(packet.omega_variance);
            packet.yaw_bias_variance = htonl(packet.yaw_bias_variance);
            packet.sonar_ping_1 = htonl(packet.sonar_ping_1);
            packet.sonar_ping_2 = htonl(packet.sonar_ping_2);
            packet.sonar_ping_3 = htonl(packet.sonar_ping_3);
            packet.sonar_ping_4 = htonl(packet.sonar_ping_4);
            packet.sonar_ping_5 = htonl(packet.sonar_ping_5);
            if (PSOTask == -1) {
                LOG.ERR("Should not send when PSO isn't running... Variables may be in weird states");
            }
            else if(sendto(sendPSOsocketFD, (caddr_t) &packet, sizeof(Pose), 0,(struct sockaddr*) &broadcast, socketAddressSize) == ERROR) {
                LOG.ERR("Error while attempting to send a packet.");
            }
            sendDiagnosticsPacket();
            if( gps_enabled == TRUE ) {
                sendGPSPacket();
            }
        } else {
            LOG.ERR("Error receiving event in sendPSOPacket loop");
        }
    }
    return(OK);
}

/**
 * Sends all the diagnostics values back
 */
STATUS sendDiagnosticsPacket() {
    DiagnosticsPacket packet;
    packet.type = DIAGNOSTICS_t;
    packet.status = OK;
    packet.eStopTriggered = Microprocessor->getESTOPTriggered();
    packet.FPGAVersion = Microprocessor->getFPGAVersion();
    packet.LWheelTicks = Microprocessor->getLeftWheelTicks();
    packet.RWheelTicks = Microprocessor->getRightWheelTicks();
    packet.LMotorTicks = Microprocessor->getLeftMotorTicks();
    packet.RMotorTicks = Microprocessor->getRightMotorTicks();
    packet.VMonitor_cRIO_mV = Microprocessor->getCRIOVLineRead();
    packet.VMonitor_eStop_mV = Microprocessor->getESTOPVLineRead();
    packet.VMonitor_5V_mV = Microprocessor->get5VLineRead();
    packet.VMonitor_13V_mv = Microprocessor->get13VLineRead();
    packet.VMonitor_24V_mV = Microprocessor->get24VLineRead();
    packet.C1Steering = Microprocessor->getRCCH1();
    packet.C2Throttle = Microprocessor->getRCCH2();
    packet.C3Mode = Microprocessor->getRCCH3();
    packet.RCOn = Microprocessor->getRCOn();
    packet.RCeStop = Microprocessor->getRCESTOP();
    packet.YawRate_mV = Microprocessor->getYawRate();
    packet.YawSwing_mV = Microprocessor->getYawSwing();
    packet.YawTemp_mV = Microprocessor->getYawTemp();
    packet.YawRef_mV = Microprocessor->getYawRef();

    if(sendto(sendPSOsocketFD, (caddr_t) &packet, sizeof(DiagnosticsPacket), 0,(struct sockaddr*) &broadcast, socketAddressSize) == ERROR) {
        LOG.ERR("Error while attempting to send a diagnostics packet.");
        return (ERROR);
    }
    LOG.VERBOSE("Sent Diagnostics packet");
    return (OK);
}

/**
 * Sends the GPS data 
 */
STATUS sendGPSPacket() {
    if (gps_is_new) {
        GPSNetworkingPacket packet;
        packet = gpsToBroadcast;
        packet.type = GPS_t;

        if(sendto(sendPSOsocketFD, (caddr_t) &packet, sizeof(GPSNetworkingPacket), 0,(struct sockaddr*) &broadcast, socketAddressSize) == ERROR) {
            LOG.ERR("Error while attempting to send a GPS packet.");
            return (ERROR);
        }
        gps_is_new = FALSE;
        LOG.VERBOSE("Sent GPS packet");
    }
    return (OK);
}



