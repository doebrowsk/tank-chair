#define DEFAULT_COMMAND_RECEIVE_PORT 50001
/* TODO: This port number should be moved to a config file later... */
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
#include <rebootLib.h>
#include <sysLib.h>
#include <string.h>
#include <math.h>
#include <time.h>

#include "MainCRIO.h"
#include "receiver.h"
#include "packets.h"
#include "sharedVariables.h"
#include "harlielog.h"
#include "PSOmain.h"
#include "hardwarediagnostics.h"
#include "sender.h"

size_t socketAddressSize;
struct sockaddr_in receiverAddr;
int receiveSocketFd;

STATUS dispatchHeadingSpeedCommand(HeadingSpeedCommand *command);
STATUS dispatchWaypointCommand(WaypointCommand *command);
STATUS dispatchAngularRateSpeedCommand(AngularRateSpeedCommand *command);
STATUS dispatchQuerySystemStatus(QuerySystemStatus *new_command);
STATUS dispatchEstopCommand();

/**
 * Creates a socket connection to recieve commands. And sets estop cmd to false
 */
STATUS initReceivers() {
    command_should_estop = FALSE;
    receiveSocketFd = 0;
    socketAddressSize = sizeof(struct sockaddr_in);
    bzero((char *) &receiverAddr, socketAddressSize);
    receiverAddr.sin_family = AF_INET;
    receiverAddr.sin_len = (u_char) socketAddressSize;
    receiverAddr.sin_port = htons(DEFAULT_COMMAND_RECEIVE_PORT);
    receiverAddr.sin_addr.s_addr = htonl(INADDR_ANY);

    if((receiveSocketFd = socket(AF_INET, SOCK_DGRAM, 0)) == ERROR) {
        LOG.ERR("Error creating command receiver socket");
        return (ERROR);
    }
    if(bind(receiveSocketFd, (struct sockaddr *) &receiverAddr, socketAddressSize) == ERROR) {
        LOG.ERR("Error binding command receiver socket");
        close(receiveSocketFd);
        return (ERROR);
    }
    return (OK);
}

/**
 * Spawns a task to receive commands through the socket
 */
STATUS startReceivers() {
    int task = taskSpawn("CommandReceiverTask", 80, VX_FP_TASK, 0xF000, (FUNCPTR) &receiveCommandPacket, 0,0,0,0,0,0,0,0,0,0);
    if(task == ERROR) {
        LOG.ERR("Error spawning CommandReceiver task");
        return(ERROR);
    } else {
        CommandReceiverTask = task;
    }
    return(OK);
}

/**
 * Continuously looks for a command packet based on the size and it's value. Then passes the command packet on to the
 * correct function.
 */
STATUS receiveCommandPacket() {
    struct sockaddr_in senderAddr;
    int senderAddrSize = sizeof(struct sockaddr_in);
    Command c;
    while(1) {
        ssize_t bytes = recvfrom(receiveSocketFd, (char *) &c, BUF_SIZE, 0, (struct sockaddr *) &senderAddr, &senderAddrSize);
        /* I'm not super happy doing the parsing this way. We need to test using the Command struct c as the buffer, so that this can look a bit cleaner with just a few casts instead of all this crazy pointer stuff. */
       LOG.VERBOSE("We have recieved some data");
	   LOG.VERBOSE("Type = %d",c.type); 
		if (c.type == HEADINGSPEEDCOMMAND_t) {
            if(bytes == sizeof(HeadingSpeedCommand)) {
                /* The following is needed because the crio really sucks at floats that are not word aligned. */
                /* TODO: Reevaulate needing this alignment stuff now that there are padding bytes in the packet */
                /* TODO: Add commands received to be logged */
                dispatchHeadingSpeedCommand((HeadingSpeedCommand *) &c);
            } else {
                /* TODO: Log to the error log instead of printf'ing */
                LOG.ERR("Received unknown packet claiming to be a HeadingSpeedCommand. It was %d bytes, but the command is %d bytes\n", bytes, sizeof(HeadingSpeedCommand));
            }
        }
        else if (c.type == ANGULARRATESPEEDCOMMAND_t) {
            if(bytes == sizeof(AngularRateSpeedCommand)) {
                dispatchAngularRateSpeedCommand((AngularRateSpeedCommand *) &c);
            }
            else {
                LOG.ERR("Received unknown packet claiming to be a AngularRateSpeedCommand. It was %d bytes, but the command is %d bytes\n", bytes, sizeof(AngularRateSpeedCommand));
            }
        }
        else if (c.type == WAYPOINTCOMMAND_t) {
            if(bytes ==  sizeof(WaypointCommand)) {
                dispatchWaypointCommand((WaypointCommand *) &c);
            }
            else {
                LOG.ERR("Received unknown packet claiming to be a WaypointCommand. It was %d bytes, but the command is %d bytes\n", bytes, sizeof(WaypointCommand));
            }
        }
        else if (c.type == REBOOTCOMMAND_t) {
            LOG.INFO("Rebooting due to command");
            reboot(BOOT_NORMAL);
        }
        else if (c.type == STOPCOMMAND_t) {
            LOG.INFO("Stopping main loop due to command");
            stopMainLoop();
        }
        else if (c.type == STARTCOMMAND_t) {
            LOG.VERBOSE("Received start main loop command");
            if (PSOTask != -1) {
                LOG.ERR("Cannot start main loop while it is already running");
            }
            else {
                LOG.INFO("Starting main loop due to command");
                startMainLoop();
            }
        }
        else if (c.type == ESTOP_t) {
            LOG.INFO("ESTOPing by command");
            dispatchEstopCommand();
        }
        else if (c.type == QUERYSYSTEMSTATUS_t) {
            if (bytes == sizeof(QuerySystemStatus)) {
                LOG.VERBOSE("Received QuerySystemStatus packet");
                dispatchQuerySystemStatus((QuerySystemStatus *) &c);
            } else {
                LOG.ERR("Received unknown packet claiming to be a QuerySystemStatus. It was %d bytes, but the command is %d bytes\n", bytes, sizeof(QuerySystemStatus));
            }
        }
        else {
            LOG.ERR("Received unknown packet. Type %d",c.type);
        }
    }
    return(OK);
}

/**
 * sets the heading speed command value
 */
STATUS dispatchHeadingSpeedCommand(HeadingSpeedCommand *new_command) {
    /* For debugging we print the command received to the serial console. */
/*
    printf("command received: %f %f\n", command->desiredHeading, command->desiredSpeed);
*/
    LOG.VERBOSE("About to dispatch heading speed packet");
    steering_mode = MODE_VECTOR;
    heading_speed_commands = *new_command;
    return (OK);
}

/**
 * Sets the angular speed 
 */
STATUS dispatchAngularRateSpeedCommand(AngularRateSpeedCommand *new_command) {
    LOG.VERBOSE("About to dispatch angular rate command");
    steering_mode = MODE_TWIST;
    angular_rate_commands = *new_command;
    clock_gettime(CLOCK_REALTIME, &last_command_received);
    return (OK);
}

/**
 * Sets the estop
 */
STATUS dispatchEstopCommand() {
    LOG.VERBOSE("Dispatching EStop Command");
    command_should_estop = (command_should_estop == TRUE ? FALSE : TRUE);
    return (OK);
}

/**
 * Not implemented but should probably query the system
 */
STATUS dispatchQuerySystemStatus(QuerySystemStatus *new_command) {
/*
    LOG.VERBOSE("Running diagnostics");
    STATUS s = checkVoltages();

    DiagnosticsPacket p;
    p.status = s;

    sendDiagnosticsPacket(&p);
*/

    LOG.ERR("Not implemented");

    return (OK);
}

/**
 * TAkes in a waypoint for more path determined movement
 */
STATUS dispatchWaypointCommand(WaypointCommand *new_command) {

    LOG.VERBOSE("waypoint received: %f %f %f %f", new_command->p.x, new_command->p.y, new_command->heading, new_command->max_speed);

/*
     * TODO Put in logic for setting the last waypoint if we were not just in waypoint mode.
*/
    steering_mode = MODE_WAGON;
    if (new_command->p.x == waypoint.p.x && new_command->p.y == waypoint.p.y) {
/*
         * Do not advance the waypoints, or else we will not be able to draw the line between points...
*/
    } else {
        lastWaypoint = waypoint;
    }
    waypoint = *new_command;
    return (OK);
}
