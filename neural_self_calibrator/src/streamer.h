/**
 * @author Cristian Axenie, cristian.axenie@tum.de
 *
 * Simple tracking application using data streamed over TCP/IP
 * from Axis IP Cam (Robot room / Holodeck), local camera or locally saved file.
 * The tracking algorithm will be used for mobile robot tracking in indoor operation.
 *
 * Stream server declarations.
 */

#include "global.h"


/*
 * Separate thread to accept socket connections
 * for sending the tracking data to a remote
 * machine
 */
void * remote_connections_handler(void *arg);


void error(const char *msg);


