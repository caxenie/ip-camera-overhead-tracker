/**
 * @author Cristian Axenie, cristian.axenie@tum.de
 * 
 * Simple tracking application using data streamed over TCP/IP
 * from Axis IP Cam (Robot room / Holodeck), local camera or locally saved file.
 * The tracking algorithm will be used for mobile robot tracking in indoor operation.
 * 
 * Stream server declarations.
 */

#include <stdio.h>
#include <sys/types.h>
#include <sys/timeb.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <netdb.h>
#include <unistd.h>
#include <errno.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <poll.h>
#include <sys/fcntl.h>
#include <pthread.h> 
#include <string.h>
#include <poll.h>
#include <limits.h>
#include <semaphore.h> 
#include <stdlib.h>

#define PORT 		"56000"	/* port number exposed by the server */
#define BACKLOG 	20      /* how many pending connections queue will hold */

/**
 * Get client sockaddr for stream server 
 */
void *get_client_address(struct sockaddr *sa);
/*
 * Separate thread to accept socket connections
 * for sending the tracking data to a remote
 * machine
 */
void * remote_connections_handler(void *data);


