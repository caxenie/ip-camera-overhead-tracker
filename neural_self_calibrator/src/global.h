#ifndef GLOBAL_H
#define GLOBAL_H

/* global header file to include all needed headers */

//C++ Headers
#include <iostream>
#include <sstream>
#include <string>
#include <fstream>

//Basic C Headers
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>

#include <signal.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netdb.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/fcntl.h>
#include <errno.h>
#include <semaphore.h>
#include <poll.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/time.h>
#include <sys/wait.h>
#include <math.h>
//#include <sys/timeb.h>


//Library Headers
#include <dc1394/dc1394.h>
#include <cv.h>
#include <highgui.h>
#include <fann.h>
#include <libconfig.h>


#include "data_types.h"

/* Frame Size */
#define FRAME_WIDTH 		780
#define FRAME_HEIGHT 		582

/* Default ROI size */
#define ROI_WIDTH 			24
#define ROI_HEIGHT 			24


#define PI 					3.141592654

/* Tracker states */
#define ON_TRACK 			1
#define OFF_TRACK 			0


//#define TARGET_DISTANCE 0.133
#define TARGET_DISTANCE 0.1562


/** 
 * The marker position has to be accessed by the tracker (updating pixel coordinates),
 * the reprojection (updating world coordinates) thread and the streaming thread (send the buffer content to client),
 * thus access needs to get locked with mutex.
**/
extern struct _MarkerPosition *marker_position;
extern pthread_mutex_t mutex;

/* signal flag to safely exit all threads */
extern int exit_state;


extern int num_of_trackers_on_track;
extern int num_of_trackers_coord_set;
extern int *cams_on_track;

#endif
