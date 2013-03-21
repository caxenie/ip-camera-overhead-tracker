/**
 * @author Cristian Axenie, cristian.axenie@tum.de
 * 
 * Simple tracking application using data streamed over TCP/IP
 * from Axis IP Cam (Robot room / Holodeck), local camera or locally saved file.
 * The tracking algorithm will be used for mobile robot tracking in indoor operation.
 * 
 * Tracker functionality declarations.
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
#include <highgui.h>
#include <cv.h>

#define MARKER_SIZE 			10	/* the object markers size */	
#define SEARCH_SPACE_SIZE   		65	/* search space window size for template matching */
#define MATCHING_THRESHOLD  		0.5	/* template matching threshold */	
#define CONVERSION_FACTOR_X		0.280	/* cam space to world space mapping value for X axis with lens distort */
#define CONVERSION_FACTOR_Y		0.290	/* cam space to world space mapping value for Y axis with lens distort */
#define MAXBUF				50 	/* max buffer length for data sending */

#define VERBOSE				/* get markers, trace and additional info */
// #define AUTO_FIND_MARKERS		/* detects markers automatically - not stable */

/* 
 * Tracker utilities 
 */
struct tracker{
	/* image header that will save the tracked positions of the object */
	IplImage *obj_pos_img;
	/* image headers that will contain templates and matching results */
	IplImage *main_marker_mask_img;
	IplImage *aux_marker_mask_img;
	IplImage *main_marker_matching_img;
	IplImage *aux_marker_matching_img;
	/* timing index */
	int idx;
};

/**
 * Tracked object properties used in tracker algorithm 
 */
struct tracked_object{
	/* current position of the main object marker in the frame */
	int x_pos;
	int y_pos;
	/* current position of the auxiliary marker in the frame */
	int x_pos_aux_mark;
	int y_pos_aux_mark;
	/* aux and main markers auxiliary pose vars */
	int mx;
	int my; 
	int ms;
	int tx;
	int ty; 
	int ts;
	/* flags to mark if the markers were detected in the frame */
	short main_is_on;
	short aux_is_on;
	/* flag to init global coordinates */
	short init;
	/* data buffer to store tracking history */
	char buffer[MAXBUF];
};

/* Tracker instance */
extern struct tracker *trk;
/* Tracked object properties */
extern struct tracked_object *obj;
/* font for GUI text  */
extern CvFont font;
/* timer utils */
extern struct timespec tstart, tcur;

#ifdef AUTO_FIND_MARKER
/**
 * Searches the main marker in the camera frame.
 * Algorithm uses a modified Hough transform 
 */
void search_main_marker();
/** 
 * Auxiliary markers searching. This marker is used for object angle computation. 
 * Marker is detected automatically
 */
void search_aux_marker();
#else
/**
 * Select markers with the mouse 
 */
void select_point(int event, int x_coord, int y_coord, int flags, void *param);
#endif
/**
 * Object markers setup used for heading angle computation
 */
void setup_trackers();
/**
 * Markers tracking functions using template matching 
 */
void track_aux_marker();
void track_main_marker();
/**
 * Compute real world pose of the object and pack data 
 * for file logging, remote sending or visualization
 */
void present_data();
