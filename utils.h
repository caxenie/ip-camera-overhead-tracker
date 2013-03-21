/**
 * @author Cristian Axenie, cristian.axenie@tum.de
 * 
 * Simple tracking application using data streamed over TCP/IP
 * from Axis IP Cam (Robot room / Holodeck), local camera or locally saved file.
 * The tracking algorithm will be used for mobile robot tracking in indoor operation.
 * 
 * Medium access utilities declarations. 
 */
 
#include <stdio.h>
#include <time.h>
#include <ctype.h>
#include <highgui.h>
#include <cv.h>

#define MAX_LOG_SIZE 			9999    /* max data log size */

/* Frame source type */
enum{
	REMOTE,
	LOCAL,
	DISK
};

/* Data to be dumped on disk */
struct data_log{
	double xpos;
	double ypos;
	double heading;
	int sample;
	double timestamp;
};

/* Frames access */
struct frame_accessor{
	/* source type  */
	int source;
	/* the video capturing */
	CvCapture *capture;
	/* current frame from stream */
	IplImage *image;
	/* frame recorder */
	CvVideoWriter *recorder;
	/* flag to mark recording */
	short is_recording;
};

/* Stream access instance */
extern struct frame_accessor *frame_provider;

/* Data log */
extern struct data_log *log_bin;

/** 
 * Initialize the stream recorder
 */
int record_stream_init();
/** 
 * Initialize the stream recorder.
 * Record the incoming frames from the source and 
 * encode them into an MPEG file on the disk
 */
int record_stream();
/** 
 * Close the stream recorder
 */
void record_stream_close();
/**
* Get the stream source local saved file, local cam or remote stream
*/
CvCapture* get_source(CvCapture *c, int nr, char** in);
/**
* Get information about the captured stream
*/
int* get_stream_properties(CvCapture *c);
/**
 * Dumps the memory saved log file to the disk 
 */
int dump_log_file(struct data_log* buffer, int buffer_size);
/**
 * Compute the time interval for the timestamp in ms 
 */
double compute_dt(struct timespec *end_time, struct timespec *start_time);

