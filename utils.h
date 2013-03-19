/**
 * @author Cristian Axenie, cristian.axenie@tum.de
 * 
 * Simple tracking application using data streamed over RTSP 
 * from Axis IP Cam (Robot room / Holodeck), local camera or locally saved file.
 * The tracking algorithm will be used for mobile robot tracking in indoor operation.
 * 
 * Medium access utilities. 
 */
 
#ifndef UTILS_H_
#define UTILS_H_
 
#include <stdio.h>
#include <time.h>
#include "highgui.h"
#include "cv.h"

/* frame source type */
enum{
	REMOTE,
	LOCAL,
	DISK
};

/* robot data to be dumped on disk */
struct robot_data_log{
	double xpos;
	double ypos;
	double heading;
	int sample;
	double timestamp;
};

/* source type var */
int source;
/* the video capturing struct init */
CvCapture *capture = NULL;
/* current frame from stream */
IplImage* image = NULL;
/* frame recorder */
CvVideoWriter *recorder;
/* flag to mark recording */
short is_recording = 0;
/* data log */
struct robot_data_log *log_file;
/* timer utils */
struct timespec tstart, tcur;

/** 
 * Initialize the stream recorder
 */
void record_stream_init(){
	/* recording parameters */
	int isColor = 1;
	int fps = 30;  
	int frame_width  = image->width; 
	int frame_height  = image->height;
	/* setup recorder */
	recorder = cvCreateVideoWriter( "CAM-TRACK_Rec.avi",				/* file to save into */
									CV_FOURCC('D', 'I', 'V', 'X'),		/* codec to be used: MPEG-4 */
									fps,								/* FPS */
									cvSize(frame_width,frame_height),	/* size */
									isColor);							/* we get RGB color */
}

/** 
 * Initialize the stream recorder.
 * Record the incoming frames from the source and 
 * encode them into an MPEG file on the disk
 */
void record_stream(){
	/* connect to capture */
	cvGrabFrame(capture);          
	/* retrieve a frame */
	image = cvRetrieveFrame(capture);  
	/* add the frame to the file */
	cvWriteFrame(recorder,image);      
}

/** 
 * Close the stream recorder
 */
void record_stream_close(){
	if(recorder)
		cvReleaseVideoWriter(&recorder);
}

/**
* Get the stream source local saved file, local cam or remote stream
*/
CvCapture* get_source(CvCapture *c, int nr, char** in)
{
	/* check if preallocated */
	if(c!=NULL) c = NULL;
	
	/* check if recording is on and init recorder */
	for (int i = 1; i < nr; i++)  /* Skip argv[0] (program name). */
    {
		if(strstr(in[i],"-r")!=NULL){
			/* set the flag */
			is_recording = 1;
			/* to get the proper source subtract the record flag */
			nr--;
		}
	}
	
	/* check source */
	if((nr == 1) || 
	   (nr == 2 && strlen(in[1])==1 && isdigit(in[1][0]))){
		printf("get_source: capture from local cam\n");
		/* capture from local cam */
		c = cvCaptureFromCAM( nr==2 ? in[1][0] - '0' : 0);
		source = LOCAL;
	} else if( strstr(in[1],"http")!=NULL && nr == 2 ) {
		/* capture from remote cam */
		printf("get_source: capture from remote cam\n");
		c = cvCreateFileCapture(in[1]);
		source = REMOTE;
	} else if( nr == 2 ) {
		printf("get_source: capture from locally saved file\n");
		/* capture from local sved file */
		c = cvCaptureFromAVI( in[1] );
		source = DISK;
	}
	return c;
}

/**
* Get information about the captured stream
* FIXME: some problems with getting FPS and 4CC on V4L2
*/
int* get_stream_properties(CvCapture *c)
{
	/* Allocate and init property vector */
	int* stream_prop = (int*)calloc(4, sizeof(int));
	/* get the height, width and FPS from the captured stream data */
	stream_prop[0] = (int)cvGetCaptureProperty(c, 		/* capture */
	                 CV_CAP_PROP_FRAME_HEIGHT); 		/* frame height */
	stream_prop[1] = (int)cvGetCaptureProperty(c, 		/* capture */
	                 CV_CAP_PROP_FRAME_WIDTH); 			/* frame width */
	/* FIXME on Linux V4L2 seem to have a problem with FPS and 4CC info */
#if defined (__WIN32__)
	stream_prop[2] = (int)cvGetCaptureProperty(c, 			    /* capture */
	                 CV_CAP_PROP_FPS)	;						/* FPS */
	stream_prop[3] = (int)cvGetCaptureProperty(c, 			    /* capture */
	                 CV_CAP_PROP_FOURCC);				        /* 4-character code of codec */
	printf("get_stream_properties: WxH CODEC @ FPS: %d x %d %d @ %d\n",stream_prop[1], stream_prop[0], stream_prop[3], stream_prop[2]);
#else
	printf("get_stream_properties: WxH %d x %d\n", stream_prop[1], stream_prop[0]);
#endif
	return stream_prop;
}

#endif

/* dumps the memory saved log file to the disk */
int dump_log_file(struct robot_data_log* buffer, int buffer_size)
{
    int i;
    time_t rawtime;
    struct tm * timeinfo;
    char log_file_name [160];
    FILE *f;

    time ( &rawtime );
    timeinfo = localtime ( &rawtime );

    strftime (log_file_name,80,"%Y-%m-%d__%H:%M:%S",timeinfo);
    strcat(log_file_name,"_overhead_tracker_position" );

    f = fopen(log_file_name, "w+");

    if(f==NULL){
        fprintf(stderr, "Cannot open log file!\n");
        return 1;
    }
    for(i=0;i<buffer_size;i++){
        fprintf(f, "%lf,%lf,%lf,%d,%lf\n", buffer[i].xpos, buffer[i].ypos, buffer[i].heading, buffer[i].sample, buffer[i].timestamp);
    }
    fclose(f);
    return 0;
}

/* compute the time interval for the timestamp in ms */
double compute_dt(struct timespec *end_time, struct timespec *start_time)
{
  struct timespec difference;

  difference.tv_sec =end_time->tv_sec -start_time->tv_sec ;
  difference.tv_nsec=end_time->tv_nsec-start_time->tv_nsec;

  while(difference.tv_nsec<0)
  {
    difference.tv_nsec+=1000000000;
    difference.tv_sec -=1;
  }

  return ((double)(1000.0*(double)difference.tv_sec+
                   (double)difference.tv_nsec/1000000.0));

}

