/**
 * @author Cristian Axenie, cristian.axenie@tum.de
 * 
 * Simple tracking application using data streamed over RTSP 
 * from Logilink IP Cameras,local camera or locally saved file.
 * The tracking algorithm will be used for mobile robot tracking in indoor operation.
 * 
 * Medium access utilities. 
 */
 
#ifndef UTILS_H_
#define UTILS_H_
 
#include <stdio.h>
#include "highgui.h"
#include "cv.h"

/* frame source type */
enum{
	REMOTE,
	LOCAL,
	DISK
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
/* file to store tracking data */
FILE *f;

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
	} else if( strstr(in[1],"rtsp")!=NULL && nr == 2 ) {
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


