/**
 * @author Cristian Axenie, cristian.axenie@tum.de
 * 
 * Simple tracking application using data streamed over TCP/IP
 * from Axis IP Cam (Robot room / Holodeck), local camera or locally saved file.
 * The tracking algorithm will be used for mobile robot tracking in indoor operation.
 * 
 * Medium access utilities definitions. 
 */

#include "utils.h"

/** 
 * Initialize the stream recorder
 */
int record_stream_init(){
	/* recording parameters */
	int isColor = 1;
	int fps = 30;  
	int frame_width  = frame_provider->image->width; 
	int frame_height  = frame_provider->image->height;
	/* add timestamp to the file */
    	time_t rawtime;
    	struct tm * timeinfo;
    	char rec_file_name [200];
	/* get OS time info */
    	time ( &rawtime );
    	timeinfo = localtime ( &rawtime );
	/* convert to string pre/postfix */
    	strftime (rec_file_name,80,"%Y-%m-%d__%H:%M:%S",timeinfo);
    	strcat(rec_file_name,"_cam_tracker_rec.avi" );

	/* setup recorder */
	if((frame_provider->recorder = cvCreateVideoWriter(rec_file_name,			/* file to save into */
					   CV_FOURCC('D', 'I', 'V', 'X'),	/* codec to be used: MPEG-4 */
					   fps,					/* FPS */
					   cvSize(frame_width,frame_height),	/* size */
					   isColor))==NULL){			/* we get RGB color */
		return -1;
	}
	return 0;
}

/** 
 * Record the incoming frames from the source and 
 * encode them into an MPEG file on the disk
 */
int record_stream(){
	/* connect to capture */
	cvGrabFrame(frame_provider->capture);        
	/* retrieve a frame */
	if((frame_provider->image = cvRetrieveFrame(frame_provider->capture, 0))==NULL){
		printf("record_stream: Error retrieving frame.\n");
		return -1;
	}  
	/* add the frame to the file */
	cvWriteFrame(frame_provider->recorder,frame_provider->image);
	return 0;
}

/** 
 * Close the stream recorder
 */
void record_stream_close(){
	if(frame_provider->recorder)
		cvReleaseVideoWriter(&(frame_provider->recorder));
}

/**
* Get the stream source local saved file, local cam or remote stream
*/
CvCapture* get_source(CvCapture *c, int nr, char** in){
	/* check if preallocated */
	if(c!=NULL) c = NULL;
	
	/* check if recording is on and init recorder */
	for (int i = 1; i < nr; i++){  /* Skip argv[0] (program name). */
		if(strstr(in[i],"-r")!=NULL){
			/* set the flag */
			frame_provider->is_recording = 1;
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
		frame_provider->source = LOCAL;
	} else if( strstr(in[1],"http")!=NULL && nr == 2 ) {
		/* capture from remote cam */
		printf("get_source: capture from remote cam\n");
		c = cvCreateFileCapture(in[1]);
		frame_provider->source = REMOTE;
	} else if( nr == 2 ) {
		printf("get_source: capture from locally saved file\n");
		/* capture from local sved file */
		c = cvCaptureFromAVI( in[1] );
		frame_provider->source = DISK;
	}
	return c;
}

/**
* Get information about the captured stream
*/
int* get_stream_properties(CvCapture *c){
	/* Allocate and init property vector */
	int* stream_prop = (int*)calloc(2, sizeof(int));
	/* get the height, width and FPS from the captured stream data */
	stream_prop[0] = (int)cvGetCaptureProperty(c, 		/* capture */
	                 CV_CAP_PROP_FRAME_HEIGHT); 		/* frame height */
	stream_prop[1] = (int)cvGetCaptureProperty(c, 		/* capture */
	                 CV_CAP_PROP_FRAME_WIDTH); 			/* frame width */
	return stream_prop;
}

/**
 * Dumps the memory saved log file to the disk 
 */
int dump_log_file(struct data_log* buffer, int buffer_size){
    /* prefix with time information the dumped file */
    int i;
    time_t rawtime;
    struct tm * timeinfo;
    char log_file_name [200];
    FILE *f;
    /* get OS time */
    time ( &rawtime );
    timeinfo = localtime ( &rawtime );
    /* prepend */
    strftime (log_file_name, 80,"%Y-%m-%d__%H:%M:%S",timeinfo);
    strcat(log_file_name,"_overhead_tracker_position" );
    /* open file for writing */	
    if((f = fopen(log_file_name, "w+"))==NULL){
        printf("dump_log_file: Cannot open log file!\n");
        return -1;
    }
    /* write data */
    for(i=0;i<buffer_size;i++){
        fprintf(f, "%lf,%lf,%lf,%d,%lf\n", buffer[i].xpos, buffer[i].ypos, buffer[i].heading, buffer[i].sample, buffer[i].timestamp);
    }
    /* close file */
    fclose(f);
    return 0;
}

/**
 * Compute the time interval for the timestamp in ms 
 */
double compute_dt(struct timespec *end_time, struct timespec *start_time){
  /* difference holder */
  struct timespec difference;
  /* compute difference */
  difference.tv_sec =end_time->tv_sec -start_time->tv_sec ;
  difference.tv_nsec=end_time->tv_nsec-start_time->tv_nsec;
  
  while(difference.tv_nsec<0)
  {
    difference.tv_nsec+=1000000000;
    difference.tv_sec -=1;
  }

  return ((double)(1000.0*(double)difference.tv_sec+
                   (double)difference.tv_nsec/1000000.0)); /* ms */

}

