/**
 * @author Cristian Axenie, cristian.axenie@tum.de
 * 
 * Simple tracking application using data streamed over TCP/IP
 * from Axis IP Cam (Robot room / Holodeck), local camera or locally saved file.
 * The tracking algorithm will be used for mobile robot tracking in indoor operation.
 * 
 * Overhead tracker main program.
 */

#include "tracking.h"
#include "streamer.h"
#include "utils.h"

/* Tracker instance */
struct tracker *trk;
/* Tracked object properties */
struct tracked_object *obj;
/* font for GUI text  */
CvFont font;
/* Stream access instance */
struct frame_accessor *frame_provider;
/* Data log */
struct data_log *log_bin;
/* sync lock with streamer thread */
short send_on;
/* client connected flag */
short client_on;
/* timer utils */
struct timespec tstart, tcur;

/* window name */
const char* win_name="Overhead tracker";

/* init application */
void init_application(){
	/* init log data support */
	log_bin = (struct data_log*)calloc(MAX_LOG_SIZE, sizeof(struct data_log));
	/* init frame source */
	frame_provider = (struct frame_accessor*)calloc(1, sizeof(struct frame_accessor));
	/* init tracker */
	trk = (struct tracker *)calloc(1, sizeof(struct tracker));
	/* init tracked object properties */
	obj = (struct tracked_object *)calloc(1, sizeof(struct tracked_object));
}

/* entry point */
int main(int argc, char* argv[]){	
	
#ifdef STREAMER_ON
	/* separate thread to handle the incomming connections */
	pthread_t conn_handler;
	/* ret code when creating thread */
	int rc;
#endif
	/* one time init switch for markers search and setup */
	int on_init = 0;

	/* init font system for coordinate display */
	cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.7, 0.7, 0.5, 1, 8);
	/* create window as placeholder for captured frames and tracking */
	cvNamedWindow(win_name, CV_WINDOW_AUTOSIZE);
	
	/* get initial time for timestamp */
	if(clock_gettime(CLOCK_REALTIME, &tstart)==-1){
		printf("main: Cannot access time subsystem\n");
		exit(EXIT_FAILURE);
	}				  

	/* init app by initializing all structs instances */
	init_application();
	/* init streamer enable flag and client flag */
	client_on = 0;
	send_on = 0;

	/* check the frames source */
	if ((frame_provider->capture = get_source(frame_provider->capture, argc, argv))==NULL) {
		printf("main: Cannot open stream\n");
		return (EXIT_FAILURE);
	}
	
	/* get an initial frame to get stream properties */
	frame_provider->image = cvQueryFrame(frame_provider->capture);
	
	/* create masking images for the object markers */
	trk->main_marker_mask_img = cvCreateImage(cvSize(MARKER_SIZE, MARKER_SIZE),
					    frame_provider->image->depth,
					    frame_provider->image->nChannels);
								   
	trk->aux_marker_mask_img = cvCreateImage(cvSize(MARKER_SIZE, MARKER_SIZE),
					    frame_provider->image->depth,
					    frame_provider->image->nChannels);
								   
	/* create images to store matching results for marker tracking */
	trk->main_marker_matching_img = cvCreateImage(cvSize(SEARCH_SPACE_SIZE - MARKER_SIZE +1,
						 SEARCH_SPACE_SIZE - MARKER_SIZE +1),
						 IPL_DEPTH_32F,
						 1);
   	trk->aux_marker_matching_img = cvCreateImage(cvSize(SEARCH_SPACE_SIZE - MARKER_SIZE +1,
						SEARCH_SPACE_SIZE - MARKER_SIZE +1),
						IPL_DEPTH_32F,
						1);

	/* set callback for the template selector */
        cvSetMouseCallback(win_name, select_point, NULL);

	/* check if recording is requested is requested and init recorder */
	if(frame_provider->is_recording==1) record_stream_init();

#ifdef STREAMER_ON
	/* start remote log data sending thread  */
	if ((rc = pthread_create(&conn_handler, NULL, remote_connections_handler, NULL))) {
   		printf("main: pthread_create, rc: %d\n", rc);
        }
#endif

	/* loop until no frames are received or user decides */
	while(1) {
		/* check if recording is requested */
		if(frame_provider->is_recording==1){
				/* start recording frames */
				record_stream();
				/* update images */
				cvShowImage(win_name, frame_provider->image);
		}
		else{
			/* process only if we receive frames from the cam */
			if((frame_provider->image = cvQueryFrame(frame_provider->capture))!=NULL){		
#ifdef STREAMER_ON			
			/* if it is sending go on */
			if(send_on==1) continue;
#endif
			/* initialize image with tracked positions to be used in visualization */
			if(trk->obj_pos_img == NULL) {
				trk->obj_pos_img = cvCreateImage(cvGetSize(frame_provider->image), IPL_DEPTH_32F, 3);
			}

   			/* tracking is enabled only for remote cam and recorded file */
			if(frame_provider->source!=LOCAL){
			/* search markers and setup once */
			if(on_init==0){
#ifdef AUTO_FIND_MARKER
				  if(obj->main_is_on == 0){
						search_main_marker();
				  }
				  else{
					  if(obj->aux_is_on == 0){
						search_aux_marker();
					  }
				  }
				 if(obj->main_is_on == 1 && obj->aux_is_on == 1){
					setup_trackers();
					on_init = 1;
				  }
#else
				if(obj->main_is_on == 1){
				   if(obj->aux_is_on == 1){
					setup_trackers();
					on_init=1;	
				   }
				}
#endif
				}
				else{
					/* if already detected track the markers */
					track_main_marker();
					track_aux_marker();
					/* write to log file and redisplay debug output */
					present_data();					
#ifdef STREAMER_ON	
					/* send data only if a client is connected */
					if(client_on==1)
						send_on = 1;
#endif
				}
			}
			/* update images */
			cvShowImage(win_name, frame_provider->image);
			}
			else break;
		}
		/* check if end stream is sent */
		if((int)(cvWaitKey(2) & 255) == 'q') {
			printf("Exiting tracker ...\n");
			if(client_on==0) goto out;
			else break;			
			/* if the recording was on we just free resources and exit */
			if(frame_provider->is_recording == 1){
				/* free the allocated memory*/
				if(frame_provider->capture)
					cvReleaseCapture(&frame_provider->capture);
				/* close recorder */
				record_stream_close();
				/* destroy window handlers */
				cvDestroyWindow(win_name);
				return EXIT_SUCCESS;
			}
			break; 
		}
	}

#ifdef STREAMER_ON
	/* wait for the stream server */
	pthread_join(conn_handler, NULL);
#endif	

out:
	/* dump log data */
	if(dump_log_file(log_bin, trk->idx)!=0){
		printf("Cannot dump file, restart experiment\n");
	}

	/* free the allocated memory*/
	if(frame_provider->capture)
		cvReleaseCapture(&frame_provider->capture);

	/* close recorder */
	record_stream_close();

	/* destroy window handlers */
	cvDestroyWindow(win_name);

	return EXIT_SUCCESS;
}
