/**
 * @author Cristian Axenie, cristian.axenie@tum.de
 * 
 * Simple tracking application using data streamed over RTSP 
 * from Logilink IP Cameras,local camera or locally saved file.
 * The tracking algorithm will be used for mobile robot tracking in indoor operation.
 * 
 * Overhead tracker main program.
 */

#include "utils.h"
#include "tracking.h"

/* entry point */
int main(int argc, char* argv[]){											  
	/* separate thread to handle the incomming connections */
//	pthread_t conn_handler;
	/* ret code when creating thread */
	int rc;
	/* one time init switch for markers search and setup */
	int one_time = 0;
	
	/* open tracking logging file */
	f = fopen("CAM-TRACK_Log.txt", "w+");

/*	
	// start remote log data sending thread 
	if ((rc = pthread_create(&conn_handler, NULL, remote_connections_handler, NULL))) {
      printf("main: pthread_create, rc: %d\n", rc);
    }

	pthread_detach(conn_handler);
*/	
	/* init font system for coordinate display */
	cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.7, 0.7, 0.5, 1, 8);
	
	/* check the frames source */
	if ((capture = get_source(capture, argc, argv))==NULL) {
		printf("main: cannot open stream!\n");
		return 1;
	}
	
	/* create window as placeholder for captured frames and tracking */
	cvNamedWindow("IP Cam Tracker", CV_WINDOW_AUTOSIZE);

	/* get an initial frame to get stream properties */
	image = cvQueryFrame(capture);

	/* check if recording is requested is requested and init recorder */
	if(is_recording==1) record_stream_init();
	
	/* create masking images for the object markers */
	main_marker_mask_img = cvCreateImage(cvSize(MARKER_SIZE, MARKER_SIZE),
								   image->depth,
								   image->nChannels);
								   
	aux_marker_mask_img = cvCreateImage(cvSize(MARKER_SIZE, MARKER_SIZE),
								   image->depth,
								   image->nChannels);
								   
	/* create images to store matching results for marker tracking */
	main_marker_matching_img = cvCreateImage(cvSize(SEARCH_SPACE_SIZE - MARKER_SIZE +1,
											  SEARCH_SPACE_SIZE - MARKER_SIZE +1),
											  IPL_DEPTH_32F,
									          1);
   	aux_marker_matching_img = cvCreateImage(cvSize(SEARCH_SPACE_SIZE - MARKER_SIZE +1,
											  SEARCH_SPACE_SIZE - MARKER_SIZE +1),
											  IPL_DEPTH_32F,
									          1);

	/* loop until no frames are received or user decides */
	while(1) {
		/* check if recording is requested */
		if(is_recording==1){
				/* start recording frames */
				record_stream();
				/* update images */
				cvShowImage("IP Cam Tracker", image);
		}
		else{
			/* process only if we receive frames from the cam */
			if((image = cvQueryFrame(capture))!=NULL){
			/* initialize image with tracked positions to be used in visualization */
			if(obj_pos_img == NULL) {
						obj_pos_img = cvCreateImage(cvGetSize(image), IPL_DEPTH_32F, 3);
			}
			/* tracking is enabled for remote cam and recorded file */
			if(source!=LOCAL){
				/* search markers and setup once */
				if(one_time==0){
				  if(main_is_on == 0){
						search_main_marker();
				  }
				  else{
					  if(aux_is_on == 0){
						search_aux_marker();
					  }
				  }
				 if(main_is_on == 1 && aux_is_on == 1){
					setup_trackers();
					one_time = 1;
				  }
				}
				else{
					/* if already detected track the markers */
					track_main_marker();
					track_aux_marker();
					/* ... and log data */
					present_data();
				}
			}
			/* update images */
			cvShowImage("IP Cam Tracker", image);
			}
			else break;
		}
		/* check if end stream is sent */
		if((int)cvWaitKey(2) == 'q') break; 
	}
/*	
	pthread_join(conn_handler , NULL);
*/	
	/* free the allocated memory*/
	if(capture)
		cvReleaseCapture(&capture);
	/* close recorder */
	record_stream_close();
	/* destroy window handlers */
	cvDestroyWindow("IP Cam Tracker");
	/* close log file */
	fclose(f);
	return 0;
}
