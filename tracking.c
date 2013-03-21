/**
 * @author Cristian Axenie, cristian.axenie@tum.de
 * 
 * Simple tracking application using data streamed over TCP/IP 
 * from Axis IP Cam (Robot room / Holodeck), local camera or locally saved file.
 * The tracking algorithm will be used for mobile robot tracking in indoor operation.
 * 
 * Tracker functionality definitions.
 */

#include "tracking.h"
#include "utils.h"

#ifdef AUTO_FIND_MARKER
/**
 * Searches the main marker in the camera frame.
 * Algorithm uses a modified Hough transform 
 */
void search_main_marker(){
	/* detect a red circle as marker for the object */
	CvScalar hsv_min = cvScalar(170, 84, 130, 0);  /* RED marker */
    	CvScalar hsv_max = cvScalar(358, 256, 255, 0);
	
	/* allocate memory for Hough circles */
	CvMemStorage* storage = cvCreateMemStorage(0);
	/* HSV converted frame and the thresholded frame */
        IplImage *hsv_frame    = cvCreateImage(cvGetSize(frame_provider->image), IPL_DEPTH_8U, 3);
        IplImage *thresholded   = cvCreateImage(cvGetSize(frame_provider->image), IPL_DEPTH_8U, 1);
	
	/* covert RGB color space to HSV as it is much easier to filter colors in the HSV color-space */
	cvCvtColor(frame_provider->image, hsv_frame, CV_BGR2HSV);
	/* filter out colors which are out of range */
	cvInRangeS(hsv_frame, hsv_min, hsv_max, thresholded);
	/* smoothing of the image to improove Hough detector */
	cvSmooth( thresholded, thresholded, CV_GAUSSIAN, 9, 9 );
	
	/* the object main marker */
	CvSeq* marker = cvHoughCircles(thresholded, storage, CV_HOUGH_GRADIENT, 2, thresholded->height/4, 100, 50, 3, 13);
	for (int i = 0; i < marker->total; i++){	
		float* p = (float*)cvGetSeqElem( marker, 0 );
		/* store coordinate to global values */
		obj->x_pos = p[0];
		obj->y_pos = p[1];
		obj->mx = x_pos;
		obj->my = y_pos;
		obj->ms = p[2];
		obj->main_is_on = 1;
	}
	/* free the allocated memory */
	cvReleaseMemStorage(&storage);
	cvReleaseImage(&hsv_frame);
	cvReleaseImage(&thresholded);
}

/** 
 * Auxiliary markers searching. This marker is used for object angle computation. 
 * Marker is detected automatically
 */
void search_aux_marker(){
	/* detect black circles as auxiliary markers for the object */	
	CvScalar hsv_min_aux_mark = cvScalar( 83, 23, 200 ,0 ); /* BLUE marker */
        CvScalar hsv_max_aux_mark = cvScalar( 130, 53, 255 ,0); 
	
	/* allocate memory for Hough circles */
	CvMemStorage* storage_aux_mark = cvCreateMemStorage(0);
	/* HSV converted frame and the thresholded frame */
        IplImage *hsv_frame_aux_mark     = cvCreateImage(cvGetSize(frame_provider->image), IPL_DEPTH_8U, 3);
        IplImage *thresholded_aux_mark   = cvCreateImage(cvGetSize(frame_provider->image), IPL_DEPTH_8U, 1);

	/* covert RGB color space to HSV as it is much easier to filter colors in the HSV color-space */
	cvCvtColor(frame_provider->image, hsv_frame_aux_mark, CV_BGR2HSV);
	/* filter out colors which are out of range */
	cvInRangeS(hsv_frame_aux_mark, hsv_min_aux_mark, hsv_max_aux_mark, thresholded_aux_mark);
	/* smoothing of the image to improove Hough detector */
	cvSmooth( thresholded_aux_mark, thresholded_aux_mark, CV_GAUSSIAN, 9, 9 );
	
	/* the object auxiliary markers */
	CvSeq* marker0 = cvHoughCircles(thresholded_aux_mark, storage_aux_mark, CV_HOUGH_GRADIENT, 2, thresholded_aux_mark->height/5, 100, 50, 3, 13);
	for (int i = 0; i < marker0->total; i++){
		float* p0 = (float*)cvGetSeqElem( marker0, 0 );
		/* store coordinate to global values */
		obj->x_pos_aux_mark = p0[0];
		obj->y_pos_aux_mark = p0[1];
		obj->tx = x_pos_aux_mark;
		obj->ty = y_pos_aux_mark;
		obj->ts = p0[2];
		obj->aux_is_on = 1;
	}
	/* free the allocated memory */
	cvReleaseMemStorage(&storage_aux_mark);
	cvReleaseImage(&hsv_frame_aux_mark);
	cvReleaseImage(&thresholded_aux_mark);
}
#else

/**
 * Select markers with the mouse 
 */
void select_point(int event, int x_coord, int y_coord, int flags, void *param){
	/* filter mouse selection event */
	if(event == CV_EVENT_LBUTTONDOWN){
	   /* select main marker */	
	   if(obj->main_is_on==0){
		/* get the coordinate in the subimage and copy to global coord */
	 	obj->mx = x_coord;
		obj->my = y_coord;
		/* setup a rectangular ROI */
		cvSetImageROI(frame_provider->image,
			     cvRect(obj->mx,
				    obj->my,
				    8,
				    8)); 	
		/* set flag for main loop */
		obj->main_is_on = 1;
		cvResetImageROI(frame_provider->image);
		return;
	    }
	    /* select aux marker */    	
	    if(obj->aux_is_on==0){
		obj->tx = x_coord;
                obj->ty = y_coord;
                /* setup a rectangular ROI */
                cvSetImageROI(frame_provider->image,
                             cvRect(obj->tx,
                                    obj->ty,
                                    8,
                                    8));
                /* set flag for main loop */
                obj->aux_is_on = 1;
		cvResetImageROI(frame_provider->image);
		return;
	    }
	}
}
#endif

/**
 * Object markers setup used for heading angle computation
 */
void setup_trackers(){
	/* initialization with respect to the main object marker */
	obj->x_pos_aux_mark = obj->tx;
	obj->y_pos_aux_mark = obj->ty;
	obj->x_pos = obj->mx;
	obj->y_pos = obj->my;
		/* setup a rectangular region of interest */
		cvSetImageROI(frame_provider->image,
		              cvRect( obj->x_pos,
		                      obj->y_pos,
		                      MARKER_SIZE,
		                      MARKER_SIZE));
		/* copy the region of interest into the template image */
		cvCopy(frame_provider->image, trk->main_marker_mask_img, NULL);					  
		
		cvSetImageROI(frame_provider->image,
		              cvRect( obj->x_pos_aux_mark,
		                      obj->y_pos_aux_mark,
				      MARKER_SIZE,
		                      MARKER_SIZE));
		/* copy the region of interest into the template image */
		cvCopy(frame_provider->image, trk->aux_marker_mask_img, NULL);
		
		/* reset the region of interest */
		cvResetImageROI(frame_provider->image);
}

/**
 * Markers tracking functions using template matching 
 */
void track_aux_marker(){
	/* get the frame properties */
	int* props = get_stream_properties(frame_provider->capture);
	/* min and max locations in the image */
	CvPoint min_pt_loc_aux_mark, max_pt_loc_aux_mark;
	/* min and max values in the image */
	double min_pt_val_aux_mark, max_pt_val_aux_mark;
	
	/* setup position of the search space window */
	int search_window_x_pos = obj->x_pos_aux_mark - ((SEARCH_SPACE_SIZE - MARKER_SIZE)/2);
	int search_window_y_pos = obj->y_pos_aux_mark - ((SEARCH_SPACE_SIZE - MARKER_SIZE)/2);

	/* check if the search space window is in the captured frame boundries */
	if(search_window_x_pos < props[1] && 
	   search_window_y_pos < props[0] &&
	   search_window_x_pos >0  && 
	   search_window_y_pos > 0) {
		
		/* search object in the search space window */
		cvSetImageROI(frame_provider->image,
		              cvRect(search_window_x_pos,
		                     search_window_y_pos,
		                     SEARCH_SPACE_SIZE,
		                     SEARCH_SPACE_SIZE
		                    ));

		/* compare template against overlaped image regions */
		cvMatchTemplate(frame_provider->image, 		/* frame where we perform the search */
		                trk->aux_marker_mask_img,	/* searched template */
		                trk->aux_marker_matching_img,	/* map of comparison results */
		                CV_TM_SQDIFF_NORMED		/* method=CV_TM_SQDIFF_NORMED */
		               );
		/* find global minimum and maximum (position and value) in resulting map of comparison results */
		cvMinMaxLoc(trk->aux_marker_matching_img, &min_pt_val_aux_mark, &max_pt_val_aux_mark, &min_pt_loc_aux_mark, &max_pt_loc_aux_mark, 0 /* no mask */);
		/* reset region of interest */
		cvResetImageROI(frame_provider->image);

		/* check if any object was found */
		if(min_pt_val_aux_mark <= MATCHING_THRESHOLD) {

			/* save the object current location */
			obj->x_pos_aux_mark = search_window_x_pos + min_pt_loc_aux_mark.x;
			obj->y_pos_aux_mark = search_window_y_pos + min_pt_loc_aux_mark.y;
#ifdef VERBOSE
			/* draw an identification rectangle there */
			cvRectangle(frame_provider->image,				/* image to draw in */
			            cvPoint(obj->x_pos_aux_mark, obj->y_pos_aux_mark), 	/* one vertex */
			            cvPoint(obj->x_pos_aux_mark + MARKER_SIZE/2,	/* opposite vertex */
			                    obj->y_pos_aux_mark + MARKER_SIZE/2),  
			            cvScalar( 255, 0, 0, 0 ),				/* color */
			            2,							/* lines thickness */
			            0,							/* line type */
			            0							/* number of fractional bits in the point coordinates */
			           );
#endif
		}
	} 
} 

void track_main_marker(){
	/* get the frame properties */
	int* props = get_stream_properties(frame_provider->capture);
	/* min and max locations in the image */
	CvPoint min_pt_loc, max_pt_loc;
	/* min and max values in the image */
	double min_pt_val, max_pt_val;
	
	/* setup position of the search space window */
	int search_window_x_pos = obj->x_pos - ((SEARCH_SPACE_SIZE - MARKER_SIZE)/2);
	int search_window_y_pos = obj->y_pos - ((SEARCH_SPACE_SIZE - MARKER_SIZE)/2);

	/* check if the search space window is in the captured frame boundries */
	if(search_window_x_pos < props[1] && 
	   search_window_y_pos < props[0] &&
	   search_window_x_pos >0  && 
	   search_window_y_pos > 0) {
		
		/* search object in the search space window */
		cvSetImageROI(frame_provider->image,
		              cvRect(search_window_x_pos,
		                     search_window_y_pos,
		                     SEARCH_SPACE_SIZE,
		                     SEARCH_SPACE_SIZE
		                    ));

		/* compare template against overlaped image regions */
		cvMatchTemplate(frame_provider->image,			/* frame where we perform the search */
		                trk->main_marker_mask_img, 			/* searched template */
		                trk->main_marker_matching_img,		/* map of comparison results */
		                CV_TM_SQDIFF_NORMED			/* method=CV_TM_SQDIFF_NORMED */
		               );
		/* find global minimum and maximum (position and value) in resulting map of comparison results */
		cvMinMaxLoc(trk->main_marker_matching_img, &min_pt_val, &max_pt_val, &min_pt_loc, &max_pt_loc, 0 /* no mask */);
		/* reset region of interest */
		cvResetImageROI(frame_provider->image);

		/* check if any object was found */
		if(min_pt_val <= MATCHING_THRESHOLD) {

			/* save the object current location */
			obj->x_pos = search_window_x_pos + min_pt_loc.x;
			obj->y_pos = search_window_y_pos + min_pt_loc.y;
#ifdef VERBOSE
			/* draw an identification rectangle there */
			cvRectangle(frame_provider->image,		 /* image to draw in */
			            cvPoint(obj->x_pos, obj->y_pos), 	 /* one vertex */
			            cvPoint(obj->x_pos + MARKER_SIZE/2,	 /* opposite vertex */
			                    obj->y_pos + MARKER_SIZE/2),
			            cvScalar( 0, 0, 255, 0 ),		 /* color */
			            2,					 /* lines thickness */
			            0,					 /* line type */
			            0					 /* number of fractional bits in the point coordinates */
			           );
#endif
		}
	} 
} 

/**
 * Compute real world pose of the object and pack data 
 * for file logging, remote sending or visualization
 */
void present_data(){
	/* local handlers of the world coordinates */
        static float X = 0.0f;
        static float Y = 0.0f;
        static float theta = 0.0f;
	/* last position of the main object marker in the frame */
	static int x_pos_ant = 0;
	static int  y_pos_ant = 0;
	/* duplicated vars for visualization */
	static int  x_pos_ant_vis = 0;
	static int y_pos_ant_vis = 0;
	/* flag for clear initialization */
	static int init_fix = 0;
	/* GUI printed string */
	char pos_val[200];
	/* init coordinates */
	if(obj->init==0){
		X = 0.0f;
		Y = 0.0f;
		theta = 0.0f;
		obj->init = 1;
	}
	else{
		/* main marker camera coordinates to world coordinates transformation and update */
		if(obj->x_pos > x_pos_ant){
			X += abs(obj->x_pos - x_pos_ant)*CONVERSION_FACTOR_X;
			if(obj->y_pos > y_pos_ant){
				Y -= abs(obj->y_pos - y_pos_ant)*CONVERSION_FACTOR_Y;	
			}
			else{
				Y += abs(obj->y_pos - y_pos_ant)*CONVERSION_FACTOR_Y;
			}
		}
		else{
			X -= abs(obj->x_pos - x_pos_ant)*CONVERSION_FACTOR_X;
			if(obj->y_pos > y_pos_ant){
				Y -= abs(obj->y_pos - y_pos_ant)*CONVERSION_FACTOR_Y;	
			}
			else{
				Y += abs(obj->y_pos - y_pos_ant)*CONVERSION_FACTOR_Y;
			}
		}

		/* compute the heading angle */
		theta = 180 - 180*atan2(obj->x_pos_aux_mark - obj->x_pos, obj->y_pos_aux_mark - obj->y_pos)/CV_PI;	
		
		/* clear problems when initializing, ensure 0 at startup */
		if(init_fix == 0){
			if(X > 0.0f || X <0.0f) X=0.0f;
			if(Y > 0.0f || Y <0.0f) Y=0.0f;
			if(theta > 0.0f || theta < 0.0f) theta =0.0f;
			init_fix = 1;
		}
        /* fill in the log struct */
	log_bin[trk->idx].xpos = X;
	log_bin[trk->idx].ypos = Y;
	log_bin[trk->idx].heading = theta;
	log_bin[trk->idx].sample = trk->idx;

	/* get current time for timestamp computation */
	if(clock_gettime(CLOCK_REALTIME, &tcur)==-1){
		printf("main: Cannot access time subsystem in the loop\n");
	}
	log_bin[trk->idx].timestamp = compute_dt(&tcur, &tstart);

	/* write in the stream buffer */
        sprintf(obj->buffer, "%f,%f,%f,%d,%lf\n", X, Y, theta, trk->idx, log_bin[trk->idx].timestamp);

	/* update position in the GUI */
	sprintf(pos_val, "DEBUG [ X: %f Y: %f   -     xc: %d yc: %d     -    theta %f]", X, Y, obj->x_pos, obj->y_pos, theta);

		cvPutText(frame_provider->image,				/* the image to write on */ 
			  pos_val,  						/* the string to write */
			  cvPoint(SEARCH_SPACE_SIZE, SEARCH_SPACE_SIZE),	/* where to place text box */
			  &font, 						/* which font to use */
			  cvScalar(255, 255, 255, 0)); 		 		/* line properties */

#ifdef VERBOSE
	/* superimpose the trace of the tracked object */
	if(x_pos_ant_vis>0 && y_pos_ant_vis>0) {	
		if(obj->x_pos>0 && obj->y_pos>0) {
			/* create a tracking line marker between 2 succesive points while target is moving (object trace) */
			cvLine(trk->obj_pos_img, cvPoint(obj->x_pos, obj->y_pos), cvPoint(x_pos_ant_vis, y_pos_ant_vis), cvScalar(0,0,255,0.0), 3, 8, 0);
		}
	}

	cvAdd(frame_provider->image, trk->obj_pos_img, frame_provider->image, NULL);
 
	/* update history for visualization */
	x_pos_ant_vis = obj->x_pos;
	y_pos_ant_vis = obj->y_pos;
#endif		
	/* update history for tracking */
	x_pos_ant = obj->x_pos;
	y_pos_ant = obj->y_pos;
	trk->idx++;
  }
}

