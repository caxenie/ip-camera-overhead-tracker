/**
 * @author Cristian Axenie, cristian.axenie@tum.de
 * 
 * Simple tracking application using data streamed over RTSP 
 * from Axis IP Cam (Robot room / Holodeck) ,local camera or locally saved file.
 * The tracking algorithm will be used for mobile robot tracking in indoor operation.
 * 
 * Tracker functionality.
 */

#include <stdio.h>
#include "highgui.h"
#include "cv.h"
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


#define MARKER_SIZE 		10		/* the object markers size */	
#define SEARCH_SPACE_SIZE   65		/* search space window size for template matching */
#define MATCHING_THRESHOLD  0.5		/* template matching threshold */	
#define CONVERSION_FACTOR_X	0.3266	/* cam space to world space mapping value for X axis */
#define CONVERSION_FACTOR_Y	0.3251	/* cam space to world space mapping value for Y axis */
#define MAXBUF				50 	   	/* max buffer length for data sending */
#define PORT 			"56000"	/* port number exposed by the server */
#define BACKLOG 		 20     // how many pending connections queue will hold
#define TIME_SIZE 			40		/* max time stamp string size */
#define MAX_LOG_SIZE 		9999
#define VERBOSE						/* get markers, trace and additional info */

// #define AUTO_FIND_MARKERS		/* detects markers automatically - not stable */

 /* image header that will save the tracked positions of the object */
IplImage* obj_pos_img = NULL;
/* image headers that will contain templates and matching results */
IplImage *main_marker_mask_img, *aux_marker_mask_img;
IplImage *main_marker_matching_img, *aux_marker_matching_img;
/* current position of the main object marker in the frame */
int x_pos = 0, y_pos = 0;
/* current position of the auxiliary marker in the frame */
int x_pos0 = 0, y_pos0 = 0;
/* aux and main markers auxiliry pose vars */
int mx = 0, my = 0, ms = 0, tx = 0, ty = 0, ts = 0;
/* flags to mark if the markers were detected in the frame */
short main_is_on = 0, aux_is_on = 0;
/* last position of the main object marker in the frame - duplicated vars for visualization */
int x_pos_ant = 0, y_pos_ant = 0, x_pos_ant_vis = 0, y_pos_ant_vis = 0;;
/* last position of the auxiliary marker in the frame */
int x_pos_ant0 = 0, y_pos_ant0 = 0;
/* timing index */
int idx = 0;
/* flag for clear initialization */
int init_fix = 0;

/* initialize font and add text and position string */
CvFont font;
char pos_val[30];

/* main marker pose x,y,theta in world space */
float X = 0.0f;
float Y = 0.0f;
float theta=0.0f;
/* main marker pose x,y in world space */
float Xm = 0.0f;
float Ym = 0.0f;

/* flag to init global coordinates */
short init = 0;
/* data buffer to store tracking history */
char buffer[MAXBUF];
/* lock for writing the data to the buffer */
pthread_mutex_t buff_lock = PTHREAD_MUTEX_INITIALIZER;
/* ok to send flag */
short on_send = 0;

void sigchld_handler(int s)
{
    while(waitpid(-1, NULL, WNOHANG) > 0);
}

// get sockaddr, IPv4 or IPv6:
void *get_in_addr(struct sockaddr *sa)
{
    if (sa->sa_family == AF_INET) {
        return &(((struct sockaddr_in*)sa)->sin_addr);
    }

    return &(((struct sockaddr_in6*)sa)->sin6_addr);
}


/**
 * Separate thread to accept socket connections
 * for sending the tracking data to a remote
 * machine
 */
void * remote_connections_handler(void *data){
	 int sockfd, new_fd;  // listen on sock_fd, new connection on new_fd
    struct addrinfo hints, *servinfo, *p;
    struct sockaddr_storage their_addr; // connector's address information
    socklen_t sin_size;
    struct sigaction sa;
    int yes=1;
    char s[INET6_ADDRSTRLEN];
    int rv;

    memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_flags = AI_PASSIVE; // use my IP

    if ((rv = getaddrinfo(NULL, PORT, &hints, &servinfo)) != 0) {
        fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(rv));
        return NULL;
    }

    // loop through all the results and bind to the first we can
    for(p = servinfo; p != NULL; p = p->ai_next) {
        if ((sockfd = socket(p->ai_family, p->ai_socktype,
                p->ai_protocol)) == -1) {
            perror("server: socket");
            continue;
        }

        if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &yes,
                sizeof(int)) == -1) {
            perror("setsockopt");
            exit(1);
        }

        if (bind(sockfd, p->ai_addr, p->ai_addrlen) == -1) {
            close(sockfd);
            perror("server: bind");
            continue;
        }

        break;
    }

    if (p == NULL)  {
        fprintf(stderr, "server: failed to bind\n");
        return NULL;
    }

    freeaddrinfo(servinfo); // all done with this structure

    if (listen(sockfd, BACKLOG) == -1) {
        perror("listen");
        exit(1);
    }

    sa.sa_handler = sigchld_handler; // reap all dead processes
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = SA_RESTART;
    if (sigaction(SIGCHLD, &sa, NULL) == -1) {
        perror("sigaction");
        exit(1);
    }

    while(1) {  // main accept() loop
        sin_size = sizeof their_addr;
        new_fd = accept(sockfd, (struct sockaddr *)&their_addr, &sin_size);
        if (new_fd == -1) {
            perror("accept");
            continue;
        }

        inet_ntop(their_addr.ss_family,
            get_in_addr((struct sockaddr *)&their_addr),
            s, sizeof s);

        //if (!fork()) { // this is the child process
        //    close(sockfd); // child doesn't need the listener
            while(1){
		//if(on_send==1){
			if (send(new_fd, buffer, sizeof(buffer), 0) == -1)
                	perror("send");
		//on_send=0;
		//}
	    }
        //}
        //close(new_fd);  // parent doesn't need this
  //  }	
  }
	return NULL;
}

#ifdef AUTO_FIND_MARKER
/**
 * Searches the main marker in the camera frame.
 * Algorithm uses a modified Hough transform 
 */
void search_main_marker(){
	/* detect a red circle as marker for the object */
    //CvScalar hsv_min = cvScalar(150, 84, 130, 0);//cvScalar(10, 64, 110, 0); /* big 5-100 RED marker */
    //CvScalar hsv_max = cvScalar(358, 256, 255, 0);//cvScalar(300, 180, 200, 0);
	CvScalar hsv_min = cvScalar(170, 84, 130, 0); /* small 3-13 RED marker */
    	CvScalar hsv_max = cvScalar(358, 256, 255, 0);
	
	/* allocate memory for Hough circles */
	CvMemStorage* storage = cvCreateMemStorage(0);
	/* HSV converted frame and the thresholded frame */
    IplImage *hsv_frame    = cvCreateImage(cvGetSize(image), IPL_DEPTH_8U, 3);
    IplImage *thresholded   = cvCreateImage(cvGetSize(image), IPL_DEPTH_8U, 1);
	
	/* covert RGB color space to HSV as it is much easier to filter colors in the HSV color-space */
	cvCvtColor(image, hsv_frame, CV_BGR2HSV);
	/* filter out colors which are out of range */
	cvInRangeS(hsv_frame, hsv_min, hsv_max, thresholded);
	/* smoothing of the image to improove Hough detector */
	cvSmooth( thresholded, thresholded, CV_GAUSSIAN, 9, 9 );
	
	/* the object main marker */
	CvSeq* marker = cvHoughCircles(thresholded, storage, CV_HOUGH_GRADIENT, 2,
									thresholded->height/4, 100, 50, 3, 13);
	for (int i = 0; i < marker->total; i++){	
		float* p = (float*)cvGetSeqElem( marker, 0 );
		/* store coordinate to global values */
		x_pos = p[0];
		y_pos = p[1];
		mx = x_pos;
		my = y_pos;
		ms = p[2];
		main_is_on = 1;
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
	CvScalar hsv_min0 = cvScalar( 83, 23, 200 ,0 ); /* BLUE marker */
    CvScalar hsv_max0 = cvScalar( 130, 53, 255 ,0); 
	
	/* allocate memory for Hough circles */
	CvMemStorage* storage0 = cvCreateMemStorage(0);
	/* HSV converted frame and the thresholded frame */
    IplImage *hsv_frame0     = cvCreateImage(cvGetSize(image), IPL_DEPTH_8U, 3);
    IplImage *thresholded0   = cvCreateImage(cvGetSize(image), IPL_DEPTH_8U, 1);

	/* covert RGB color space to HSV as it is much easier to filter colors in the HSV color-space */
	cvCvtColor(image, hsv_frame0, CV_BGR2HSV);
	/* filter out colors which are out of range */
	cvInRangeS(hsv_frame0, hsv_min0, hsv_max0, thresholded0);
	/* smoothing of the image to improove Hough detector */
	cvSmooth( thresholded0, thresholded0, CV_GAUSSIAN, 9, 9 );
	
	/* the object auxiliary markers */
	CvSeq* marker0 = cvHoughCircles(thresholded0, storage0, CV_HOUGH_GRADIENT, 2,
									thresholded0->height/5, 100, 50, 3, 13);
	for (int i = 0; i < marker0->total; i++){
		float* p0 = (float*)cvGetSeqElem( marker0, 0 );
		/* store coordinate to global values */
		x_pos0 = p0[0];
		y_pos0 = p0[1];
		tx = x_pos0;
		ty = y_pos0;
		ts = p0[2];
		aux_is_on = 1;
		
	}
	/* free the allocated memory */
	cvReleaseMemStorage(&storage0);
	cvReleaseImage(&hsv_frame0);
	cvReleaseImage(&thresholded0);
}
#else

/* Select main marker with the mouse */
void select_point(int event, int x_coord, int y_coord, int flags, void *param)
{
	/* filter mouse selection event */
	if(event == CV_EVENT_LBUTTONDOWN){
	   /* select main marker */	
	   if(main_is_on==0){
		/* get the coordinate in the subimage and copy to global coord */
	 	mx = x_coord;
		my = y_coord;
		/* setup a rectangular ROI */
		cvSetImageROI(image,
			     cvRect(mx,
				    my,
				    8,
				    8)); 	
		/* set flag for main loop */
		main_is_on = 1;
		cvResetImageROI(image);
		return;
	    }
	    /* select aux marker */    	
	    if(aux_is_on==0){
		tx = x_coord;
                ty = y_coord;
                /* setup a rectangular ROI */
                cvSetImageROI(image,
                             cvRect(tx,
                                    ty,
                                    8,
                                    8));
                /* set flag for main loop */
                aux_is_on = 1;
		cvResetImageROI(image);
		return;
	    }
	}
}

#endif


/**
 * Object markers setup used for heading angle computation
 */
void setup_trackers()
{
	/* initialization with respect to the main object marker */
	x_pos0 = tx;
	y_pos0 = ty;
	x_pos = mx;
	y_pos = my;
	/* setup a rectangular region of interest */
		cvSetImageROI(image,
		              cvRect( x_pos,
		                      y_pos,
		                      MARKER_SIZE,
		                      MARKER_SIZE));
		/* copy the region of interest into the template image */
		cvCopy(image, main_marker_mask_img, NULL);					  
		
		cvSetImageROI(image,
		              cvRect( x_pos0,
		                      y_pos0,
							  MARKER_SIZE,
		                      MARKER_SIZE));
		/* copy the region of interest into the template image */
		cvCopy(image, aux_marker_mask_img, NULL);
		
		/* reset the region of interest */
		cvResetImageROI(image);
}

/**
 * Markers tracking functions using template matching 
 */
void track_aux_marker(){
	/* get the frame properties */
	int* props = get_stream_properties(capture);
	/* min and max locations in the image */
	CvPoint min_pt_loc, max_pt_loc;
	/* min and max values in the image */
	double min_pt_val, max_pt_val;
	
	/* setup position of the search space window */
	int search_window_x_pos = x_pos0 - ((SEARCH_SPACE_SIZE - MARKER_SIZE)/2);
	int search_window_y_pos = y_pos0 - ((SEARCH_SPACE_SIZE - MARKER_SIZE)/2);

	/* check if the search space window is in the captured frame boundries */
	if(search_window_x_pos < props[1] && 
	   search_window_y_pos < props[0] &&
	   search_window_x_pos >0  && 
	   search_window_y_pos > 0) {
		
		/* search object in the search space window */
		cvSetImageROI(image,
		              cvRect(search_window_x_pos,
		                     search_window_y_pos,
		                     SEARCH_SPACE_SIZE,
		                     SEARCH_SPACE_SIZE
		                    ));

		/* compare template against overlaped image regions */
		cvMatchTemplate(image, 							/* frame where we perform the search */
		                aux_marker_mask_img, 			/* searched template */
		                aux_marker_matching_img,		/* map of comparison results */
		                CV_TM_SQDIFF_NORMED				/* method=CV_TM_SQDIFF_NORMED */
		               );
		/* find global minimum and maximum (position and value) in resulting map of comparison results */
		cvMinMaxLoc(aux_marker_matching_img, &min_pt_val, &max_pt_val, &min_pt_loc, &max_pt_loc, 0 /* no mask */);
		/* reset region of interest */
		cvResetImageROI(image);

		/* check if any object was found */
		if(min_pt_val <= MATCHING_THRESHOLD) {

			/* save the object current location */
			x_pos0 = search_window_x_pos + min_pt_loc.x;
			y_pos0 = search_window_y_pos + min_pt_loc.y;
#ifdef VERBOSE
			/* draw an identification rectangle there */
			cvRectangle(image,								/* image to draw in */
			            cvPoint(x_pos0, y_pos0), 			/* one vertex */
			            cvPoint(x_pos0 + MARKER_SIZE,		/* opposite vertex */
			                    y_pos0 + MARKER_SIZE),
			            cvScalar( 255, 0, 0, 0 ),			/* color */
			            2,									/* lines thickness */
			            0,									/* line type */
			            0									/* number of fractional bits in the point coordinates */
			           );
#endif

		}
	} 
} 

void track_main_marker()
{
	/* get the frame properties */
	int* props = get_stream_properties(capture);
	/* min and max locations in the image */
	CvPoint min_pt_loc, max_pt_loc;
	/* min and max values in the image */
	double min_pt_val, max_pt_val;
	
	/* setup position of the search space window */
	int search_window_x_pos = x_pos - ((SEARCH_SPACE_SIZE - MARKER_SIZE)/2);
	int search_window_y_pos = y_pos - ((SEARCH_SPACE_SIZE - MARKER_SIZE)/2);

	/* check if the search space window is in the captured frame boundries */
	if(search_window_x_pos < props[1] && 
	   search_window_y_pos < props[0] &&
	   search_window_x_pos >0  && 
	   search_window_y_pos > 0) {
		
		/* search object in the search space window */
		cvSetImageROI(image,
		              cvRect(search_window_x_pos,
		                     search_window_y_pos,
		                     SEARCH_SPACE_SIZE,
		                     SEARCH_SPACE_SIZE
		                    ));

		/* compare template against overlaped image regions */
		cvMatchTemplate(image, 							/* frame where we perform the search */
		                main_marker_mask_img, 			/* searched template */
		                main_marker_matching_img,		/* map of comparison results */
		                CV_TM_SQDIFF_NORMED				/* method=CV_TM_SQDIFF_NORMED */
		               );
		/* find global minimum and maximum (position and value) in resulting map of comparison results */
		cvMinMaxLoc(main_marker_matching_img, &min_pt_val, &max_pt_val, &min_pt_loc, &max_pt_loc, 0 /* no mask */);
		/* reset region of interest */
		cvResetImageROI(image);

		/* check if any object was found */
		if(min_pt_val <= MATCHING_THRESHOLD) {

			/* save the object current location */
			x_pos = search_window_x_pos + min_pt_loc.x;
			y_pos = search_window_y_pos + min_pt_loc.y;
#ifdef VERBOSE
			/* draw an identification rectangle there */
			cvRectangle(image,								/* image to draw in */
			            cvPoint(x_pos, y_pos), 				/* one vertex */
			            cvPoint(x_pos + MARKER_SIZE,		/* opposite vertex */
			                    y_pos + MARKER_SIZE),
			            cvScalar( 0, 0, 255, 0 ),			/* color */
			            2,									/* lines thickness */
			            0,									/* line type */
			            0									/* number of fractional bits in the point coordinates */
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
	/* init coordinates */
	if(init==0){
		X = 0.0f;
		Y = 0.0f;
		theta = 0.0f;
		init = 1;
	}
	else{
		/* main marker camera coordinates to world coordinates transformation and update */
		if(x_pos > x_pos_ant){
			X += abs(x_pos - x_pos_ant)*CONVERSION_FACTOR_X;
			if(y_pos > y_pos_ant){
				Y -= abs(y_pos - y_pos_ant)*CONVERSION_FACTOR_Y;	
			}
			else{
				Y += abs(y_pos - y_pos_ant)*CONVERSION_FACTOR_Y;
			}
		}
		else{
			X -= abs(x_pos - x_pos_ant)*CONVERSION_FACTOR_X;
			if(y_pos > y_pos_ant){
				Y -= abs(y_pos - y_pos_ant)*CONVERSION_FACTOR_Y;	
			}
			else{
				Y += abs(y_pos - y_pos_ant)*CONVERSION_FACTOR_Y;
			}
		}
		
		/* compute the heading angle */
		theta = 180 - 180*atan2(x_pos0 - x_pos, y_pos0 - y_pos)/CV_PI;	
		
		/* clear problems when initializing, ensure 0 at startup */
		if(init_fix == 0){
			if(X > 0.0f || X <0.0f) X=0.0f;
			if(Y > 0.0f || Y <0.0f) Y=0.0f;
			if(theta > 0.0f || theta < 0.0f) theta =0.0f;
			init_fix = 1;
		}
	/* update log data: X,Y,theta,timestamp */
        /* get current frame time */
        if(clock_gettime(CLOCK_REALTIME, &tcur)==-1){
                   printf("Cannot access time subsystem.");
        }
        /* compute the timestamp */
        log_file[idx].timestamp = compute_dt(&tcur, &tstart);
	log_file[idx].xpos = X;
	log_file[idx].ypos = Y;
	log_file[idx].heading = theta;
	log_file[idx].sample = idx;
	//if(idx%5==0){
			/* sync to socket send */
	//		on_send = 1;
			/* lock buffer for writing */
	//		pthread_mutex_lock(&buff_lock);
			/*prepare buffer to be sent to remote connections in the second thread */;
			sprintf(buffer, "%f,%f,%f,%d,%lf\n", X, Y, theta, idx, log_file[idx].timestamp);
			/* unlock buffer */
	//		pthread_mutex_unlock(&buff_lock);
	//}
	/* update position in the frame */
	sprintf(pos_val, "DEBUG [ X: %f Y: %f   -     xc: %d yc: %d     -    theta %f]", X, Y, x_pos, y_pos, theta);
			cvPutText(image, 														/* the image to write on */  
					  pos_val, 														/* the string to write */
					  cvPoint(SEARCH_SPACE_SIZE, SEARCH_SPACE_SIZE),     			/* where to place text box */
					  &font, 														/* which font to use */
					  cvScalar(255, 255, 255, 0)); 									/* line properties */
#ifdef VERBOSE
	/* superimpose the trace of the tracked object */
	if(x_pos_ant_vis>0 && y_pos_ant_vis>0) {
		if(x_pos>0 && y_pos>0) {
			/* create a tracking line marker between 2 succesive points while target is moving (object trace) */
	cvLine(obj_pos_img, cvPoint(x_pos, y_pos), cvPoint(x_pos_ant_vis, y_pos_ant_vis), cvScalar(0,0,255), 3);
		}
	}
	cvAdd(image, obj_pos_img, image);

	/* update history for visualization */
	x_pos_ant_vis = x_pos;
	y_pos_ant_vis = y_pos;
#endif		
	/* timed update history for tracking */
	if(idx%10==0){
		x_pos_ant = x_pos;
		y_pos_ant = y_pos;
		x_pos_ant0 = x_pos0;
		y_pos_ant0 = y_pos0;
	}
	idx++;
  }
}

