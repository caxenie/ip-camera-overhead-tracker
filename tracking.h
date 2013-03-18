/**
 * @author Cristian Axenie, cristian.axenie@tum.de
 * 
 * Simple tracking application using data streamed over RTSP 
 * from Logilink IP Cameras,local camera or locally saved file.
 * The tracking algorithm will be used for mobile robot tracking in indoor operation.
 * 
 * Tracker functionality.
 */

#ifndef TRACKING_H_
#define TRACKING_H_

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

#define MARKER_SIZE 		10		/* the object markers size */	
#define SEARCH_SPACE_SIZE   65		/* search space window size for template matching */
#define MATCHING_THRESHOLD  0.5		/* template matching threshold */	
#define CONVERSION_FACTOR_X	0.27	/* cam space to world space mapping value for X axis */
#define CONVERSION_FACTOR_Y	0.25	/* cam space to world space mapping value for Y axis */
#define MAXBUF				50 	   	/* max buffer length for data sending */
#define PORT_NUM 			56000	/* port number exposed by the server */
#define TIME_SIZE 			40		/* max time stamp string size */

#define VERBOSE						/* get markers, trace and additional info */

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
//pthread_mutex_t buff_lock = PTHREAD_MUTEX_INITIALIZER;
/* ok to send flag */
short on_send = 0;

/**
 * Add a system time timestamp to the tracking information
 */
char* insert_timestamp(){
	/* only if file exists */
	if(f!=NULL)
	{
		/* time keeping structure */
		const struct tm *tm;
		/* to extract also ms */
		struct timeb tp;
		/* get time with finer granularity */
		ftime(&tp);
		/* timestamp representation length */
		size_t len;
		/* store current time */
		time_t now;
		
		/* allocate string to hold the timestamp */
		char *s = (char*)calloc ( TIME_SIZE, sizeof (char) );
		char *ms = (char*)calloc ( TIME_SIZE, sizeof (char) );
		
		/* get time */
		now = time ( NULL );
		/* get extensive representation of the time data */
		tm = localtime ( &now );
		/* convert to string representation and return it */
		len = strftime ( s, TIME_SIZE, "%I:%M:%S", tm );
		/* get ms */
		sprintf(ms, ":%d", tp.millitm);
		/* final string encoded timestamp */
		strcat(s,ms);
		return s;
	}
	return (char*)"-";
}

/**
 * Initialize remote access setup using sockets
 */
int init_remote_access(){
	/* socket filedes of the server */
	int sockfd = 0;
	/* server socket information */
	struct sockaddr_in self;
	/* option value access var */
	int tr=1;
	int sock_buf_size = 100000;
	
	/* create streaming socket */
    if ( (sockfd = socket(AF_INET, SOCK_STREAM, 0)) < 0)
	{
		printf("init_remote_access: error creating socket!\n");
		perror("Socket");
		exit(errno);
	}
	
	/* set socket to non-blocking */
	fcntl(sockfd, F_SETFL, O_NONBLOCK);  
	/* set socket to asynchronous I/O */
	fcntl(sockfd, F_SETFL, O_ASYNC);     
 
	/* initialize address/port structure */
	bzero(&self, sizeof(self));
	self.sin_family = AF_INET;
	self.sin_port = htons(PORT_NUM);
	self.sin_addr.s_addr = INADDR_ANY;

	/* set socket updated opts */
	if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR,&tr,sizeof(int)) == -1) {
		/* allow address reuse */
		printf("init_remote_access: error setting addr reuse socket option!\n");
		perror("setsockopt");
	}
	
	if (setsockopt(sockfd, SOL_SOCKET, SO_SNDBUF, (char *)&sock_buf_size, sizeof(sock_buf_size)) == -1) {
		/* allow bigger send buffer */
		printf("init_remote_access: error setting buff len socket option!\n");
		perror("setsockopt");
	}	

	/* assign a port number to the socket */
    if ( bind(sockfd, (struct sockaddr*)&self, sizeof(self)) != 0 )
	{
		printf("init_remote_access: bind error!\n");
		perror("socket--bind");
		exit(errno);
	}

	/* make it a listening socket */
	if ( listen(sockfd, 20) != 0 )
	{
		printf("init_remote_access: listen error!\n");
		perror("socket--listen");
		exit(errno);
	}	
	return sockfd;
}

/**
 * Accepts incoming connection and sends the tracking 
 * data over the network
 */
void send_tracking_data(int clientfd){
			/* send tracking data */
			send(clientfd, buffer, sizeof(buffer), MSG_DONTWAIT | MSG_MORE);
			usleep(1000);/* us */
}

/**
 * Close connection when tracking finished
 */
void close_remote_access(int clientfd, int sockfd){
			/* close data connection */
			close(clientfd);
			/* clean up (should never get here!) */
			close(sockfd);
}

/** 
 * Return accepted socket or -1 to avoid blocking and waiting. 
 * If timeout is 0, wait without timeout 
 */
int accept_and_noblock(int timeout, int sockfd){
	/* connected client file descriptor */
	int clientfd = 0;
	/* client information */
	struct sockaddr_in client_addr;
	/* client address size */
	int addrlen=sizeof(client_addr);
	/* timeval struct to store timing information */
    struct timeval tmo;
	/* RW fildes for synchronous I/O multiplexing */
    fd_set fds, efds;
	
	/* timeout in seconds */
    tmo.tv_sec = timeout; 
    tmo.tv_usec = 1;

	/* multiplex sockets in the single-threaded environment*/
     FD_ZERO(&fds); 
	 FD_SET(sockfd,&fds); 
	 FD_ZERO(&efds); 
	 FD_SET(sockfd,&efds);
	 
	 /* FIXME: it seems that select() fails sometimes due to the
	  * use of FD_* family macros when settign bit arrays for
	  * fd_sets. This is a well known problem in a multithreaded
	  * environment using sockets. The alternative is using the
	  * poll() function which is not limited to a FD_SIZE of 1024
	  * so we can have fds bigger than the limit. 
	  * A workaround here is using local vars to store and pass
	  * as function args instead of using global (shared) vars for
	  * communication parameters.
	  */
	  
#if 0
	struct pollfd fds[2]; 
	fds[0].fd = sockfd;
	fds[0].events= POLLIN | POLLPRI;
	
	fds[1].fd = sockfd;
	fds[1].events= POLLOUT | POLLPRI;
#endif	

	/* check sockets for R/W or pending operations before timeout expires */
    if(select (sockfd+1,&fds,NULL,&efds, timeout ? &tmo:(struct timeval *)0)){
		
#if 0
	if (timeout > INT_MAX)
        {
            /* Sorry poll only takes int */
            timeout = INT_MAX;

        }
	int rc = 0;
	if((rc = poll(fds, 2, timeout))>0){		
		if (fds[1].revents & POLLOUT) {
#endif

	  /* check if we have a client wanting to connect and if any return its file descriptor */
      if((clientfd = accept(sockfd,(struct sockaddr*)&client_addr,(socklen_t *)&addrlen))>0){
        return(clientfd);
	  }
	  
#if 0
	}
   }else{
	   if(rc==-1)
			printf("ERROR \n");
	   else 
		   printf("TIMEOUT\n");
   }
#endif

	} /* end select() */
   return(-1);
} 

/**
 * Separate thread to accept socket connections
 * for sending the tracking data to a remote
 * machine
 */
void *remote_connections_handler(void *in){
	/* init remote broadcast of tracking data */
	/* get server socket and setup communication parameters */
	int srv = init_remote_access();
	/* init client filedes */
	int cl = 0;
	/* flag to mark that a client is connected */
	short cl_on = 0;
	/* wait for connections */
	while(1){
			/* check if any clients want to connect */
			if((cl = accept_and_noblock(1, srv))<0 && cl_on == 0){
			}
			else{
				cl_on = 1;
			}
			/* if a client connected send tracking data to it */
			if(cl_on){
					while(1){
						/* sync to tracker data write */
						if(on_send==1){
						/* send data to client */
						send_tracking_data(cl);
						/* reset sync flag */
						on_send = 0;
						}
					 }
			}
		/* if we don't have input stop */
		if(image==NULL) break;
	}
	/* close socket for remote broadcast */
	close_remote_access(cl, srv);
	return NULL;
}

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
	/* update log file: X,Y,theta,timestamp */
	if(f!=NULL) 
		//fprintf(f,"%f,%f,%f,%s\n", X, Y, theta, insert_timestamp());
		fprintf(f, "%f,%f,%f,%d\n", X, Y, theta, idx);

	if(idx%15==0){
			/* sync to socket send */
			on_send = 1;
			/* lock buffer for writing */
//			pthread_mutex_lock(&buff_lock);
			/*prepare buffer to be sent to remote connections in the second thread */;
			//sprintf(buffer, "%f,%f,%f,%s\n", X, Y, theta, insert_timestamp());
			sprintf(buffer, "%f,%f,%f,%d\n", X, Y, theta, idx);
			/* unlock buffer */
//			pthread_mutex_unlock(&buff_lock);
	}
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
			cvLine(obj_pos_img, cvPoint(x_pos, y_pos), cvPoint(x_pos_ant_vis, y_pos_ant_vis), cvScalar(0,0,255), 1);
		}
	}
	cvAdd(image, obj_pos_img, image);

	/* update history for visualization */
	x_pos_ant_vis = x_pos;
	y_pos_ant_vis = y_pos;
#endif		
	/* timed update history for tracking */
	if(idx%450==0){
		x_pos_ant = x_pos;
		y_pos_ant = y_pos;
		x_pos_ant0 = x_pos0;
		y_pos_ant0 = y_pos0;
	}
	idx++;
  }
}

#endif 
