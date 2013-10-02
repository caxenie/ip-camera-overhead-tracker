/**
 * @author Cristian Axenie, cristian.axenie@tum.de
 * 
 * Camera calibration helper tool.
 * Prints the coordinates of the selected points in camera space for determining
 * the camera-space to world-space mapping.
 */

#include <stdio.h>
#include "highgui.h"
#include "cv.h"


/* position of the current marked calibration point */
int x_pos = 0, y_pos = 0;

/* image that stores the chose calibation markers */
IplImage* markers_img;

/* initialize font and add text and marker position string */
CvFont font;
char pos_val[30];
/* point index */
int i = 0;

/* the video capturing struct init */
CvCapture *capture = NULL;

/* current frame from the source */
IplImage* image = NULL;
/* file to save the selected points coordinates */
FILE *f;

/* flag to mark the use of an image instead of stream */
short is_image;

/**
* Get the stream source local saved file, local cam or remote stream
*/
CvCapture* get_source(CvCapture *c, int nr, char** in)
{
	if(c!=NULL) c = NULL;
	/* check source */
	if(nr == 1 || (nr == 2 && strlen(in[1])==1 && isdigit(in[1][0]))) {
		printf("get_source: capture from local cam\n");
		/* capture from local cam */
		c = cvCaptureFromCAM( nr==2 ? in[1][0] - '0' : 0);
	} else if( ((strstr(in[1],"rtsp")!=NULL) || (strstr(in[1],"http")!=NULL)) && (nr == 2) ) {
		/* capture from remote cam */
		printf("get_source: capture from remote cam\n");
		c = cvCreateFileCapture(in[1]);
	} else if( strstr(in[1],"avi")!=NULL && nr == 2 ) {
		printf("get_source: capture from locally saved file\n");
		/* capture from local sved file */
		c = cvCaptureFromAVI( in[1] );
        }
	return c;
}


/**
 * Mouse handler callback for selecting camera calibration points
 */
void select_point(int event, int x_coord, int y_coord, int flags, void* param)
{
	/* check the type of event that occured and filter desired event */
	if(event == CV_EVENT_LBUTTONDOWN) {
		/* get the coord of the subimage */
		x_pos = x_coord;
		y_pos = y_coord;
		/* setup a rectangular region of interest */
		cvSetImageROI(image,
		              cvRect( x_pos,
		                      y_pos,
		                      10,
		                      10));
		/* increase index */
		i++;
		/* save points coord to file */
		if(f!=NULL) fprintf(f,"pt %d [ %d,%d ]\n", i, x_pos, y_pos);
		/* reset the region of interest */
		cvResetImageROI(image);
	}
}

/**
 * Mark the selected points in the received frames
 */
void mark_points()
{
	/* events are generated also when starting the display but we only consider user's clicks */
	if(i>0){
			/* draw an identification rectangle where a point was selected */
			cvRectangle(markers_img,			/* image to draw in */
			            cvPoint(x_pos-5, 
					    y_pos-5), 			/* one vertex */
			            cvPoint(x_pos+5,			/* opposite vertex */
			                    y_pos+5),
			            cvScalar( 0, 0, 255, 0 ),		/* color */
			            2,					/* lines thickness */
			            0,					/* line type */
			            0					/* number of fractional bits in the point coordinates */
			           );


			/* add a coordinate view */
			sprintf(pos_val, "%d [x=%d - y=%d]",i , x_pos, y_pos);
			cvPutText(markers_img, 				/* the image to write on */  
					  pos_val, 			/* the string to write */
					  cvPoint(x_pos, y_pos),        /* where to place text box */
					  &font, 			/* which font to use */
					  cvScalar(255, 255, 255, 0));  /* line properties */
			/* overimpose over the current frames */
			cvAdd(image,markers_img,image);
	}
} 

/* entry point */
int main(int argc, char* argv[])
{
	/* opent the points file */
	f = fopen("calibration-points.txt","w+");
	/* init font system */
	cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX, 0.5, 0.5, 0.5, 1, 8);

	/* check the type of input */
	if(argc==2 && strstr(argv[1], "jpg")!=NULL) is_image = 1;

	if(is_image==0){
		/* check the source */
		if ((capture = get_source(capture, argc, argv))==NULL) {
			printf("main: cannot open stream!\n");
			return 1;
		}
		/* get an initial frame to get properties */
		image = cvQueryFrame(capture);

	}
	else{
		image = cvLoadImage(argv[1], CV_LOAD_IMAGE_COLOR);
	}

	/* create window as placeholder for captured frames and marking */
	cvNamedWindow("Calibration helper", CV_WINDOW_AUTOSIZE);

	/* create image to store markers */
	markers_img = cvCreateImage(cvSize(image->width, image->height),
	                               image->depth,
	                               image->nChannels);

	/* set callback for the template selector */
	cvSetMouseCallback("Calibration helper", select_point, NULL);

	/* loop until user decides */
	while(1) {
		if(is_image==0){
			/* grab and return a frame from source or loop*/
			image = cvQueryFrame(capture);
		}
		/* object tracking */
		mark_points();

		/* update images */
		cvShowImage("Calibration helper", image);
		
		/* check if quit is sent */
		if((int)(cvWaitKey(2) & 255) == 'q') 
			 break; 
		
	}
	/* free the allocated memory*/
	cvReleaseCapture(&capture);
	cvDestroyWindow("Calibration helper");
	return 0;
}
