#ifndef COLOR_DIFF_TRACKER_H
#define COLOR_DIFF_TRACKER_H


/* include all necessary headers */
#include "global.h"


using namespace std;



/**
 * Struct to hold color values for color difference computation in HSV color space.
**/
typedef struct _ColorValue
{
    double h;
    double s;
    double v;
} ColorValue;



/**
 * The Marker type encapsulates all data describing one marker like position,
 * velocity, color, etc.
 * For each LED a marker is created within the tracker struct
**/
typedef struct _Marker
{
    /* signal flags */
    int pos_is_set;
    int vel_is_set;
    int acc_is_set;
    int roi_set;

    /* The variables that actually hold the marker position, vel and acc in pixel coordinates */
    CvPoint2D32f pos_measured;						//the measured position of the marker
    CvPoint2D32f pos_predicted;						//the predicted position
    CvPoint2D32f vel;								//the estimated velocity
    CvPoint2D32f acc;								//the estimated acceleration
	CvPoint2D32f pos_estimated;						//marker position estimated by kalman filter

    /* The Region of Interest, restrains search to a small area around predicted new marker position */
    CvRect roi;


    /* Image header to hold resulting distance map */
    IplImage *result;


    /* const HSV values for color error computation -> distance map */
    ColorValue colorvalue;

    /* Marker (LED) color */
    int color;


} Marker;


/**
 * Tracker struct:
 * For each camera one tracker is created.
 * Each tracker holds the camera attached to it and a number of marker structs according to the number of LEDs
**/
typedef struct _Tracker
{
    /* unique id for each tracker */
    int idx;

    /* the source to capture from, either REMOTE or DISK */
    int source;

	/* color or b/w camera */
	int color;

    /* header to store frame received either from camera (REMOTE) or from disk (DISK) */
    IplImage *frame;

	/* keep track of how many frames have been received so far */
    uint32_t frame_counter;

    /* the camera connected to the tracker */
    dc1394camera_t *camera;

    /* vector to hold marker structs; each marker struct corresponds to one LED */
    vector<Marker *> marker;

    /* window name for debug output */
    char win_name[16];

	/* tracker state: either on_track or off_track */
    int state;
	

} Tracker;



/**
 * Intialization routines to setup tracker and its markers
 * One tracker per camera is created and each tracker holds one marker struct for each LED
**/
void init_tracker(Tracker *tracker, dc1394camera_t *cam, int id, int num_of_markers, int colors[]);
void init_marker(Marker *marker, int color);


/* Tracking routine, a wrap up routine for calling compute_diff_maps() and locate_marker() */
void track(Tracker *tracker);



/**
 * compute_diff_maps:
 * Computes the differences between the pixel values in the region of interest and the desired color values.
 * Result is stored in a matrix, the difference map.
**/
void compute_color_diff_map(Tracker *tracker);


/**
 * locate_marker:
 * After the distance map is computed by compute_distance_map(), this routine finds the global maximum in the matrix.
 * The weighted sum of array values of an area around global max gives the estimated marker position.
**/
int locate_marker(Tracker *tracker);


int bw_locate_marker(Tracker *tracker);



#endif
