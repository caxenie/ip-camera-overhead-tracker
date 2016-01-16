#ifndef COLOR_DIFF_TRACKER_H
#define COLOR_DIFF_TRACKER_H


/* include all necessary headers */
#include "global.h"


using namespace std;



/* Tracking routine, a wrap up routine for calling compute_diff_maps() and locate_marker() */
void track(StereoCluster *cluster);



/**
 * compute_diff_maps:
 * Computes the differences between the pixel values in the region of interest and the desired color values.
 * Result is stored in a matrix, the difference map.
**/
void compute_color_diff_map(Tracker *tracker, struct StaticData *data);


/**
 * locate_marker:
 * After the distance map is computed by compute_distance_map(), this routine finds the global maximum in the matrix.
 * The weighted sum of array values of an area around global max gives the estimated marker position.
**/
int locate_marker(Tracker *tracker, struct StaticData *data);


/* thresholding and blob extraction */
int bw_detect_blobs(Tracker *tracker, struct StaticData *data);

/* blob extraction */
void blob_classification(StereoCluster *cluster);
/* track blobs based on velocity model */
void bw_track_blobs(StereoCluster *cluster);


#endif
