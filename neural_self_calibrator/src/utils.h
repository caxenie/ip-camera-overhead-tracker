#ifndef UTILS_H
#define UTILS_H

#include "global.h"


StereoCluster **init_stereo_cluster(int argc, char *argv[]);

void init_data(int argc, char *argv[], struct StaticData *data);
void read_config(struct StaticData *data);


/* retrieve command line options */
void parse_command_line(int argc, char *argv[], struct StaticData *data);


int set_camera_params(dc1394camera_t *cameras,struct StaticData *data);
int start_camera_transmission(dc1394camera_t *cameras);


/**
 * Intialization routines to setup tracker and its markers
 * One tracker per camera is created and each tracker holds one marker struct for each LED
**/
int init_tracker(Tracker *tracker, dc1394camera_t *cam, struct StaticData *data);

void init_marker(Marker *marker, int color);


/* camera initialization, setup and stream transmission starting */
int init_cams(std::vector<dc1394camera_t *> cameras, dc1394_t *dc1394);





/* clean up dc1394 api */
void cleanup_and_exit(dc1394_t *dc1394, std::vector<dc1394camera_t *>& cameras);

void sig_handler(int signal);





void init_global_data();


struct _MarkerPosition* init_marker_position(struct StaticData *data);





/* print general help information */
void show_help();


void compute_net_coordinates(StereoCluster *cluster);


std::vector<cv::KalmanFilter *> init_kalman_filter(int N);

void compute_RPY_angles(CvPoint3D32f *marker, double *alpha, double *beta, double *gamma);

void compute_epipolar_line(CvPoint2D32f *marker, double **Fundamental, double *a, double *c);

void relate_blobs(std::vector<Tracker *> tracker, double **Fundamental);




void print_data(struct StaticData *data);




#endif
