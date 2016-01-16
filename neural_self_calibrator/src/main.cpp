/**
 * @author Felix Ebert, felix.ebert@mytum.de
 *
 * Neural Self-Calibrating Robot Tracking System.
 *
 * Main routine of 3D tracking application for indoor operation in the robot lab.
 * Usually the software receives data from the Guppy Firewire cameras, nevertheless it is possible to read data from disk.
 * Furthermore, data from the cams can be recorded to disk as raw images (pgm files).
 * The resulting pixel coordinates and computed world coordinates are send over a network stream to a remote client.
 * So far, 3D tracking works for two cameras (stereo setup) and up to three markers,
 * orientation estimation, though, is only available for three markers.
 */

#include "ColorDiffTracker.h"
#include "streamer.h"
#include "utils.h"


using namespace std;
using namespace cv;


//#define CALIBRATION


/**
 * Function declarations
**/
/*-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-*/
/* tracking routine locates marker and writes pixel coordinates */
void *receive_frames_and_track(void *arg);

/* reprojection routine computes world coordinates */
void *reproject(void *arg);

/* get new frame from either the cameras or from disk */
int retrieve_new_frame(Tracker *tracker, struct StaticData *data);


/*-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-*/



/**
 * global variables
**/
/*-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-*/

//mutual exclusion to prevent simultaneous access to global struct from multiple threads
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t render_mutex = PTHREAD_MUTEX_INITIALIZER;

/* Global struct to hold marker positions in pixel and in world coordinates and streaming buffer */
struct _MarkerPosition *marker_position = NULL;


int exit_state = 0;

/* thread synchronization variables */
int num_of_clusters_on_track = 0;
int thread_counter = 0;
int *cluster_on_track;

FILE *net_log[2];
/*-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-*/



/**
 * Entry point
**/
int main(int argc, char *argv[])
{

    net_log[0] = fopen("net_log_0.txt","w");
    net_log[1] = fopen("net_log_1.txt","w");

    int sc;

    StereoCluster **cluster = NULL;
    cluster = init_stereo_cluster(argc, argv);
    if(!cluster)
    {
        fprintf(stderr,"Error initializing stereo clusters\n");
        exit(EXIT_FAILURE);
    }
    else
    {
        sc = cluster[0]->data->NUM_OF_CAM_PAIRS;
        printf("%d stereo cluster intialized\n",sc);
    }

    cluster_on_track = (int *)calloc(sc,sizeof(int));

    marker_position = init_marker_position(cluster[0]->data);
    if(!marker_position)
    {
        fprintf(stderr,"global position data struct can not be initialzed\n");
        exit(EXIT_FAILURE);
    }


    int rc;

    /* start streaming and reprojection in separate threads */
    pthread_t conn_handler;
    pthread_t *tracking_thr = (pthread_t *)calloc(sc,sizeof(pthread_t));;
    pthread_t rep;

	/* launch one tracking thread for each stereo cluster */
    for(int i=0; i<sc; ++i)
    {
        rc = pthread_create(&tracking_thr[i],NULL,receive_frames_and_track,(void *)cluster[i]);
    }

	
    if((rc = pthread_create(&rep,NULL,reproject,(void *)cluster[0]->data))!=0)
    {
        printf("Error starting reprojection thread: %d\n",rc);
        exit(EXIT_FAILURE);
    }

	/* start streaming thread */
    if((rc = pthread_create(&conn_handler,NULL,remote_connections_handler,(void *)cluster[0]->data))!=0)
    {
        printf("Error starting streaming thread: %d\n",rc);
        exit(EXIT_FAILURE);
    }




	/* wait for all threads to close down */
    for(int i=0; i<sc; ++i)
    {
        pthread_join(tracking_thr[i],NULL);
    }
    pthread_join(conn_handler,NULL);
    pthread_join(rep,NULL);


    return 0;
}




void *receive_frames_and_track(void *arg)
{
    StereoCluster *cluster = (StereoCluster *) arg;

    FILE *vector_log = fopen("vector_log.txt","w");
    if(!vector_log)
    {
        fprintf(stderr,"Unable to open log file to store coordinate system\n");
        exit(EXIT_FAILURE);
    }


    int frame_counter = 0;

    /* time measurement variables for fps computation (DEBUG output) */
    timeval start, end;
    gettimeofday(&start, 0);
    timeval fps_tick = start;
    int fps_frame_count = 0;

    char key = '\0';

    //main processing loop: grab images and process
    //loop until user breaks or no new frame is acquired
    int rc = 0;

    while(key != 'q')
    {
		/* collect new frames */
        rc = retrieve_new_frame(cluster->tracker[0],cluster->data);
        if(rc<0)
        {
            break;
        }
        rc = retrieve_new_frame(cluster->tracker[1], cluster->data);
        if(rc<0)
        {
            break;
        }


        /* then track, i.e. find new image pixel coordinates of markers */
		/* tracker->state = {ON_TRACK, OFF_TRACK} indicates if all markers have been found or not respectively */
        track(cluster);

        if(cluster->state==ON_TRACK)
        {

            compute_net_coordinates(cluster);

            pthread_mutex_lock(&mutex);

            marker_position->on_track[cluster->id] = 1;
            thread_counter++;

            for(int p=0; p<cluster[0].data->NUM_OF_MARKERS; ++p)
            {
                /* store pixel coordinates in global data sstruct */
                marker_position->cluster_coordinates[cluster->id]->pxl_coordinates_0[p].x = cluster->tracker[0]->marker[p]->pos_measured.x;
                marker_position->cluster_coordinates[cluster->id]->pxl_coordinates_0[p].y = cluster->tracker[0]->marker[p]->pos_measured.y;
                marker_position->cluster_coordinates[cluster->id]->pxl_coordinates_1[p].x = cluster->tracker[1]->marker[p]->pos_measured.x;
                marker_position->cluster_coordinates[cluster->id]->pxl_coordinates_1[p].y = cluster->tracker[1]->marker[p]->pos_measured.y;

                /* store network coordinates in global data struct */
                marker_position->cluster_coordinates[cluster->id]->net_coordinates[p].x = cluster->net_coordinates[p].x;
                marker_position->cluster_coordinates[cluster->id]->net_coordinates[p].y = cluster->net_coordinates[p].y;
                marker_position->cluster_coordinates[cluster->id]->net_coordinates[p].z = cluster->net_coordinates[p].z;

                fprintf(cluster->pxl_log_cam0,"%f %f ",cluster->tracker[0]->marker[p]->pos_measured.x,cluster->tracker[0]->marker[p]->pos_measured.y);
                fprintf(cluster->pxl_log_cam1,"%f %f ",cluster->tracker[1]->marker[p]->pos_measured.x,cluster->tracker[1]->marker[p]->pos_measured.y);
                fprintf(cluster->net_log,"%f %f %f\n",marker_position->cluster_coordinates[cluster->id]->net_coordinates[p].x,marker_position->cluster_coordinates[cluster->id]->net_coordinates[p].y,marker_position->cluster_coordinates[cluster->id]->net_coordinates[p].z);

            }
            fprintf(cluster->pxl_log_cam0,"\n");
            fprintf(cluster->pxl_log_cam1,"\n");

            /* thread synchronization */
            if(thread_counter>=num_of_clusters_on_track)
            {
                /* reset thread counter */
                thread_counter = 0;
                /* set flag to allow read out */
                marker_position->allowed_to_read = 1;
            }

            int k = 0;
            for(int j=0; j<cluster[0].data->NUM_OF_CAM_PAIRS; ++j) k += marker_position->on_track[j];

            if(num_of_clusters_on_track!=k)
            {
                num_of_clusters_on_track = k;
            }


            pthread_mutex_unlock(&mutex);
        }
        else
        {
            marker_position->on_track[cluster->id] = 0;
        }

        /* if verbosity is set, print fps rate to the console */
        if(cluster[0].data->verbose)
        {
            //fps measurements
            gettimeofday(&end, 0);
            double time = ((end.tv_sec - fps_tick.tv_sec) + (end.tv_usec - fps_tick.tv_usec) / 1000000.0);
            if(time > 10.0)
            {
                fps_tick = end;
                double fps = (double)(frame_counter - fps_frame_count) / time;
                fps_frame_count = frame_counter;
                cout << fps << " fps" << endl;
            }

            if(cluster->state==ON_TRACK)
            {
                for(int p=0; p<cluster->data->NUM_OF_MARKERS; ++p)
                {
                    if(p==0)
                    {
                        cvCircle(cluster->tracker[1]->frame,cvPoint((int)cluster->tracker[1]->marker[p]->pos_measured.x,(int)cluster->tracker[1]->marker[p]->pos_measured.y),15,cvScalar(255,255,255),1,CV_AA);
                    }
                    cvCircle(cluster->tracker[0]->frame,cvPoint((int)cluster->tracker[0]->marker[p]->pos_measured.x,(int)cluster->tracker[0]->marker[p]->pos_measured.y),12,cvScalar(255,255,0),1,CV_AA);
                    cvCircle(cluster->tracker[1]->frame,cvPoint((int)cluster->tracker[1]->marker[p]->pos_measured.x,(int)cluster->tracker[1]->marker[p]->pos_measured.y),12,cvScalar(255,255,255),1,CV_AA);
                }
            }
        }


        if(frame_counter%10==0 && cluster->data->verbose)
        {
            pthread_mutex_lock(&render_mutex);
            cvShowImage(cluster->tracker[0]->win_name,cluster->tracker[0]->frame);
            cvShowImage(cluster->tracker[1]->win_name,cluster->tracker[1]->frame);
            pthread_mutex_unlock(&render_mutex);
            key = cvWaitKey(2);


            if(cluster->data->NUM_OF_MARKERS==2 && key=='s')
            {
                double ex = marker_position->world_coordinates[1].x - marker_position->world_coordinates[0].x;
                double ey = marker_position->world_coordinates[1].y - marker_position->world_coordinates[0].y;
                double ez = marker_position->world_coordinates[1].z - marker_position->world_coordinates[0].z;
                fprintf(vector_log,"%f %f %f\n",ex,ey,ez);
                printf("%f %f %f\n",ex,ey,ez);
            }
        }


        frame_counter++;


        /* if SIGINT has been signalled, break loop */
        if(exit_state==1) break;

    }

    fclose(vector_log);


    exit_state = 1;
    return 0;
}


#ifdef CALIBRATION

void *reproject(void *arg)
{
    StaticData *data = (StaticData *)arg;

    while(1)
    {

        if(exit_state) break;


        /* lock mutex to safely read out data */
        pthread_mutex_lock(&mutex);
        if(marker_position->allowed_to_read == 0)
        {
            /* unlock mutex again */
            pthread_mutex_unlock(&mutex);
            /* continue as long as not allowed to read */
            continue;
        }


        printf("on track: %d\n",num_of_clusters_on_track);
        if(num_of_clusters_on_track == 2)
        {
            for(int nm=0; nm<data->NUM_OF_MARKERS; ++nm)
            {
                fprintf(net_log[0],"%f %f %f\n",    marker_position->cluster_coordinates[0]->net_coordinates[nm].x,
                                                    marker_position->cluster_coordinates[0]->net_coordinates[nm].y,
                                                    marker_position->cluster_coordinates[0]->net_coordinates[nm].z);
                fprintf(net_log[1],"%f %f %f\n",    marker_position->cluster_coordinates[1]->net_coordinates[nm].x,
                                                    marker_position->cluster_coordinates[1]->net_coordinates[nm].y,
                                                    marker_position->cluster_coordinates[1]->net_coordinates[nm].z);
            }
        }
        marker_position->allowed_to_read = 0;
        pthread_mutex_unlock(&mutex);

    }
}

#else

/**
 * Reprojection thread:
 * Computes Cartesian world coordinates when image pixel coordinates have been computed
 * The trained networks and rotation matrices are loaded from disk for neural reprojection.
**/
void *reproject(void *arg)
{

    struct StaticData *data = (struct StaticData *)arg;

	/**
	 * Apply kalman filter on Cartesian coordinates for noise suppression.
	 * Create one filter instance for each expected marker.
	**/
	vector<KalmanFilter *> kalman = init_kalman_filter(data->NUM_OF_MARKERS);
	/* Measurement vector: obviously three elements, one for each Cartesian dimension */
	Mat_<float> measurement(3,1); 
	/* reset measurements to zero */
	measurement.setTo(Scalar(0));

    double dt;


    /* time measurement variables */
    struct timeval start;
    struct timeval current;
    /* timestamp is computed with respect to this starting point */
    gettimeofday(&start,0);



    /* marker world coordinates */
    CvPoint3D32f *marker = (CvPoint3D32f *) calloc(data->NUM_OF_MARKERS,sizeof(CvPoint3D32f));
    /* rotated marker cosy for orientation computation */
    CvPoint3D32f cosy[3];

    /* cartesian coordinates are computed with respect to the initial location of the red marker */
    CvPoint3D32f *r_init = (CvPoint3D32f *) calloc(data->NUM_OF_CAM_PAIRS,sizeof(CvPoint3D32f));
    /* flag to check whether the initial coordinates have been set or not */
    int *init = (int *) calloc(data->NUM_OF_CAM_PAIRS,sizeof(int));
    for(int k=0; k<data->NUM_OF_CAM_PAIRS; ++k) init[k] = 0;

    double abs;


    /* RPY angles */
    double alpha;
    double beta;
    double gamma;


	int global_init = 0;

    char *temp = (char *)calloc(data->MAX_BUF,sizeof(char));

    /* enter infinite loop */
    while(1)
    {

        if(exit_state) break;


        /* lock mutex to safely read out data */
        pthread_mutex_lock(&mutex);
        if(marker_position->allowed_to_read == 0)
        {
			/* unlock mutex again */
            pthread_mutex_unlock(&mutex);
			/* continue as long as not allowed to read */
            continue;
        }

        int k = 0;
        for(int p=0; p<data->NUM_OF_CAM_PAIRS; ++p) k += marker_position->on_track[p];
        

//      printf("%d stereo clusters on track\n");

        CvPoint3D32f *net = (CvPoint3D32f *)calloc(data->NUM_OF_MARKERS,sizeof(CvPoint3D32f));

        double dt;
        for(int nm=0; nm<data->NUM_OF_MARKERS; ++nm)
        {
            for(int p=0; p<data->NUM_OF_CAM_PAIRS; ++p)
            {
                if(marker_position->on_track[p])
                {
                    if(!init[p])
                    {
                        r_init[p].x = marker_position->cluster_coordinates[p]->net_coordinates[nm].x;
                        r_init[p].y = marker_position->cluster_coordinates[p]->net_coordinates[nm].y;
                        r_init[p].z = marker_position->cluster_coordinates[p]->net_coordinates[nm].z;
                        init[p] = 1;
                    }
                    net[nm].x += marker_position->cluster_coordinates[p]->net_coordinates[nm].x - r_init[p].x;
                    net[nm].y += marker_position->cluster_coordinates[p]->net_coordinates[nm].y - r_init[p].y;
                    net[nm].z += marker_position->cluster_coordinates[p]->net_coordinates[nm].z - r_init[p].z;
//                    printf("%d: %f %f %f\n",nm,net[nm].x,net[nm].y,net[nm].z);
                    if(nm==8)
                    {
                        dt = sqrt(net[nm].x*net[nm].x + net[nm].y*net[nm].y + net[nm].z*net[nm].z);
                        printf("%d: %f\n",p,dt);
                    }
                    if(nm==0)
                    {
                        printf("%d, %d: %f %f %f\n",nm,p,marker_position->cluster_coordinates[p]->net_coordinates[nm].x,marker_position->cluster_coordinates[p]->net_coordinates[nm].y,marker_position->cluster_coordinates[p]->net_coordinates[nm].z);
                    }
                }
            }
        }
        /**
         * If positions have been read, set flag to wait until new position estimates from tracking algorithm arrive.
         * Backprojection thread runs much faster than tracking threads, thus to avoid computing the same values all the time
         * this thread waits until new measurements from the tracking thread is available
        **/
        marker_position->allowed_to_read = 0;
        /* unlock buffer so that tracking algorithm can write new coordinates */
        pthread_mutex_unlock(&mutex);

        for(int nm=0; nm<data->NUM_OF_MARKERS; ++nm)
        {
            net[nm].x /= k;
            net[nm].y /= k;
            net[nm].z /= k;
//           printf("%d: %f %f %f\n",nm,net[nm].x,net[nm].y,net[nm].z);

            if(!init && nm==0)
            {
                r_init->x = net[0].x;
                r_init->y = net[0].y;
                r_init->z = net[0].z;
                init[0] = 1;
            }

            /* apply Kalman filter */
            Mat prediction = kalman[nm]->predict();

            measurement(0) = net[nm].x - r_init->x;
            measurement(1) = net[nm].y - r_init->y;
            measurement(2) = net[nm].z - r_init->z;

            Mat estimated = kalman[nm]->correct(measurement);

            net[nm].x = estimated.at<float>(0);
            net[nm].y = estimated.at<float>(1);
            net[nm].z = estimated.at<float>(2);
        
            //printf("%d: %f %f %f\n",nm,net[nm].x,net[nm].y,net[nm].z);

            marker_position->world_coordinates[nm].x = net[nm].x;
            marker_position->world_coordinates[nm].y = net[nm].y;
            marker_position->world_coordinates[nm].z = net[nm].z;
        }       





#if 0
        /* compute Cartesian world coordinates of marker */
        for(int nm=0; nm<global_data->NUM_OF_MARKERS; ++nm)
        {
			


            /* set initial red marker position as origin of world cosy */
			/* stereo_cluster is a global struct encapsulating the trained network, rotation matrix and other data for this particular cam_pair */
            if(!init[cam_pair] && !global_init)
            {
                r_init[cam_pair].x = marker_world_coord[nm].x;
                r_init[cam_pair].y = marker_world_coord[nm].y;
                r_init[cam_pair].z = marker_world_coord[nm].z;
	
                /* r_init is only set once at startup */
                init[cam_pair] = 1;
				global_init = 1;
				printf("cam pair %d initialized\n",cam_pair);
            }

            /* compute corrected world coordinates by translating the network output */
            marker[nm].x = marker_world_coord[nm].x - r_init[cam_pair].x;
            marker[nm].y = marker_world_coord[nm].y - r_init[cam_pair].y;
            marker[nm].z = marker_world_coord[nm].z - r_init[cam_pair].z;
			
        }

		/* now check if uninitialized cam pairs overlap with current one */
		for(int cp_it=0; cp_it<global_data->NUM_OF_CAM_PAIRS; ++cp_it)
		{
			if(cp_it==cam_pair) continue;
			
			/* if cam pair is not initialized */
			if(!init[cp_it] && global_init)
			{
				/* check if markers can be seen */
				if(cams_on_track[cp_it]==ON_TRACK && cams_on_track[cp_it+1]==ON_TRACK)
				{
					/* compute translation/offset such that both stereo pairs share common origin */
					
					/* TODO: read out all pixel coordinates at the same time (above), 
					otherwise it is not guaranteed that the values from different cam pairs still correspond to each other */					
					pthread_mutex_lock(&mutex);
					
            		/* read out and normalize image pixel coordinates between -1 and 1 */
            		marker_pxl_coord[0][0].x = 2.0*(markerPosition->pxl_coordinates[cp_it][0].x/(double)FRAME_WIDTH) - 1.0;
            		marker_pxl_coord[0][0].y = 2.0*(markerPosition->pxl_coordinates[cp_it][0].y/(double)FRAME_HEIGHT) - 1.0;
            		marker_pxl_coord[0][1].x = 2.0*(markerPosition->pxl_coordinates[cp_it+1][0].x/(double)FRAME_WIDTH) - 1.0;
            		marker_pxl_coord[0][1].y = 2.0*(markerPosition->pxl_coordinates[cp_it+1][0].y/(double)FRAME_HEIGHT) - 1.0;

        			
					pthread_mutex_unlock(&mutex);

					/* apply to network */ 
					compute_rotated_net_output(stereo_cluster[cp_it], marker_pxl_coord[0], &marker_world_coord[0]);


            		/* set initial red marker position as origin of world cosy */
                	r_init[cp_it].x = marker_world_coord[0].x - marker[0].x;
                	r_init[cp_it].y = marker_world_coord[0].y - marker[0].y;
                	r_init[cp_it].z = marker_world_coord[0].z - marker[0].z;

                	/* r_init is only set once at startup */
                	init[cp_it] = 1;
					printf("cam pair %d initialized\n",cp_it);

//					printf("%f %f %f\n",marker[0].x,marker[0].y,marker[0].z);
//					printf("%f %f %f\n",marker_world_coord[0].x,marker_world_coord[0].y,marker_world_coord[0].z);
//					printf("%f %f %f\n",marker_world_coord[0].x-r_init[cp_it].x,marker_world_coord[0].y-r_init[cp_it].y,marker_world_coord[0].z-r_init[cp_it].z);
            
				}
				
			}
		}
#endif

        /* get current time for timestamp */
        gettimeofday(&current,0);

        /* lock buffer to safely write to global struct */
        pthread_mutex_lock(&mutex);

        /* write timestamp and world coordinates to global struct */
        marker_position->timestamp = (current.tv_sec - start.tv_sec)*1000 + (current.tv_usec - start.tv_usec)/1000;

        memset(marker_position->buffer,'\0',data->MAX_BUF);
        /* fill streaming buffer with computed values for timestamp, pxl coordinates and world coordinates */
		if(data->NUM_OF_MARKERS==3)
		{
        	sprintf(marker_position->buffer,"%6ld %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f\n",
                marker_position->timestamp,
                marker_position->world_coordinates[0].x*100.0, marker_position->world_coordinates[0].y*100.0, marker_position->world_coordinates[0].z*100.0,
                marker_position->world_coordinates[1].x*100.0, marker_position->world_coordinates[1].y*100.0, marker_position->world_coordinates[1].z*100.0,
                marker_position->world_coordinates[2].x*100.0, marker_position->world_coordinates[2].y*100.0, marker_position->world_coordinates[2].z*100.0,
                alpha*180.0/PI, beta*180.0/PI, gamma*180.0/PI
               );
		}
		else
		{
			sprintf(marker_position->buffer,"%6ld ", marker_position->timestamp);
			for(int i=0; i<data->NUM_OF_MARKERS; ++i)
			{
                memset(temp,'\0',data->MAX_BUF);
				sprintf(temp,"%.4f %.4f %.4f ",
                		marker_position->world_coordinates[i].x*100.0, marker_position->world_coordinates[i].y*100.0, marker_position->world_coordinates[i].z*100.0
               			);
                strcat(marker_position->buffer,temp);
			}
            dt = sqrt(  (marker_position->world_coordinates[0].x)*(marker_position->world_coordinates[0].x) + 
                        (marker_position->world_coordinates[0].y)*(marker_position->world_coordinates[0].y) + 
                        (marker_position->world_coordinates[0].z)*(marker_position->world_coordinates[0].z) );
                
            sprintf(temp,"%f\n",dt);
			strcat(marker_position->buffer,temp);
            //printf("%f %f %f\n",marker_position->world_coordinates[0].x*100.0,marker_position->world_coordinates[0].y*100.0,marker_position->world_coordinates[0].z*100.0);
            //printf("%s",marker_position->buffer);
		}

        /* set flag that streaming server is allowed to send buffer out */
        marker_position->allowed_to_send = 1;
		
		/* and most important, unlock mutex again */
        pthread_mutex_unlock(&mutex);


    }

    pthread_exit(NULL);

}
#endif

/**
 * Frame collecting routine:
 * Each tracker calls this function to get a new frame.
**/
int retrieve_new_frame(Tracker *tracker, struct StaticData *data)
{
    IplImage *raw = NULL;

    /* if tracker source is REMOTE, collect new frames from camera */
    if(tracker->source==REMOTE)
    {
        /* create OpenCV image header */
        raw = cvCreateImage(cvSize(FRAME_WIDTH,FRAME_HEIGHT),IPL_DEPTH_8U,1);
        /* dc1394 frame header */
        dc1394video_frame_t *frame;

        /* reserve frame from buffer */
        if(dc1394_capture_dequeue(tracker->camera, /*DC1394_CAPTURE_POLICY_POLL*/DC1394_CAPTURE_POLICY_WAIT, &frame) != DC1394_SUCCESS)
        {
            cerr << "Error while grabbing images from cam " << tracker->idx << endl;
            return -1;
        }
        /* Convert dc1394 frame to IplImage */
        raw->imageData = (char *)(frame->image);
        raw->imageSize = frame->image_bytes;

        /* if recording is active, store raw image on disk */
        if(data->is_recording)
        {
            char filename[64];
            snprintf(filename,63,"../Data/recording/dc1394_cam%d_%05d.pgm",tracker->idx,tracker->frame_counter);
            cvSaveImage(filename,raw);
        }

        if(tracker->color)
        {
            /* convert 8bit bayer masked image to full rgb image (demosaic) */
            cvCvtColor(raw,tracker->frame,CV_BayerBG2BGR);
        }
        else
        {
            cvCopy(raw,tracker->frame);
        }

        /* free frame in buffer */
        if(dc1394_capture_enqueue(tracker->camera, frame) != DC1394_SUCCESS)
        {
            cerr << "Error while releasing frame from cam " << tracker->idx << endl;
            return -1;
        }
    }
    /* if tracker source is DISK, collect new frames from disk */
    else if(tracker->source==DISK)
    {
        char img_name[64];
        snprintf(img_name,63,"%sdc1394_cam%d_%05d.pgm",data->img_base_path,tracker->idx,tracker->frame_counter);
        /* load image from disk */
        raw = cvLoadImage(img_name,CV_LOAD_IMAGE_UNCHANGED);

        /* OpenCV function seems not to work when not called from the main thread */
        /* BUT: selfwritten function is very slow */
//		raw = read_file(img_name);

        if(!raw)
        {
            return -1;
        }

        /* convert 8bit bayer masked image to full rgb image (demosaic) */
        cvCvtColor(raw,tracker->frame,CV_BayerBG2BGR);
    }
    else
    {
        /* if neither REMOTE or DISK is set as source, there clearly went something wrong */
        return -1;
    }

    /* keep track of amount of frames received so far */
    tracker->frame_counter++;

    /* release memory of intermediate image */
    cvReleaseImage(&raw);

    return 0;
}
