/**
 * @author Felix Ebert, felix.ebert@mytum.de
 *
 *
 * Neural Self-calibrating Robot Tracking System.
 *
 * Main routine of 3D tracking application for indoor operation in the robot lab.
 * Usually the software receives data from the Guppy Firewire cameras, nevertheless it is possible to read data from disk.
 * Furthermore, data from the cams can be recorded to disk as raw images.
 * The resutling pixel coordinates and computed world coordinates are send over a network stream to a remote client.
 * So far, 3D tracking works for two cameras (stereo setup) and up to three markers, 
 * orientation estimation, though, is only available for three markers. 
 */

#include "../../src/global.h"
#include "../../src/ColorDiffTracker.h"
#include "../../src/streamer.h"
#include "../../src/utils.h"


#define ON_TRACK 1
#define OFF_TRACK 0

using namespace std;
using namespace cv;

const int on_track = 1;
const int off_track = 0;
int *cams_on_track = (int *)calloc(4,sizeof(int));

/**
 * Function declarations
**/
/*-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-*/

/* camera initialization, setup and stream transmission starting */
int init_cams(vector<dc1394camera_t *> cameras, dc1394_t *dc1394);


/* tracking routine locates marker and writes pixel coordinates */
int receive_frames_and_track(vector<Tracker *> tracker);

/* reprojection routine computes world coordinates */
void *reproject(void *arg);

/* get new frame from either the cameras or from disk */
int retrieve_new_frame(Tracker *tracker);
/*-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-*/



struct GlobalData *global_data;

/**
 * global variables
**/
/*-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-*/

/* unique identifiers of cameras */
unsigned long cam_guid[] =
{
    2892819639783898, 	// l0 (color)
    2892819639783895, 	// l1 (color)
    2892819639783896,	// l2 (color)
    2892819639783897,	// l3 (color)
    2892819639812243,	// h0 (b/w)
    2892819639812241, 	// h1 (b/w)
    2892819639812152,	// h2 (b/w)
    2892819639812244	// h3 (b/w)
};


//mutual exclusion to prevent simultaneous access to global struct from multiple threads
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;

//Global struct to hold marker positions in pixel and in world coordinates
struct _MarkerPosition *markerPosition;

int exit_state = 0;


/**
 * Entry point
**/
int main(int argc, char *argv[])
{

    init_data(argc,argv);
    markerPosition->buffer = (char *)calloc(global_data->MAX_BUF,sizeof(char));


    /* init dc1394 API */
    dc1394_t *dc1394 = dc1394_new();
    dc1394camera_list_t *camera_list;

    int cam_count = global_data->NUM_OF_CAMS;

    if(global_data->capture_source==REMOTE)
    {
        /* list all cameras found */
        dc1394error_t err = dc1394_camera_enumerate(dc1394, &camera_list);
        unsigned int found_cam_count = camera_list->num;
        DC1394_ERR_RTN(err, "Failed to enumerate cameras");

        if(found_cam_count == 0)
        {
            cerr << "No cameras found" << endl;
            exit(1);
        }

    }
   

    vector<dc1394camera_t *> cameras(cam_count);
    vector<dc1394featureset_t> features(cam_count);
    vector<dc1394format7modeset_t> modesets(cam_count);

    int cam_num;
    if(global_data->capture_source==REMOTE)
    {
        dc1394camera_t *camera;
        for(int i=0; i<cam_count; i++)
        {
            if(i==0)
            {
                cam_num = 6;
            }
            else if(i==1)
            {
                cam_num = 3;
            }
            camera = dc1394_camera_new(dc1394, cam_guid[cam_num] /*camera_list->ids[i].guid*/);
            if(!camera)
            {
                cerr << "Failed to initialize cam " << i << endl;
                exit(1);
            }
            cameras[i] = camera;
            // get camera features
            if(dc1394_feature_get_all(camera, &features[i]) != DC1394_SUCCESS)
            {
                cerr << "Could not get camera " << i << "s feature information!" << endl;
            }
            else
            {
                // print all features the camera supports
                //dc1394_feature_print_all(&features[i], stdout);
            }
            // check format 7 capabilities
            if(dc1394_format7_get_modeset(camera, &modesets[i]) != DC1394_SUCCESS)
            {
                cerr << "Could not query Format_7 informations on cam " << i << endl;
            }
        }
        dc1394_camera_free_list(camera_list);

        // set up cameras
        set_camera_params(cameras,dc1394);
        // start transmission
        start_camera_transmission(cameras,dc1394);
    }
    else if(global_data->capture_source==DISK)
    {
        for(int i=0; i<cam_count; ++i)
        {
            cameras[i] = NULL;
        }
    }

    // create one tracker for each camera connected
    vector<Tracker *> tracker(cam_count);
    //init tracker
    for(int i=0; i<cam_count; ++i)
    {
        tracker[i] = new Tracker;
        init_tracker(tracker[i],cameras[i],i,global_data->NUM_OF_MARKERS,global_data->colors);
    }


    //allocate memory for global storage of coordinates
    markerPosition = (struct _MarkerPosition *)calloc(1, sizeof(struct _MarkerPosition));




    //start streaming and reprojection in separate threads
    pthread_t reproject_thr;

    int rc;
    if((rc = pthread_create(&reproject_thr,NULL,reproject,NULL))!=0)
    {
        printf("Error starting reprojection thread: %d\n",rc);
        exit(-1);
    }


    /* start streaming server in separate thread */
    pthread_t conn_handler;
    if(pthread_create(&conn_handler, NULL, remote_connections_handler, NULL)!=0)
    {
        printf("Error starting streaming thread\n");
        exit(-1);
    }

	/* everything is set up, start the actual tracking procedure */
    receive_frames_and_track(tracker);

	/* threads do not close properly when exit flag is set */
//	pthread_join(reproject_thr,NULL);
//	pthread_join(conn_handler,NULL);

    if(global_data->capture_source==REMOTE)
    {
        cleanup_and_exit(dc1394, cameras);
    }

    return 0;
}

int receive_frames_and_track(vector<Tracker *> tracker)
{

    int frame_counter = 0;
    int cam_count = tracker.size();

    CvFont font;
    cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX,0.7,0.7,0.0,1,8);


	/* time measurement variables for fps computation (DEBUG output) */
    timeval start, end;
    gettimeofday(&start, 0);
    timeval fps_tick = start;
    int fps_frame_count = 0;

		
	printf("%f ",tracker[0]->Fundamental[0][0]);
	printf("%f ",tracker[0]->Fundamental[0][1]);
	printf("%f \n",tracker[0]->Fundamental[0][2]);
	printf("%f ",tracker[0]->Fundamental[1][0]);
	printf("%f ",tracker[0]->Fundamental[1][1]);
	printf("%f \n",tracker[0]->Fundamental[1][2]);
	printf("%f ",tracker[0]->Fundamental[2][0]);
	printf("%f ",tracker[0]->Fundamental[2][1]);
	printf("%f \n",tracker[0]->Fundamental[2][2]);

    char key = '\0';

    //main processing loop: grab images and process
    //loop until user breaks or no new frame is acquired
    int rc = 0;

    struct timeval dcvdns_start, dcvdns_end;
    double dt;

    while(key != 'q')
    {
        gettimeofday(&dcvdns_start,0);

        /* read out new frames for all cameras */
        for(int cc=0; cc<cam_count; ++cc)
        {
            rc = retrieve_new_frame(tracker[cc]);
            if(rc<0)
            {
                break;
            }
        }
        /* then track */
        for(int cc=0; cc<cam_count; ++cc)
        {
			/* find new marker location */
            track(tracker,cc);

			/* if verbosity is set, create debug output */
            if(global_data->verbose)
            {
				/* rendering images is time consuming, to reduce processor load do this only every nth frame */
                if(tracker[cc]->frame_counter%5==0)
                {
                    for(int nm=0; nm<global_data->NUM_OF_MARKERS; ++nm)
                    {
                        if(tracker[cc]->marker[nm]->pos_is_set)
                        {
                            if(tracker[cc]->color)
                            {
							/* plot one rectangle for each found marker with the size corresponding to the ROI size */
                            cvRectangle(
                                tracker[cc]->frame,
                                cvPoint(tracker[cc]->marker[nm]->pos_measured.x-tracker[cc]->marker[nm]->roi.width,
                                        tracker[cc]->marker[nm]->pos_measured.y-tracker[cc]->marker[nm]->roi.height),
                                cvPoint(tracker[cc]->marker[nm]->pos_measured.x+tracker[cc]->marker[nm]->roi.width,
                                        tracker[cc]->marker[nm]->pos_measured.y+tracker[cc]->marker[nm]->roi.height),
                                CV_RGB(255,255,0),1,CV_AA);
                            }
                            else
                            {
                                if(nm==0)
                                {
                                    cvCircle(tracker[cc]->frame,cvPoint(tracker[cc]->marker[0]->pos_measured.x,tracker[cc]->marker[0]->pos_measured.y),8,cvScalar(255,255,255),1,CV_AA);
                                }
                                else
                                {
                                    cvRectangle(tracker[cc]->frame,
                                        cvPoint(tracker[cc]->marker[nm]->pos_measured.x-5,
                                            tracker[cc]->marker[nm]->pos_measured.y-5),
                                        cvPoint(tracker[cc]->marker[nm]->pos_measured.x+5,
                                            tracker[cc]->marker[nm]->pos_measured.y+5),
                                        CV_RGB(255,255,255),1,CV_AA);
                                }
                            }
                        }
                    }

                }
            }


        }

        /* lock mutex to write to global struct */
        pthread_mutex_lock(&mutex);
        /* write pixel coordinates of all markers */
        if(tracker[0]->state==ON_TRACK && tracker[1]->state==ON_TRACK)
        {
            for(int nm=0; nm<global_data->NUM_OF_MARKERS; ++nm)
            {
                
                markerPosition->pxl_coordinates[6][nm].x = tracker[0]->marker[nm]->pos_measured.x;
                markerPosition->pxl_coordinates[6][nm].y = tracker[0]->marker[nm]->pos_measured.y;
                markerPosition->pxl_coordinates[3][nm].x = tracker[1]->marker[nm]->pos_measured.x;
                markerPosition->pxl_coordinates[3][nm].y = tracker[1]->marker[nm]->pos_measured.y;
                
            }
            markerPosition->allowed_to_read = 1;
        }
        /* synchronise threads, this flag indicates that new pixel coordinates have been computed and the reprojection thread is allowed to access */
        /* unlock mutex so that reprojection thread can read out data */
        pthread_mutex_unlock(&mutex);





		/* if verbosity is set, print fps rate to the console */
        if(global_data->verbose)
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
        }


		/* when debug output is created, cvWaitKey() returns control to the OS to render the images */
        if(global_data->verbose)
        {
            if(tracker[0]->frame_counter%5==0)
            {
                /* render the frames */
                cvShowImage(tracker[0]->win_name,tracker[0]->frame);
                cvShowImage(tracker[1]->win_name,tracker[1]->frame);
            }
            key = cvWaitKey(2);
        }

        frame_counter++;

        gettimeofday(&dcvdns_end,0);

        dt = (double)(dcvdns_end.tv_sec - dcvdns_start.tv_sec)*1000.0 + (double)(dcvdns_end.tv_usec - dcvdns_start.tv_usec)/1000.0;
        printf("main: %f ms\n",dt);

    }

    exit_state = 1;

    return 0;
}


/**
 * Reprojection thread: 
 * Computes Cartesian world coordinates when image pixel coordinates have been computed 
 * The trained networks and rotation matrices are loaded from disk for neural reprojection.
**/
void *reproject(void *arg)
{
    /* create Kalman filter for each marker */
	vector<KalmanFilter *> kalman = init_kalman_filter(global_data->NUM_OF_MARKERS);

    /* create stereo cluster for cam pair */
    StereoCluster *cluster = (StereoCluster *)calloc(1,sizeof(StereoCluster));
    cluster->ann = fann_create_from_file("cam_tracker.net");
    if(!cluster->ann)
    {
        fprintf(stderr,"Error intializing stereo cluster\n");
        exit(EXIT_FAILURE);
    }

    char RotMatName[32];
    sprintf(RotMatName,"RotationMatrix.txt");

    fstream fd_rotation_matrix(RotMatName,ios::in);
    /* if no rotation matrix is available, set to identity matrix */
    if(!fd_rotation_matrix.good())
    {
        cout << "No Rotation Matrix available" << endl;
        for(int i=0; i<3; ++i)
        {
            for(int j=0; j<3; ++j)
            {
                cluster->RotationMatrix[i][j] = 0;
                if(i==j) cluster->RotationMatrix[i][j] = 1;
            }
        }
    }
    /* otherwise read rotation matrix from file */
    else
    {
        for(int i=0; i<3; ++i)
        {
            for(int j=0; j<3; ++j)
            {
                fd_rotation_matrix >> cluster->RotationMatrix[i][j];
            }
        }
    }



    /* time measurement variables */
    struct timeval start;
    struct timeval current;
	/* timestamp is computed with respect to this starting point */
    gettimeofday(&start,0);

	

    /* marker world coordinates */
    CvPoint3D32f *marker = (CvPoint3D32f *) calloc(global_data->NUM_OF_MARKERS,sizeof(CvPoint3D32f));
    /* rotated marker cosy for orientation computation */

	/* cartesian coordinates are computed with respect to the initial location of the red marker */
    CvPoint3D32f r_init;
    r_init.x = 0;
    r_init.y = 0;
    r_init.z = 0;
	/* flag to check whether the initial coordinates have been set or not */
	int init = 0;


	/* RPY angles */
    double alpha;
    double beta;
    double gamma;

    /* temporary storage for marker pixel coordinates */
    CvPoint2D32f **marker_pxl_coord = (CvPoint2D32f **) calloc(global_data->NUM_OF_MARKERS,sizeof(CvPoint2D32f *));
    for(int i=0; i<global_data->NUM_OF_MARKERS; ++i)
    {
        marker_pxl_coord[i] = (CvPoint2D32f *) calloc(2,sizeof(CvPoint2D32f));
    }   
    CvPoint3D32f *marker_world_coord = (CvPoint3D32f *) calloc(global_data->NUM_OF_MARKERS,sizeof(CvPoint3D32f));


    Mat_<float> measurement(3,1); 
    /* reset measurements to zero */
    measurement.setTo(Scalar(0));
	double d;
    int counter = 0;
	/* enter infinite loop */

    struct timeval dcvdns_start, dcvdns_end;
    double dt;
    while(1)
    {
        gettimeofday(&dcvdns_start,0);

		if(exit_state) break;

        /* lock mutex to safely read out data */
        pthread_mutex_lock(&mutex);
        if(markerPosition->allowed_to_read == 0)
        {
            /* unlock mutex again */
            pthread_mutex_unlock(&mutex);
            /* continue as long as not allowed to read */
            continue;
        }
        for(int nm=0; nm<global_data->NUM_OF_MARKERS; ++nm)
        {
            /* read out and normalize image pixel coordinates between -1 and 1 */
            marker_pxl_coord[nm][0].x = 2.0*(markerPosition->pxl_coordinates[6][nm].x/(double)FRAME_WIDTH) - 1.0;
            marker_pxl_coord[nm][0].y = 2.0*(markerPosition->pxl_coordinates[6][nm].y/(double)FRAME_HEIGHT) - 1.0;
            marker_pxl_coord[nm][1].x = 2.0*(markerPosition->pxl_coordinates[3][nm].x/(double)FRAME_WIDTH) - 1.0;
            marker_pxl_coord[nm][1].y = 2.0*(markerPosition->pxl_coordinates[3][nm].y/(double)FRAME_HEIGHT) - 1.0;

        }

        /**
         * If positions have been read, set flag to wait until new position estimates from tracking algorithm arrive.
         * Backprojection thread runs much faster than tracking threads, thus to avoid computing the same values all the time
         * this thread waits until new measurements from the tracking thread is available
        **/
        markerPosition->allowed_to_read = 0;
        /* unlock buffer so that tracking algorithm can write new coordinates */
        pthread_mutex_unlock(&mutex);

        if(!init)
        {
            compute_rotated_net_output(cluster, marker_pxl_coord[0], &marker_world_coord[0]);

            r_init.x += marker_world_coord[0].x;
            r_init.y += marker_world_coord[0].y;
            r_init.z += marker_world_coord[0].z;

            counter++;
            if(counter==10)
            {
                r_init.x /= 10.0;
                r_init.y /= 10.0;
                r_init.z /= 10.0;
                init = 1;
            }
            continue;
        }

        /* compute Cartesian world coordinates of marker */
        for(int nm=0; nm<global_data->NUM_OF_MARKERS; ++nm)
        {
            compute_rotated_net_output(cluster, marker_pxl_coord[nm], &marker_world_coord[nm]);


            /* compute corrected world coordinates by translating the network output */
            marker[nm].x = marker_world_coord[nm].x - r_init.x;
            marker[nm].y = marker_world_coord[nm].y - r_init.y;
            marker[nm].z = marker_world_coord[nm].z - r_init.z;

            /* Run Kalman filter */
            Mat prediction = kalman[nm]->predict();

            measurement(0) = marker[nm].x;
            measurement(1) = marker[nm].y;
            measurement(2) = marker[nm].z;

            Mat estimated = kalman[nm]->correct(measurement);

            marker[nm].x = estimated.at<float>(0);
            marker[nm].y = estimated.at<float>(1);
            marker[nm].z = estimated.at<float>(2);
 
        }
        d = sqrt(marker[0].x*marker[0].x + marker[0].y*marker[0].y + marker[0].z*marker[0].x);
//       printf("%f\n",d*100.0);

//        printf("%f %f %f\n",marker[0].x*100.0,marker[0].y*100.0,marker[0].z*100.0);

        /**
		 * Determine RPY angels; only makes sense for three markers.
		 * With three markers in a plane two perpendicular vectors (x and y) can be defined, 
		 * the third (z) corresponds to the crossproduct of x and y. 
         * Then the orientation in terms of roll, pitch and yaw of this coordinate system is computed.
		**/
        if(global_data->NUM_OF_MARKERS==3)
        {
            compute_RPY_angles(marker,&alpha, &beta, &gamma);
        }

//      printf("%s\n",track_data);
        /* get current time for timestamp */
        gettimeofday(&current,0);

		/* lock buffer to safely write to global struct */
        pthread_mutex_lock(&mutex);

        /* write timestamp and world coordinates to global struct */
        markerPosition->timestamp = (current.tv_sec - start.tv_sec)*1000 + (current.tv_usec - start.tv_usec)/1000;

        for(int nm=0; nm<global_data->NUM_OF_MARKERS; ++nm)
        {
            markerPosition->world_coordinates[nm].x = marker[nm].x;
            markerPosition->world_coordinates[nm].y = marker[nm].y;
            markerPosition->world_coordinates[nm].z = marker[nm].z;
        }
#if 0
        /* fill streaming buffer with computed values for timestamp, pxl coordinates and world coordinates */
        if(global_data->NUM_OF_MARKERS==3)
        {
            sprintf(markerPosition->buffer,"%6ld %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f %.4f\n",
                markerPosition->timestamp,
                marker[0].x*100.0, marker[0].y*100.0, marker[0].z*100.0,
                marker[1].x*100.0, marker[1].y*100.0, marker[1].z*100.0,
                marker[2].x*100.0, marker[2].y*100.0, marker[1].z*100.0,
                alpha*180.0/PI, beta*180.0/PI, gamma*180.0/PI
               );
        }
        else
        {
            sprintf(markerPosition->buffer,"%6ld ", markerPosition->timestamp);
            for(int i=0; i<global_data->NUM_OF_MARKERS; ++i)
            {
                sprintf(markerPosition->buffer,"%.4f %.4f %.4f ",
                        marker[i].x*100.0, marker[i].y*100.0, marker[i].z*100.0
                        );
            }
            sprintf(markerPosition->buffer,"\n");
        }
        printf("%s\n",markerPosition->buffer);
#endif
        /* set flag that streaming server is allowed to send buffer out */
        markerPosition->allowed_to_send = 1;
        
        /* and most important, unlock mutex again */
        pthread_mutex_unlock(&mutex);

        //printf("%d\n",exit_state);
        if(exit_state) break;

        gettimeofday(&dcvdns_end,0);

        dt = (double)(dcvdns_end.tv_sec - dcvdns_start.tv_sec)*1000.0 + (double)(dcvdns_end.tv_usec - dcvdns_start.tv_usec)/1000.0;
        printf("reproject: %f ms\n",dt);
	
    }

    pthread_exit(NULL);

}




/**
 * Frame collecting routine:
 * Each tracker calls this function to get a new frame.
**/
int retrieve_new_frame(Tracker *tracker)
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
        if(global_data->is_recording)
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
        snprintf(img_name,63,"%sdc1394_cam%d_%05d.pgm",global_data->img_base_path,tracker->idx,tracker->frame_counter);
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

