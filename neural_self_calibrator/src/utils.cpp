#include "utils.h"

using namespace std;


StereoCluster **init_stereo_cluster(int argc, char *argv[])
{


/******************************************************************************************************/
    /*- Stereo cluster struct initialization -*/
    struct StaticData *data = (struct StaticData *)calloc(1,sizeof(struct StaticData));

    /* init global data and read config */ 
    init_data(argc,argv, data);

    print_data(data);

    StereoCluster **stereo_cluster = (StereoCluster **)calloc(data->NUM_OF_CAM_PAIRS,sizeof(StereoCluster *));
    if(!stereo_cluster)
    {
        fprintf(stderr,"init_stereo_cluster(): Error allocating memory\n");
        exit(EXIT_FAILURE);
    }


    for(int i=0; i<data->NUM_OF_CAM_PAIRS; ++i)
    {
        stereo_cluster[i] = (StereoCluster *)calloc(1,sizeof(StereoCluster));
        stereo_cluster[i]->data = data;

        /* create two trackers for each stereo cluster */
        stereo_cluster[i]->tracker = (Tracker **)calloc(1,sizeof(Tracker *));

        stereo_cluster[i]->tracker[0] = (Tracker *)calloc(1,sizeof(Tracker));
        stereo_cluster[i]->tracker[1] = (Tracker *)calloc(1,sizeof(Tracker));

//      stereo_cluster[i]->id = data->CAM_PAIRS[i];
        stereo_cluster[i]->id = i;
        stereo_cluster[i]->net_coordinates = (CvPoint3D32f *) calloc(stereo_cluster[i]->data->NUM_OF_MARKERS,sizeof(CvPoint3D32f));
    }
/******************************************************************************************************/










/******************************************************************************************************/
    /*- DC1394 Camera Initializations -*/

    vector<dc1394camera_t *> cameras;
    vector<dc1394featureset_t> features(2*data->NUM_OF_CAM_PAIRS);
    vector<dc1394format7modeset_t> modesets(2*data->NUM_OF_CAM_PAIRS);


    if(data->capture_source==REMOTE)
    {
        /* init dc1394 API */
        dc1394_t *dc1394 = dc1394_new();
        dc1394camera_list_t *camera_list;


        /* list all cameras connected */
        dc1394error_t err = dc1394_camera_enumerate(dc1394, &camera_list);
        int found_cam_count = camera_list->num;
        if(err!=0)
        {
            fprintf(stderr,"Camera enumeration failed with dc1394 error code: %d\n",err);
            exit(EXIT_FAILURE);
        }


        if(found_cam_count < data->NUM_OF_CAMS)
        {
            cerr << "Not enough cameras found" << endl;
               exit(1);
        }



        /* set up cameras and start transmission */
        dc1394camera_t *camera;

        for(int i=0; i<data->NUM_OF_CAM_PAIRS; i++)
        {
            int cp[2];
            switch(data->CAM_PAIRS[i])
            {
                case 0:
                    cp[0] = 0;
                    cp[1] = 5;
                    break;
                case 1:
                    cp[0] = 3;
                    cp[1] = 6;
                    break;
                case 2:
                    cp[0] = 1;
                    cp[1] = 4;
                    break;
                case 3:
                    cp[0] = 2;
                    cp[1] = 7;
                    break;
                default:
                    break;
            } 
            printf("%d %d\n",cp[0],cp[1]);

            for(int k=0; k<2; ++k)
            {
                camera = dc1394_camera_new(dc1394, data->cam_guid[cp[k]] /*camera_list->ids[i].guid*/);
                if(!camera)
                {
                    cerr << "Failed to initialize cam " << endl;
                    exit(EXIT_FAILURE);
                }
                cameras.push_back(camera);
            }
            stereo_cluster[i]->cam_idx[0] = cp[0];
            stereo_cluster[i]->cam_idx[1] = cp[1];
        }

        for(int i=0; i<2*data->NUM_OF_CAM_PAIRS; ++i)
        {

            // get camera features
            if(dc1394_feature_get_all(cameras[i], &features[i]) != DC1394_SUCCESS)
            {
                cerr << "Could not get camera " << i << "s feature information!" << endl;
            }
            else
            {
                // print all features the camera supports
                //dc1394_feature_print_all(&features[i], stdout);
            }
            // check format 7 capabilities
            if(dc1394_format7_get_modeset(cameras[i], &modesets[i]) != DC1394_SUCCESS)
            {
                cerr << "Could not query Format_7 informations on cam " << i << endl;
            }

            // set up cameras
            if(set_camera_params(cameras[i], data) < 0)
            {
                fprintf(stderr,"init_stereo_cluster(): error setting camera parameters\n");
                cleanup_and_exit(dc1394,cameras);
            }
            // start transmission
            if(start_camera_transmission(cameras[i]) < 0)
            {
                fprintf(stderr,"init_stereo_cluster(): error starting transmission\n");
                cleanup_and_exit(dc1394,cameras);
            }
        }


        dc1394_camera_free_list(camera_list);
        
    }
    else if(data->capture_source==DISK)
    {
        for(int i=0; i<data->NUM_OF_CAMS; i++)
        {
            cameras[i] = NULL;
        }
    }
/******************************************************************************************************/






/******************************************************************************************************/
    /*- Tracker initialization -*/
    for(int i=0; i<data->NUM_OF_CAM_PAIRS; i++)
    {
        /* first tracker corresponds to color camera */
        init_tracker(stereo_cluster[i]->tracker[0],cameras[2*i],data);
        /* second tracker corresponds to b/w camera */
        init_tracker(stereo_cluster[i]->tracker[1],cameras[2*i+1],data);
    }
/******************************************************************************************************/


//    printf("(%d %d) (%d %d)\n",stereo_cluster[0]->cam_idx[0],stereo_cluster[0]->cam_idx[1],stereo_cluster[1]->tracker[0]->idx,stereo_cluster[1]->tracker[1]->idx);



/******************************************************************************************************/
    for(int sc=0; sc<data->NUM_OF_CAM_PAIRS; ++sc)
    {
        
        char filename[256];
        sprintf(filename,"/home/nst3018/Development/cam_track_new/Tracker/CalibData/fundamental_%d.txt",data->CAM_PAIRS[sc]);
        printf("%d: loading %s\n",sc,filename);
//      printf("%s\n",filename);
        ifstream fun_fd(filename,ios::in);
        if(!fun_fd.good())
        {
            printf("init_stereo_cluster(): Unable to read fundamental matrix\n");
            exit(EXIT_FAILURE);
        }

        for(int row=0; row<3; ++row)
        {
            for(int column=0; column<3; ++column)
            {
                fun_fd >> stereo_cluster[sc]->FundamentalMatrix[column][row];
        
            }
        }
    

        /*- Read transformation matrix -*/
        char MatName[32];
        /* stereo cluster 1 works currently as the reference cluster */
        sprintf(MatName,"/home/nst3018/Development/cam_track_new/Tracker/CalibData/T%d1.txt",data->CAM_PAIRS[sc]);
        printf("%d: %s\n",sc,MatName);
        fstream fd_matrix(MatName,ios::in);
        /* if no rotation matrix is available, set to identity matrix */
        if(!fd_matrix.good())
        {
            cout << "No Trafo Matrix available" << endl;
            for(int i=0; i<4; ++i)
            {
                for(int j=0; j<4; ++j)
                {
                    stereo_cluster[sc]->TrafoMatrix[i][j] = 0;
                    if(i==j) stereo_cluster[sc]->TrafoMatrix[i][j] = 1;
                }
            }
        }
        /* otherwise read rotation matrix from file */
        else
        {
            for(int i=0; i<4; ++i)
            {
                for(int j=0; j<4; ++j)
                {
                    fd_matrix >> stereo_cluster[sc]->TrafoMatrix[i][j];
                    printf("%f ",stereo_cluster[sc]->TrafoMatrix[i][j]);
                }
                printf("\n");
            }
        }
    

        /* read trained networks from file */
        char ann_filename[32];
        sprintf(ann_filename,"/home/nst3018/Development/cam_track_new/Tracker/CalibData/cam_pair_%d.net",data->CAM_PAIRS[sc]);
        printf("loading: %s\n",ann_filename);
        stereo_cluster[sc]->ann = fann_create_from_file(ann_filename);
        if(!stereo_cluster[sc]->ann)
        {
            printf("ANN could not be loaded\n");
            exit(EXIT_FAILURE);
        }
    }
/******************************************************************************************************/


    char log_file_name[64];
    for(int sc=0; sc<data->NUM_OF_CAM_PAIRS; ++sc)
    {
        sprintf(log_file_name,"cam%d_log.txt",stereo_cluster[sc]->tracker[0]->idx);
        stereo_cluster[sc]->pxl_log_cam0 = fopen(log_file_name,"w");
        if(!stereo_cluster[sc]->pxl_log_cam0)
        {
            fprintf(stderr,"Error opening log file\n");
            exit(EXIT_FAILURE);
        }
        sprintf(log_file_name,"cam%d_log.txt",stereo_cluster[sc]->tracker[1]->idx);
        stereo_cluster[sc]->pxl_log_cam1 = fopen(log_file_name,"w");
        if(!stereo_cluster[sc]->pxl_log_cam1)
        {
            fprintf(stderr,"Error opening log file\n");
            exit(EXIT_FAILURE);
        }
        sprintf(log_file_name,"net_coord_%d.txt",stereo_cluster[sc]->id);
        stereo_cluster[sc]->net_log = fopen(log_file_name,"w");
        if(!stereo_cluster[sc]->net_log)
        {
            fprintf(stderr,"Unable to open network coord log file\n");
            exit(EXIT_FAILURE);
        }
    }

    for(int sc=0; sc<data->NUM_OF_CAM_PAIRS; ++sc)
    {
        printf("StereoCluster %d: (cam%d, cam%d)\n",stereo_cluster[sc]->id,stereo_cluster[sc]->tracker[0]->idx,stereo_cluster[sc]->tracker[1]->idx);
    }

    return stereo_cluster;
}



void init_data(int argc, char *argv[], struct StaticData *data)
{
    /* register SIGINT interrupt handler */
    if( signal(SIGINT,sig_handler) == SIG_ERR )
    {
        fprintf(stderr,"Error registering signal handler\n");
        exit(EXIT_FAILURE);
    }

    
    /* initialize everything with default values */
    data->MAX_BUF = 200;
    data->PORT_NUM = 56000;

    data->NUM_OF_MARKERS = 2;
    data->NUM_OF_CAMS = 4;
    data->NUM_OF_CAM_PAIRS = 1;
    data->CAM_PAIRS = (int *)calloc(1,sizeof(int));
    data->CAM_PAIRS[0] = 0;

    data->capture_source = REMOTE;

    data->img_base_path = (char *)calloc(128,sizeof(char));
    sprintf(data->img_base_path,"../Data/recording/");

    data->cam_config = (CamConfig *)calloc(1,sizeof(CamConfig));

    data->cam_config->bytes_per_packet = 2936;
    data->cam_config->shutter = 100;
    data->cam_config->ub_value = 650;
    data->cam_config->vr_value = 520;

    data->cam_guid = (unsigned long *)calloc(8,sizeof(unsigned long));   
    /* unique identifiers of cameras */
    data->cam_guid[0] = 2892819639783898;        // l0 (color)
    data->cam_guid[1] = 2892819639783895;        // l1 (color)
    data->cam_guid[2] = 2892819639783896;        // l2 (color)
    data->cam_guid[3] = 2892819639783897;        // l3 (color)
    data->cam_guid[4] = 2892819639812243;        // h0 (b/w)
    data->cam_guid[5] = 2892819639812241;        // h1 (b/w)
    data->cam_guid[6] = 2892819639812152;        // h2 (b/w)
    data->cam_guid[7] = 2892819639812244;        // h3 (b/w)


    data->RotationMatrix = (double **)calloc(3,sizeof(double*));
    for(int i=0; i<3; ++i)
    {
        data->RotationMatrix[i] = (double *)calloc(3,sizeof(double));
    }

    /*- Read rotation matrix -*/
    char RotMatName[32];
    sprintf(RotMatName,"/home/nst3018/Development/cam_track_new/Tracker/CalibData/RotationMatrix.txt");
    printf("%s\n",RotMatName);
    fstream fd_rotation_matrix(RotMatName,ios::in);
    /* if no rotation matrix is available, set to identity matrix */
    if(!fd_rotation_matrix.good())
    {
        cout << "No Rotation Matrix available" << endl;
        for(int i=0; i<3; ++i)
        {
            for(int j=0; j<3; ++j)
            {
                data->RotationMatrix[i][j] = 0;
                if(i==j) data->RotationMatrix[i][j] = 1;
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
                fd_rotation_matrix >> data->RotationMatrix[i][j];
            }
        }
    }

    data->verbose = 0;
    
    data->record_coordinate_system = 0;

    /* read global configuration from file */
    read_config(data);

    /* parse the command line options */
    parse_command_line(argc, argv, data);


}

/* read settings from config file and overwrite current settings if available */
void read_config(struct StaticData *data)
{

	printf("\n*--------* Configuration *--------*\n");
	
	config_t config;
	config_init(&config);
	
	if(!config_read_file(&config, "config.txt"))
	{
		fprintf(stderr,"%s:%d - %s\n",config_error_file(&config),config_error_line(&config),config_error_text(&config));	
	}

	const char *cs;
	if(config_lookup_string(&config,"cam_tracker.variables.capture_source",&cs))
	{
		if(strcmp("REMOTE",cs)==0)
		{
			data->capture_source = REMOTE;
			printf("Capture Source: REMOTE\n");
		}
		else if(strcmp("DISK",cs))
		{
			data->capture_source = DISK;
			printf("Capture Source: DISK\n");
		}
	}

	int bpp;
	if(config_lookup_int(&config,"cam_tracker.variables.bytes_per_packet",&bpp))
	{
		data->cam_config->bytes_per_packet = bpp;
		printf("IEEE 1394 packet size: %d\n",data->cam_config->bytes_per_packet);
	}

	const char *bp;
	if(config_lookup_string(&config,"cam_tracker.variables.img_base_path",&bp))
	{
		sprintf(data->img_base_path,"%s\n",bp);
		printf("base path: %s\n",data->img_base_path);
	}

	int shutter;
	if(config_lookup_int(&config,"cam_tracker.variables.shutter",&shutter))
	{
		data->cam_config->shutter = shutter;
		printf("Shutter: %d\n",data->cam_config->shutter);
	}

	int ub;
	if(config_lookup_int(&config,"cam_tracker.variables.ub_value",&ub))
	{
		data->cam_config->ub_value = ub;
		printf("White Balance UB: %d\n",data->cam_config->ub_value);
	}
		
	int vr;
	if(config_lookup_int(&config,"cam_tracker.variables.vr_value",&vr))
	{
		data->cam_config->vr_value = vr;
		printf("White Balance VR: %d\n",data->cam_config->vr_value);
	}

	int port_num;
	if(config_lookup_int(&config,"cam_tracker.variables.PORT_NUM",&port_num))
	{
		data->PORT_NUM = port_num;
		printf("Port Number: %d\n",data->PORT_NUM);
	}

	int buffer_size;
	if(config_lookup_int(&config,"cam_tracker.variables.SENDING_BUFFER_SIZE",&buffer_size))
	{
		data->MAX_BUF = buffer_size;
		printf("Buffer Size: %d\n",data->MAX_BUF);
	}

	int noc;
	if(config_lookup_int(&config,"cam_tracker.variables.NUM_OF_CAMS",&noc))
	{
		data->NUM_OF_CAMS = noc;
		printf("Num of Cams: %d\n",data->NUM_OF_CAMS);
	}

	int nom;
	if(config_lookup_int(&config,"cam_tracker.variables.NUM_OF_MARKERS",&nom))
	{
		data->NUM_OF_MARKERS = nom;
		printf("Num of Markers: %d\n",data->NUM_OF_MARKERS);
	}

	int nocp;
	if(config_lookup_int(&config,"cam_tracker.variables.NUM_OF_CAM_PAIRS",&nocp))
	{
		data->NUM_OF_CAM_PAIRS = nocp;
		printf("Num of Cam Pairs: %d\n",data->NUM_OF_CAM_PAIRS);
	}

	const config_setting_t *colors;
	colors = config_lookup(&config,"cam_tracker.variables.colors");
	int count = config_setting_length(colors);
	if(count<data->NUM_OF_MARKERS)
	{
		fprintf(stderr,"Wrong color settings for marker\n");
		exit(EXIT_FAILURE);
	}

	data->colors = (int *)calloc(data->NUM_OF_MARKERS,sizeof(int));
	const char *c = NULL;
	printf("Colors: ");
	for(int i=0; i<data->NUM_OF_MARKERS; ++i)
	{
		c = config_setting_get_string_elem(colors,i);
		if(!c)
		{
			fprintf(stderr,"%s:%d - %s\n",config_error_file(&config),config_error_line(&config),config_error_text(&config));
			exit(EXIT_FAILURE);
		}
		if(strcmp(c,"RED")==0)
		{
			data->colors[i] = RED;
			printf("RED ");
		}
		else if(strcmp(c,"GREEN")==0)
		{
			data->colors[i] = GREEN;
			printf("GREEN ");
		}
		else if(strcmp(c,"BLUE")==0)
		{
			data->colors[i] = BLUE;
			printf("BLUE ");
		}
		else
		{
			fprintf(stderr,"Unknown color\n");
			exit(EXIT_FAILURE);
		}
	}
	printf("\n");

    const config_setting_t *cam_pairs;
    cam_pairs = config_lookup(&config,"cam_tracker.variables.CAM_PAIRS");
    int len = config_setting_length(cam_pairs);
    if(len!=data->NUM_OF_CAM_PAIRS)
    {
        fprintf(stderr,"Number of used cam pairs does not fit size of cam pair list\n");
        exit(EXIT_FAILURE);
    }
    
    data->CAM_PAIRS = (int *)calloc(len,sizeof(int));

    for(int p=0; p<len; ++p)
    {
        data->CAM_PAIRS[p] = config_setting_get_int_elem(cam_pairs,p);
    }


	printf("*---------------------------------*\n\n");

    config_destroy(&config);
}


/* read command line options and set global variables accordingly */
void parse_command_line(int argc, char *argv[], struct StaticData *data)
{

    int c;

    /*
     * command line options:
     * s: shutter
     * r: record
     * v: verbose
     * h: help
    */
    while((c = getopt(argc, argv, "s:rvhT")) != -1)
    {
        switch(c)
        {
        case 's':
            data->cam_config->shutter = atoi(optarg);
            printf("shutter = %d\n",data->cam_config->shutter);
            break;
        case 'r':
            data->is_recording = 1;
            cout << "Record raw data to disk\n" << endl;
            break;
        case 'v':
            data->verbose = 1;
            printf("Verbose output\n");
            break;
        case 'h':
            show_help();
            exit(0);
            break;
        case 'T':
            data->record_coordinate_system = 1;
            break;
        }
    }
}





/* initialization routine for firewire cams */
int start_camera_transmission(dc1394camera_t * camera)
{
    // start camera transmissions
        
    if(dc1394_capture_setup(camera, 10, DC1394_CAPTURE_FLAGS_DEFAULT) != DC1394_SUCCESS)
    {
        cerr << "Could not setup cam " << endl
             << "Make sure that the video mode and framerate are supported" << endl;
        return -1;
    }

    if(dc1394_video_set_transmission(camera, DC1394_ON) != DC1394_SUCCESS)
    {
        cerr << "Could not start iso transmission on cam " << endl;
        return -1;
    }
    
    return 1;
}

/* initialization routine for firewire cams */
int set_camera_params(dc1394camera_t *camera, struct StaticData *data)
{
    //Set camera parameters
    

    // normal mode with 400 MBit
    if(dc1394_video_set_operation_mode(camera, DC1394_OPERATION_MODE_LEGACY) != DC1394_SUCCESS)
    {
        cerr << "Could not set normal operation mode for cam " << endl;
        return -1;
    }

    if(dc1394_video_set_iso_speed(camera, DC1394_ISO_SPEED_400) != DC1394_SUCCESS)
    {
        cerr << "Could not set iso speed 400 for cam " << endl;
        return -1;
    }

    //Format7_0
    if(dc1394_video_set_mode(camera, DC1394_VIDEO_MODE_FORMAT7_0) != DC1394_SUCCESS)
    {
        cerr << "Could not set video mode Format7_0 for cam " << endl;
        return -1;
    }

    if(dc1394_format7_set_image_size(camera, DC1394_VIDEO_MODE_FORMAT7_0,  FRAME_WIDTH, FRAME_HEIGHT) != DC1394_SUCCESS)
    {
        cerr << "Could not set Format7 image size for cam " << endl;
        return -1;
    }


    /* Shutter configuration */
    float shutter_min, shutter_max;
    /* get info about possible shutter values */
    if(dc1394_feature_get_absolute_boundaries(camera, DC1394_FEATURE_SHUTTER, &shutter_min, &shutter_max)!= DC1394_SUCCESS)
    {
        cerr << "Could not dc1394_feature_get_absolute_boundaries" << endl;
        return -1;
    }
    float shutter = 0;
    /* read current shutter setting */
    if(dc1394_feature_get_absolute_value(camera, DC1394_FEATURE_SHUTTER, &shutter) != DC1394_SUCCESS)
    {
        cerr << "Could not dc1394_feature_get_absolute_value" << endl;
        return -1;
    }
//      cout << "Camera " << i << ": shutter=" << shutter << ", min=" << shutter_min << ", max=" << shutter_max << endl;
    /* set new shutter value */
    if(dc1394_feature_set_value(camera, DC1394_FEATURE_SHUTTER, data->cam_config->shutter) != DC1394_SUCCESS)
    {
        cerr << "Could not dc1394_feature_set_absolute_value" << endl;
        return -1;
    }
    if(dc1394_feature_get_absolute_value(camera, DC1394_FEATURE_SHUTTER, &shutter) != DC1394_SUCCESS)
    {
        cerr << "Could not dc1394_feature_get_absolute_value" << endl;
        return -1;
    }
    cout << "Camera " << ": shutter=" << shutter << ", min=" << shutter_min << ", max=" << shutter_max << endl;

    /* white balance */
    uint32_t u_b_value,v_r_value;
    if(dc1394_feature_whitebalance_get_value(camera,&u_b_value,&v_r_value) != DC1394_SUCCESS)
    {
        cerr << "Could not dc1394_feature_whitebalance_get_value" << endl;
        return -1;
    }
//      cout << "Camera " << i << ": whitebalance = (" << u_b_value << ", " << v_r_value << ")" << endl;
    if(dc1394_feature_whitebalance_set_value(camera,data->cam_config->ub_value,data->cam_config->vr_value) != DC1394_SUCCESS)
    {
        cerr << "Could not dc1394_feature_whitebalance_set_value" << endl;
        return -1;
    }
    if(dc1394_feature_whitebalance_get_value(camera,&u_b_value,&v_r_value) != DC1394_SUCCESS)
    {
        cerr << "Could not dc1394_feature_whitebalance_get_value" << endl;
        return -1;
    }
    cout << "Camera " << ": whitebalance = (" << u_b_value << ", " << v_r_value << ")" << endl;

    dc1394color_coding_t cm;
    if(camera->guid==data->cam_guid[0] || 
        camera->guid==data->cam_guid[1] || 
        camera->guid==data->cam_guid[2] || 
        camera->guid==data->cam_guid[3])
    {
        cm = DC1394_COLOR_CODING_RAW8;
    }
    else
    {
        cm = DC1394_COLOR_CODING_MONO8;
    }
#if 0
    /* Color coding scheme setting */
    dc1394color_coding_t cm = DC1394_COLOR_CODING_MONO8;
    switch(color_mode)
    {
    case Mono8:
        /* 8 bit grayscale image */
        cm = DC1394_COLOR_CODING_MONO8;
        break;
    case RGB24:
        /* 24 bit 3 channel color image */
        cm = DC1394_COLOR_CODING_RGB8;
        break;
    case Bayer8:
        /* 8 bit bayer filtered grayscale */
        cm = DC1394_COLOR_CODING_RAW8;
        break;
    };
#endif

    if(dc1394_format7_set_color_coding(camera, DC1394_VIDEO_MODE_FORMAT7_0, cm) != DC1394_SUCCESS)
    {
        cerr << "Could not set color coding for cam " << endl;
        return -1;
    }

    /* IEEE 1394 packet payload size */
    if(dc1394_format7_set_packet_size(camera, DC1394_VIDEO_MODE_FORMAT7_0, data->cam_config->bytes_per_packet) != DC1394_SUCCESS)
    {
        cerr << "Could not set " << data->cam_config->bytes_per_packet << " bytes per packet for cam " << endl;
        return -1;
    }

    return 1;

}


/* Global function for cleaning up allocated memory and exiting. */
void cleanup_and_exit(dc1394_t *dc1394, vector<dc1394camera_t *>& cameras)
{
    for(unsigned int i=0; i<cameras.size(); i++)
    {
        dc1394_video_set_transmission(cameras[i], DC1394_OFF);
        dc1394_capture_stop(cameras[i]);
    }
    for(unsigned int i=0; i<cameras.size(); i++)
    {
        dc1394_camera_free(cameras[i]);
    }
    dc1394_free(dc1394);
    exit(1);
}


/* print help to command line */
void show_help()
{
    printf("\ncam_track:\n\nThe software supports either capturing from firewire cameras or from local disc. ");
    printf("Default values set to enable tracking of two markers and two cameras at 49 fps under normal conditions. ");
    printf("If tracking result is unstable, try to change the shutter setting.\n\n");
    printf("For more config options check the \"config.txt\" configuration file\n\n");
    printf("\th: Show this help\n\n");
 
    printf("\ts <speed>: shutter speed (default 100)\n");
    printf("\tr: record raw data to disk\n");
    printf("\tv: verbosity\n");
}

void sig_handler(int signal)
{
    if(signal!=SIGINT)
    {
        return;
    }
    printf("\nquit\n");
    exit_state = 1;
}




void print_data(struct StaticData *data)
{

    printf("Camera Settings:\n");
    printf("\tbytes per packet: %d\n",data->cam_config->bytes_per_packet);
    printf("\tshutter: %d\n",data->cam_config->shutter);
    printf("\twhite balance: (%d,%d)\n",data->cam_config->ub_value,data->cam_config->vr_value);

    printf("\n");

    printf("Global Settings:\n");

    printf("\tcapture source: ");
    if(data->capture_source==REMOTE)
    {
        printf("REMOTE\n");
    }
    else if(data->capture_source==DISK)
    {
        printf("DISK\n");
    }

    printf("\tnum of cams: %d\n",data->NUM_OF_CAMS);
    printf("\tnum of markers: %d\n",data->NUM_OF_MARKERS);
    printf("\tnum of cam pairs: %d\n",data->NUM_OF_CAM_PAIRS);

    printf("\tcam pairs: ");
    for(int p=0; p<data->NUM_OF_CAM_PAIRS; ++p)
    {
        printf("%d ",data->CAM_PAIRS[p]);
    }
    printf("\n");

    printf("\nMarker color order: ");
    printf("( ");

    for(int i=0; i<data->NUM_OF_MARKERS; ++i)
    {
        switch(data->colors[i])
        {
            case RED:
                printf("RED");
                break;
            case GREEN:
                printf("GREEN");
                break;
            case BLUE:
                printf("BLUE");
                break;
            default:
                break;
        }
        printf(" ");
        
    }
    printf(")\n");

    if(data->is_recording)
    {
        printf("\trecord raw data @ %s\n",data->img_base_path);
    }
    if(data->verbose)
    {
        printf("verbose output\n");
    }

    printf("\n");

    printf("Streaming Server Settings:\n");
    printf("\tBuffer Size: %d\n",data->MAX_BUF);
    printf("\tPort: %d\n",data->PORT_NUM);


}

/**
 * initiates the tracker, i.e. connect the camera, declare image headers, init marker structs
**/
int init_tracker(Tracker *tracker, dc1394camera_t *cam, struct StaticData *data)
{
    /* connect the camera */
    tracker->camera = cam;


    /* if cam==NULL, capture from disk, otherwise capture from camera */
    if(data->capture_source==REMOTE && !tracker->camera)
    {
        fprintf(stderr,"init_tracker(): cameras not initialized\n");
        exit(EXIT_FAILURE);
    }
    else if(data->capture_source==DISK && tracker->camera)
    {
        fprintf(stderr,"init_tracker(): capture source set to DISK but cameras are initialized\n");
        exit(EXIT_FAILURE);
    }

    tracker->source = data->capture_source;
    if(cam!=NULL)
    {

        tracker->cam_guid = cam->guid;
        if(tracker->cam_guid == data->cam_guid[0])
        {
            tracker->idx = 0;
        }
        else if(tracker->cam_guid == data->cam_guid[1])
        {
            tracker->idx = 1;
        }
        else if(tracker->cam_guid == data->cam_guid[2])
        {
            tracker->idx = 2;
        }
        else if(tracker->cam_guid == data->cam_guid[3])
        {
            tracker->idx = 3;
        }
        else if(tracker->cam_guid == data->cam_guid[4])
        {
            tracker->idx = 4;
        }
        else if(tracker->cam_guid == data->cam_guid[5])
        {
            tracker->idx = 5;
        }
        else if(tracker->cam_guid == data->cam_guid[6])
        {
           tracker->idx = 6;
        }
        else if(tracker->cam_guid == data->cam_guid[7])
        {
            tracker->idx = 7;
        }

        /* first four cams are color cams, the last four b/w */
        if(cam->guid==data->cam_guid[0] || cam->guid==data->cam_guid[1] ||
            cam->guid==data->cam_guid[2] || cam->guid==data->cam_guid[3] )
        {
            cout << "color" << endl;
            tracker->color = 1;
        }
        else
        {
            cout << "b/w" << endl;
            tracker->color = 0;
        }
    }

    /* create frame header */
    CvSize sz = cvSize(FRAME_WIDTH,FRAME_HEIGHT);
    if(tracker->color)
    {
        /* if color camera, create header for 3 channel image */
        tracker->frame = cvCreateImage(sz,IPL_DEPTH_8U,3);
    }
    else
    {
        /* if b/w cam, create one channel image */
        tracker->frame = cvCreateImage(sz,IPL_DEPTH_8U,1);
    }

    /* create and init markers */
    Marker *new_marker;
    for(int i=0; i<data->NUM_OF_MARKERS; ++i)
    {
        /* allocate memory for new marker struct */
        new_marker = new Marker;
        /* push newly created marker into vector */
        tracker->marker.push_back(new_marker);
        /* init the newly created marker struct */
        init_marker(tracker->marker[i],data->colors[i]);
    }

    /* debug output: window name to display images */
    sprintf(tracker->win_name,"Cam%d",tracker->idx);

    /* reset frame counter */
    tracker->frame_counter = 0;


    /* initially no marker has been detected */
    tracker->state = OFF_TRACK;

    return 1;
}

/**
 * init markers, i.e. init position, velocity and acceleration values
 * and set color values for color difference computation
**/
void init_marker(Marker *marker, int color)
{
    /* flag to indicate if the marker has been found */
    marker->pos_is_set = 0;
    marker->vel_is_set = 0;
    marker->acc_is_set = 0;
    marker->roi_set = 0;

    /* init variables (position, velocity, acceleration) with 0 */
    marker->pos_measured = cvPoint2D32f(0.0,0.0);
    marker->pos_predicted = cvPoint2D32f(0.0,0.0);
    marker->vel.x = 0;
    marker->vel.y = 0;
    marker->acc.x = 0;
    marker->acc.y = 0;

    marker->blob_pos.x = 0;
    marker->blob_pos.y = 0;

    /* set the color for the current marker (LED) */
    marker->color = color;

    /* depending on led color set values to compare with */
    if(color==RED)
    {
        marker->colorvalue.h = 0.0;
        marker->colorvalue.s = 1.0;
        marker->colorvalue.v = 1.0;
    }
    else if(color==GREEN)
    {
        marker->colorvalue.h = 0.33;
        marker->colorvalue.s = 1.0;
        marker->colorvalue.v = 1.0;
    }
    else if(color==BLUE)
    {
        marker->colorvalue.h = 0.66;
        marker->colorvalue.s = 1.0;
        marker->colorvalue.v = 1.0;
    }


    /* init region of interest to whole image */
    marker->roi = cvRect(0,0,FRAME_WIDTH,FRAME_HEIGHT);
}


/* Run input through ANN and fill output with the rotated elements of the network output. */
void compute_net_coordinates(StereoCluster *cluster)
{
    if(!cluster->ann)
    {
        fprintf(stderr,"Neural network not available\n");
        exit(EXIT_FAILURE);
    }


    /* allocate memory for network input and resulting output */
    fann_type network_input[4];
    fann_type *network_output;

    for(int nm=0; nm<cluster->data->NUM_OF_MARKERS; ++nm)
    {
        network_input[0] = 2.0*(cluster->tracker[0]->marker[nm]->pos_measured.x/(double)FRAME_WIDTH) - 1.0;
        network_input[1] = 2.0*(cluster->tracker[0]->marker[nm]->pos_measured.y/(double)FRAME_HEIGHT) - 1.0;
        network_input[2] = 2.0*(cluster->tracker[1]->marker[nm]->pos_measured.x/(double)FRAME_WIDTH) - 1.0;
        network_input[3] = 2.0*(cluster->tracker[1]->marker[nm]->pos_measured.y/(double)FRAME_HEIGHT) - 1.0;

//        printf("%d: %f %f %f %f\n",cluster->id,network_input[0],network_input[1],network_input[2],network_input[3]);
        /* apply inputs to neural network, output corresponds to arbitrarily rotated world coordinates */
        network_output = fann_run(cluster->ann, network_input); 

        //printf("cluster: %d, %d: %f %f %f\n",cluster->id,nm,network_output[0],network_output[1],network_output[2]);
        /* compute corrected world coordinates by rotating and translating the network output, 
        i.e. all clusters share a common coordinate system */
        
        cluster->net_coordinates[nm].x =    network_output[0]*cluster->TrafoMatrix[0][0] +
                                            network_output[1]*cluster->TrafoMatrix[0][1] +
                                            network_output[2]*cluster->TrafoMatrix[0][2] + cluster->TrafoMatrix[0][3];
//        printf("%d: %f %f %f %f\n",cluster->id,cluster->TrafoMatrix[0][0],cluster->TrafoMatrix[0][1],cluster->TrafoMatrix[0][2],cluster->TrafoMatrix[0][3]);
        cluster->net_coordinates[nm].y =    network_output[0]*cluster->TrafoMatrix[1][0] +
                                            network_output[1]*cluster->TrafoMatrix[1][1] +
                                            network_output[2]*cluster->TrafoMatrix[1][2] + cluster->TrafoMatrix[1][3];
        cluster->net_coordinates[nm].z =    network_output[0]*cluster->TrafoMatrix[2][0] +
                                            network_output[1]*cluster->TrafoMatrix[2][1] +
                                            network_output[2]*cluster->TrafoMatrix[2][2] + cluster->TrafoMatrix[2][3];
        
        /*
        cluster->net_coordinates[nm].x = network_output[0];
        cluster->net_coordinates[nm].y = network_output[1];
        cluster->net_coordinates[nm].z = network_output[2];
        */

//        printf("%d %d: %f %f %f\n",cluster->id,nm,cluster->net_coordinates[nm].x,cluster->net_coordinates[nm].y,cluster->net_coordinates[nm].z);

    }
    double dt = sqrt( (cluster->net_coordinates[0].x-cluster->net_coordinates[1].x)*(cluster->net_coordinates[0].x-cluster->net_coordinates[1].x) + 
                    (cluster->net_coordinates[0].y-cluster->net_coordinates[1].y)*(cluster->net_coordinates[0].y-cluster->net_coordinates[1].y) + 
                    (cluster->net_coordinates[0].z-cluster->net_coordinates[1].z)*(cluster->net_coordinates[0].z-cluster->net_coordinates[1].z) 
        );
    //printf("%f\n",dt*100.0);

}


std::vector<cv::KalmanFilter *> init_kalman_filter(int N)
{
    const float delta_t = 0.02;
    std::vector<cv::KalmanFilter *> kalman(N);

    for(int ka=0; ka<N; ++ka)
    {
        /* Kalman filter initialization */
        kalman[ka] = new cv::KalmanFilter(9,3,0);

#if 0
        cv::KalmanFilter temp(  9,              // 9D state vector
                                3,              // 3D measurement vector
                                0               // no control input
                            );
#endif
        kalman[ka]->transitionMatrix = *(cv::Mat_<float>(9,9)  <<  1,0,0,delta_t,0,0,0.5*delta_t*delta_t,0,0,
                                                        0,1,0,0,delta_t,0,0,0.5*delta_t*delta_t,0,
                                                        0,0,1,0,0,delta_t,0,0,0.5*delta_t*delta_t,
                                                        0,0,0,1,0,0,delta_t,0,0,
                                                        0,0,0,0,1,0,0,delta_t,0,
                                                        0,0,0,0,0,1,0,0,delta_t,
                                                        0,0,0,0,0,0,1,0,0,  
                                                        0,0,0,0,0,0,0,1,0,
                                                        0,0,0,0,0,0,0,0,1); 
        // init...
        kalman[ka]->statePre.at<float>(0) = 0;
        kalman[ka]->statePre.at<float>(1) = 0;
        kalman[ka]->statePre.at<float>(2) = 0;
        kalman[ka]->statePre.at<float>(3) = 0;
        kalman[ka]->statePre.at<float>(4) = 0;
        kalman[ka]->statePre.at<float>(5) = 0;
        kalman[ka]->statePre.at<float>(6) = 0;
        kalman[ka]->statePre.at<float>(7) = 0;
        kalman[ka]->statePre.at<float>(8) = 0;
    
        setIdentity(kalman[ka]->measurementMatrix);
        setIdentity(kalman[ka]->processNoiseCov, cv::Scalar::all(2e-4));
        setIdentity(kalman[ka]->measurementNoiseCov, cv::Scalar::all(2e-4));
        setIdentity(kalman[ka]->errorCovPost, cv::Scalar::all(.1));
            
        
    }

    return kalman;
}


void compute_RPY_angles(CvPoint3D32f *marker, double *alpha, double *beta, double *gamma)
{
    double abs;

    /* Compute unit vectors of rotated cosy */
    CvPoint3D32f cosy[3];

    /* vector in x direction */
    cosy[0].x = marker[2].x - (marker[0].x+marker[1].x)/2.0;
    cosy[0].z = marker[2].z - (marker[0].z+marker[1].z)/2.0;
    cosy[0].y = marker[2].y - (marker[0].y+marker[1].y)/2.0;
    /* normalize */
    abs = sqrt(cosy[0].x*cosy[0].x + cosy[0].y*cosy[0].y + cosy[0].z*cosy[0].z);
    cosy[0].x /= abs;
    cosy[0].y /= abs;
    cosy[0].z /= abs;

    /* unit vector in y direction */
    cosy[1].x = marker[1].x - marker[0].x;
    cosy[1].y = marker[1].y - marker[0].y;
    cosy[1].z = marker[1].z - marker[0].z;
    /* normalize */
    abs = sqrt(cosy[1].x*cosy[1].x + cosy[1].y*cosy[1].y + cosy[1].z*cosy[1].z);
    cosy[1].x /= abs;
    cosy[1].y /= abs;
    cosy[1].z /= abs;


    /* unit vector in z direction */
    cosy[2].x = cosy[0].y*cosy[1].z - cosy[0].z*cosy[1].y;
    cosy[2].y = cosy[0].z*cosy[1].x - cosy[0].x*cosy[1].z;
    cosy[2].z = cosy[0].x*cosy[1].y - cosy[0].y*cosy[1].x;
    

    /* Compute RPY angles */
    *beta = atan2(-cosy[0].z,sqrt(cosy[0].x*cosy[0].x + cosy[0].y*cosy[0].y));
    *alpha = atan2(cosy[0].y/cos(*beta),cosy[0].x/cos(*beta));
    *gamma = atan2(cosy[1].z/cos(*beta),cosy[2].z/cos(*beta));
}



struct _MarkerPosition* init_marker_position(struct StaticData *data)
{
    /* allocate memory */
    struct _MarkerPosition *mp = (struct _MarkerPosition *)calloc(1, sizeof(struct _MarkerPosition));
    mp->cluster_coordinates = (struct _ClusterCoordinates **)calloc(data->NUM_OF_CAM_PAIRS,sizeof(_ClusterCoordinates *));

    for(int p=0; p<data->NUM_OF_CAM_PAIRS; ++p)
    {
        mp->cluster_coordinates[p] = (struct _ClusterCoordinates *)calloc(data->NUM_OF_CAM_PAIRS,sizeof(_ClusterCoordinates));
        mp->cluster_coordinates[p]->pxl_coordinates_0 = (CvPoint2D32f *)calloc(data->NUM_OF_MARKERS,sizeof(CvPoint2D32f));
        mp->cluster_coordinates[p]->pxl_coordinates_1 = (CvPoint2D32f *)calloc(data->NUM_OF_MARKERS,sizeof(CvPoint2D32f));

        mp->cluster_coordinates[p]->net_coordinates = (CvPoint3D32f *)calloc(data->NUM_OF_MARKERS,sizeof(CvPoint3D32f));
    } 

    mp->world_coordinates = (CvPoint3D32f *)calloc(data->NUM_OF_MARKERS,sizeof(CvPoint3D32f));


    mp->buffer = (char *)calloc(data->MAX_BUF,sizeof(char));
    memset(mp->buffer,0,data->MAX_BUF);


    mp->allowed_to_read = 0;
    mp->allowed_to_send = 0;

    
    printf("init_marker_position(): marker position initiated\n");


    mp->on_track = (int *)calloc(data->NUM_OF_CAM_PAIRS,sizeof(int));

    return mp;
}
