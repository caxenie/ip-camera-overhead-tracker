#ifndef DATA_TYPES
#define DATA_TYPES

/**
 * Struct to hold color values for color difference computation in HSV color space.
**/
typedef struct _ColorValue
{
    double h;
    double s;
    double v;
} ColorValue;


enum color
{
    RED,
    GREEN,
    BLUE,
    YELLOW
};

/* frame source type */
enum srctype
{
    REMOTE,
    DISK,
    LOCAL
};

/* color mode */
enum ColorMode
{
    Mono8,			// 8-bit per pixel (Grayscale). Corresponds to DC1394_COLOR_CODING_MONO8.
    Bayer8,			// Raw Bayer Coding: 8-bit value per pixel (RGGB). Corresponds to DC1394_COLOR_CODING_RAW8.
    RGB24			// Three 8-bit values = 24 bits per pixel (RGB). Corresponds to DC1394_COLOR_CODING_RGB8.
};

struct CamConfig 
{
	int bytes_per_packet;			// payload packet size for ieee 1394, influences fps
	int shutter;					// shutter speed
	uint32_t ub_value;				// white value setting: green to blue value in [0,1022]
	uint32_t vr_value;				// white value setting: green to red value in [0,1022]

};

/* static data */
struct StaticData
{
    /* settings for cameras */
	struct CamConfig *cam_config;

    /* universal identifier for each cam */
	unsigned long *cam_guid;

    /* the ordered color sequence of the markers (LED: colors of emitted light) */
	int *colors;

    /* capture either from cam (REMOTE) or from disk (DISK) */
	int capture_source; 

    /* general setting for tracking algorithm */
	int NUM_OF_CAMS;
	int NUM_OF_MARKERS;
	int NUM_OF_CAM_PAIRS;
    int *CAM_PAIRS;


    /* maximum size of streaming buffer */
	int MAX_BUF;

    /* port number of streaming server */
	int PORT_NUM;
	
	// set the base path for recording where to save stream
	char *img_base_path;

    /* recording flag */
	int is_recording;

    /* verbosity */
	int verbose;

	
	/* flag for recording calibration data */
	int record_coordinate_system;

    double **RotationMatrix;
};

struct _ClusterCoordinates
{
    CvPoint2D32f *pxl_coordinates_0;
    CvPoint2D32f *pxl_coordinates_1;
    CvPoint3D32f *net_coordinates;
};

/**
 * Global marker position struct
 * Tracking algorithm writes pixel coordinates,
 * reprojection routine writes world coordinates and
 * streaming thread sends buffer content to remote client
**/
struct _MarkerPosition
{
    /* signals flags for streamer and reprojection routine */
    int allowed_to_read;
    int allowed_to_send;

    int *on_track;
    
    /* pxl and net coordinates of each cluster */
    struct _ClusterCoordinates **cluster_coordinates;


    /* final marker world coordinates */
    CvPoint3D32f *world_coordinates;        

    /* timestamp since program start */
    uint64_t timestamp;

    /* sending buffer */
    char *buffer;
};

#endif


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
    CvPoint2D32f blob_pos;

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
    std::vector<Marker *> marker;

    /* window name for debug output */
    char win_name[16];

	/* tracker state: either on_track or off_track */
    int state;
	
    int idx;
    unsigned long cam_guid;

} Tracker;




/** 
 * central data struct: 
 *
 * holds information about:
 *  -> global settings
 *  -> cameras 
 *  -> epipolar geometry 
 *  -> calibration 
 *  -> tracking 
**/
typedef struct _StereoCluster
{
    /* amount of stereo pairs */
    int N; 
    int cam_pair;
    int id;

    int state;

    /* global settings */
    struct StaticData *data;

    /* Each stereo cluster consists of two cameras, thus one tracker for each camera */
    Tracker **tracker;

    
    /* Calibration data */
    double TrafoMatrix[4][4];
    struct fann *ann;
    double FundamentalMatrix[3][3];


    CvPoint3D32f *net_coordinates;


    /* universal identifiers of both cams attached to this cluster */
    unsigned long cam_guid[2];
    /* local ids of these cams */
    int cam_idx[2];


    /* the file descriptor for logging pixel coordinates */
    FILE *pxl_log_cam0;
    FILE *pxl_log_cam1;
    FILE *net_log;

}StereoCluster;