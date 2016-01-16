#include "ColorDiffTracker.h"


/**
 * initiates the tracker, i.e. connect the camera, declare image headers, init marker structs
**/
void init_tracker(Tracker *tracker, dc1394camera_t *cam, int id, int num_of_markers, int colors[])
{
    /* connect the camera */
    tracker->camera = cam;

    /* if cam==NULL, capture from disk, otherwise capture from camera */
    if(!tracker->camera)
    {
        tracker->source = DISK;
    }
    else
    {
        tracker->source = REMOTE;
    }

	/* first four cams are color cams, the last four b/w */
	if(cam->guid==2892819639783898 ||
        cam->guid==2892819639783895 ||
        cam->guid==2892819639783896 ||
        cam->guid==2892819639783897 )
	{
        cout << "color" << endl;
		tracker->color = 1;
	}
	else
	{
        cout << "b/w" << endl;
		tracker->color = 0;
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
    for(int i=0; i<num_of_markers; ++i)
    {
		/* allocate memory for new marker struct */
        new_marker = new Marker;
		/* push newly created marker into vector */
        tracker->marker.push_back(new_marker);
		/* init the newly created marker struct */
        init_marker(tracker->marker[i],colors[i]);
    }

    /* debug output: window name to display images */
    sprintf(tracker->win_name,"Cam%d",id);

	/* reset frame counter */
    tracker->frame_counter = 0;

	/* assign id */
    tracker->idx = id;

	/* initially no marker has been detected */
    tracker->state = off_track;
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


/**
 * wrapper function to aggregate calls to compute_diff_maps() and findMinimum()
 * After this routine was called, the marker position, velocity and acceleration estimates are updated
**/
void track(Tracker *tracker)
{

	/* if no valid frame is available, clearly something went wrong */
    if(!tracker->frame)
    {
        cout << "No frame" << endl;
        exit(-1);
    }

    if(tracker->color)
    {
        /* compute the color difference maps and search for the global minimum */
        compute_color_diff_map(tracker);
        locate_marker(tracker);
    }
    else
    {
        bw_locate_marker(tracker);
    }


	/* keep trakc of which tracker is on track globally */
    cams_on_track[tracker->idx] = tracker->state;

}

int bw_locate_marker(Tracker *tracker)
{
    IplImage *thresh = cvCreateImage(cvGetSize(tracker->frame),IPL_DEPTH_8U,1);
    cvThreshold(tracker->frame,thresh,100,255,CV_THRESH_BINARY);

    cvShowImage("thresh",thresh);
}

/* computes the color difference map for the region of interest */
void compute_color_diff_map(Tracker *tracker)
{

    CvSize sz;

    /* for all markers (LEDs) */
    for(uint8_t nm=0; nm<tracker->marker.size(); ++nm)
    {
        /* if marker was found in the prior frame */
        if(tracker->marker[nm]->pos_is_set)
        {
            if(tracker->marker[nm]->vel_is_set)
            {
                /* if marker velocity is known predict new position */
                tracker->marker[nm]->pos_predicted.x = tracker->marker[nm]->pos_measured.x + tracker->marker[nm]->vel.x;
                tracker->marker[nm]->pos_predicted.y = tracker->marker[nm]->pos_measured.y + tracker->marker[nm]->vel.y;
            }
            else
            {
                /* otherwise take last known position for the center of the ROI as best guess */
                tracker->marker[nm]->pos_predicted.x = tracker->marker[nm]->pos_measured.x;
                tracker->marker[nm]->pos_predicted.y = tracker->marker[nm]->pos_measured.y;
            }

            /**
             * dynamically adapt roi size according to marker acceleration
             * linear upper boundary with slope 1.5 and intersect 4.
             * roi size is actually 2 times the computed value, because LED can be located at
             * x_predicted + x_prediction_error or at
             * x_predicted - x_prediction_error.
             * The same holds true for the y direction.
            **/
            uint8_t adapt_roi_width;
            uint8_t adapt_roi_height;
            if(tracker->marker[nm]->acc_is_set)
            {
                /**
                 * if marker acc is known adapt roi size dynamically according to linear
                 * upper bound of prediction error made (measured)
                **/
                adapt_roi_width = 2 * round(fabs(1.5 *tracker->marker[nm]->acc.x) + 4.0);
                adapt_roi_height = 2 * round(fabs(1.5 * tracker->marker[nm]->acc.y) + 4.0);
            }
            else
            {
                /* otherwise take default roi size */
                adapt_roi_width = ROI_WIDTH;
                adapt_roi_height = ROI_HEIGHT;
            }

            /* check wether roi is within image boundaries and update roi position */
            if(		(int)tracker->marker[nm]->pos_predicted.x-adapt_roi_width/2>=0 &&
                    (int)tracker->marker[nm]->pos_predicted.x+adapt_roi_width/2<FRAME_WIDTH &&
                    (int)tracker->marker[nm]->pos_predicted.y-adapt_roi_height/2>=0 &&
                    (int)tracker->marker[nm]->pos_predicted.y+adapt_roi_height/2<FRAME_HEIGHT
              )
            {
                tracker->marker[nm]->roi = cvRect(	(int)tracker->marker[nm]->pos_predicted.x-adapt_roi_width/2,
                                                    (int)tracker->marker[nm]->pos_predicted.y-adapt_roi_height/2,
                                                    adapt_roi_width,
                                                    adapt_roi_height
                                                 );
                /* set the region of interest to the computed size and origin */
                cvSetImageROI(tracker->frame,tracker->marker[nm]->roi);
                tracker->marker[nm]->roi_set = 1;
            }
            /* otherwise extend search on whole image */
            else
            {
                tracker->marker[nm]->roi = cvRect(0,0,FRAME_WIDTH,FRAME_HEIGHT);
                tracker->marker[nm]->roi_set = 0;
            }

        }
        /* otherwise search on whole image */
        else
        {
            tracker->marker[nm]->roi = cvRect(0,0,FRAME_WIDTH,FRAME_HEIGHT);
            tracker->marker[nm]->roi_set = 0;
        }


        sz = cvSize(tracker->marker[nm]->roi.width,tracker->marker[nm]->roi.height);

        /**
         * Define intermediate images.
         * ROI is converted to floating point HSV color space.
        **/
        IplImage *hsv = cvCreateImage(sz,IPL_DEPTH_8U,3);
        IplImage *hsv_f = cvCreateImage(sz,IPL_DEPTH_32F,3);

        /* create image header to hold distance map according to the computed ROI size */
        tracker->marker[nm]->result = cvCreateImage(sz,IPL_DEPTH_32F,1);

        /* reset distance map to zero */
        cvZero(tracker->marker[nm]->result);

        float h_val,s_val,v_val;
        float res;

        /* Convert ROI to floating point HSV image */
        cvCvtColor(tracker->frame,hsv,CV_BGR2HSV);
        cvConvertScale(hsv,hsv_f,1.0);


        /**
         * compute the mean squared error (color distance) for all markers between the pixel value and the specified marker color (red,green,blue)
         * for all pixels in the region of interest.
         * store result in  the floating point image tracker->result.
        **/
        for(int i=0; i<tracker->marker[nm]->roi.width; ++i)
        {
            for(int j=0; j<tracker->marker[nm]->roi.height; ++j)
            {
                /* normalize pixel values between 0 and 1; h in [0,180], s,v in [0,255] */
                h_val = ((float*)(hsv_f->imageData + hsv_f->widthStep*j))[i*3] / 181.0;
                s_val = ((float*)(hsv_f->imageData + hsv_f->widthStep*j))[i*3+1] / 256.0;
                v_val = ((float*)(hsv_f->imageData + hsv_f->widthStep*j))[i*3+2] / 256.0;

                /**
                 * exclude values that are not in the same color range (H component) and values that are too dark (V component)
                **/
                if(fabs(h_val-tracker->marker[nm]->colorvalue.h)<0.1 && v_val>0.1)
                {
                    /**
                     * compute root mean squared errors and store result in tracker->result.
                     * res = sqrt(3) - e; minima are converted into maxima and vice versa
                    **/
                    res = 1.73 - sqrt(
                              (h_val-tracker->marker[nm]->colorvalue.h)*(h_val-tracker->marker[nm]->colorvalue.h)
                              + 	(s_val-tracker->marker[nm]->colorvalue.s)*(s_val-tracker->marker[nm]->colorvalue.s)
                              + 	(v_val-tracker->marker[nm]->colorvalue.v)*(v_val-tracker->marker[nm]->colorvalue.v)
                          );
                    ((float*)(tracker->marker[nm]->result->imageData + tracker->marker[nm]->result->widthStep*j))[i] = res;

                }
                else
                {
                    res = 0.0;
                }

            }
        }
        /* reset region of interest of the camera frame */
        cvResetImageROI(tracker->frame);

        /* clean up memory of intermediate images */
        cvReleaseImage(&hsv);
        cvReleaseImage(&hsv_f);
    }

}


/* finds the weighted mean (i.e. the most probable center of the marker) after the distance map has been created by createColorDiff() routine */
int locate_marker(Tracker *tracker)
{
    int marker_counter = 0;

    /* for all markers */
    for(uint8_t nm=0; nm<tracker->marker.size(); ++nm)
    {

        /* store old position for velocity calculation */
        CvPoint2D32f old_position = tracker->marker[nm]->pos_measured;
        /* store old velocity for acceleration calculation */
        CvPoint2D32f old_velocity = cvPoint2D32f(tracker->marker[nm]->vel.x,tracker->marker[nm]->vel.y);


        /* variables to hold global min and max values and their locations */
        double v_min,v_max;
        CvPoint v_minLoc,v_maxLoc;

        /**
         * Find global min and max values (value + location) of the distance map.
         * NB: min and max values have been inverted, i.e. to search for the smallest color distance, the global maximum
         * of the distance map has to be found
        **/
        cvMinMaxLoc(tracker->marker[nm]->result,&v_min,&v_max,&v_minLoc,&v_maxLoc);


        /* decide whether the marker is visible in the frame or not */
        if(v_max<1.0)
        {
            /* if maximum value in distance map is too small, the marker is not visible */
            tracker->marker[nm]->pos_is_set = 0;
            tracker->marker[nm]->vel_is_set = 0;
            tracker->marker[nm]->acc_is_set = 0;

            /* clean up memory */
            cvReleaseImage(&tracker->marker[nm]->result);
            /* and continue with next marker */
            continue;
        }

        /* if ROI is not set, find new marker position within an area around global maximum of distance map */
        if(!tracker->marker[nm]->roi_set)
        {
            /**
             * Compute the weighted mean of all values of area around global maximum.
             * The row (j) and column (i) indices are weighted with the error values of w = distance_map(i,j).
             * Add +1 to the roi indices and subtract again later to avoid neglecting first row (j=0) and first column (i=0) respectively.
            **/
            float val;
            double x_acc = 0.0;
            double y_acc = 0.0;
            double w_acc = 0.0;
	
			/* check if area around global max is within image boundaries */
            if(v_maxLoc.x-ROI_WIDTH/2>=0 && v_maxLoc.x+ROI_WIDTH/2<FRAME_WIDTH && v_maxLoc.y-ROI_HEIGHT/2>=0 && v_maxLoc.y+ROI_HEIGHT/2<FRAME_HEIGHT)
            {
				/**
	             * Compute the weighted mean of all values within the roi.
	             * The row (j) and column (i) indices are weighted with the error values of w = distance_map(i,j).
	             * Add +1 to the roi indices and subtract again later to avoid neglecting first row (j=0) and first column (i=0) respectively.
	            **/
                for(int i=v_maxLoc.x-ROI_WIDTH/2; i<v_maxLoc.x+ROI_WIDTH/2; ++i)
                {
                    for(int j=v_maxLoc.y-ROI_HEIGHT/2; j<v_maxLoc.y+ROI_HEIGHT/2; ++j)
                    {
						/* get weight value at current position */
                        val = ((float*)(tracker->marker[nm]->result->imageData + tracker->marker[nm]->result->widthStep*j))[i];

						/* increase accumulated sums in x and y direction */                        
						x_acc += (val*(double)(i+1));
                        y_acc += (val*(double)(j+1));
						/* update weight accumulator */
                        w_acc += val;

                    }
                }

                /* if sum of weights, i.e. values in distance map, is too small, marker is not detected */
                if(w_acc<=1.0e-3)
                {
                    tracker->marker[nm]->pos_is_set = 0;
                    tracker->marker[nm]->vel_is_set = 0;
                    tracker->marker[nm]->acc_is_set = 0;
                }
                /* else if marker has been located, update position, velocity and acceleration values accordingly */
                else
                {

                    tracker->marker[nm]->pos_measured.x = x_acc / w_acc - 1.0;
                    tracker->marker[nm]->pos_measured.y = y_acc / w_acc - 1.0;

					/* marker is intially located, set specific flag to 1 */
                    tracker->marker[nm]->pos_is_set = 1;
                    tracker->marker[nm]->vel_is_set = 0;
                    tracker->marker[nm]->acc_is_set = 0;

                }
            }
            else
            {
				/* marker has not been detected */
                tracker->marker[nm]->pos_is_set = 0;
                tracker->marker[nm]->vel_is_set = 0;
                tracker->marker[nm]->acc_is_set = 0;
            }
        }
        else
        {
            /**
             * Compute the weighted mean of all values within the roi.
             * The row (j) and column (i) indices are weighted with the error values of w = distance_map(i,j).
             * Add +1 to the roi indices and subtract again later to avoid neglecting first row (j=0) and first column (i=0) respectively.
            **/
            float val;
            double x_acc = 0.0;
            double y_acc = 0.0;
            double w_acc = 0.0;
            for(int i=0; i<tracker->marker[nm]->roi.width; ++i)
            {
                for(int j=0; j<tracker->marker[nm]->roi.height; ++j)
                {
                    val = ((float*)(tracker->marker[nm]->result->imageData + tracker->marker[nm]->result->widthStep*j))[i];
                    x_acc += (val*(double)(i+1));
                    y_acc += (val*(double)(j+1));
                    w_acc += val;

                }
            }

            /* if marker is not visible in roi, set flag to zero, i.e. in the next frame the whole image is searched */
            if(w_acc<=1.0e-3)
            {
                tracker->marker[nm]->pos_is_set = 0;
                tracker->marker[nm]->vel_is_set = 0;
                tracker->marker[nm]->acc_is_set = 0;
            }
            /* else if marker has been located update position, velocity and acceleration values accordingly */
            else
            {

                tracker->marker[nm]->pos_measured.x = x_acc / w_acc - 1.0 + tracker->marker[nm]->roi.x;
                tracker->marker[nm]->pos_measured.y = y_acc / w_acc - 1.0 + tracker->marker[nm]->roi.y;

                if(tracker->marker[nm]->pos_is_set)
                {
                    tracker->marker[nm]->vel.x = tracker->marker[nm]->pos_measured.x - old_position.x;
                    tracker->marker[nm]->vel.y = tracker->marker[nm]->pos_measured.y - old_position.y;
                    if(tracker->marker[nm]->vel_is_set)
                    {
                        tracker->marker[nm]->acc.x = tracker->marker[nm]->vel.x - old_velocity.x;
                        tracker->marker[nm]->acc.y = tracker->marker[nm]->vel.y - old_velocity.y;
                        tracker->marker[nm]->acc_is_set = 1;
                    }
                    tracker->marker[nm]->vel_is_set = 1;
                }

                tracker->marker[nm]->pos_is_set = 1;

            }

			/* increase marker counter */
            if(tracker->marker[nm]->pos_is_set) marker_counter++;

        }

        /* iff all markers have been detected (amount of located marker equals to total amount of markers), set tracker state to on_track */
        if(marker_counter==NUM_OF_MARKERS)
        {
            tracker->state = on_track;
        }
        /* otherwise set tracker state to off_track */
        else
        {
            tracker->state = off_track;
        }


        /* clean up memory */
        cvReleaseImage(&tracker->marker[nm]->result);


    }
    return 0;
}


