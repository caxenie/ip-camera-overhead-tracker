#include "ColorDiffTracker.h"



	
/**
 * wrapper function to aggregate calls to compute_diff_maps() and findMinimum()
 * After this routine was called, the marker position, velocity and acceleration estimates are updated
**/
void track(StereoCluster *cluster)
{

	/* if no valid frame is available, clearly something went wrong */
    if(!cluster->tracker[0]->frame || !cluster->tracker[1]->frame)
    {
        fprintf(stderr,"Fatal: no frame captured\n");
        exit(EXIT_FAILURE);
    }


    /* make sure that first tracker is connected to color cam */
    if(cluster->tracker[0]->color)
    {
        /* compute the color difference maps and search for the global minimum */
        compute_color_diff_map(cluster->tracker[0],cluster->data);
        locate_marker(cluster->tracker[0],cluster->data);
    }
    
    int nc;
    /* if color tracker is on track, locate markers in b/w frame */
    if(cluster->tracker[0]->state==ON_TRACK)
    {
        if(cluster->tracker[1]->color)
        {
            /* compute the color difference maps and search for the global minimum */
            compute_color_diff_map(cluster->tracker[0],cluster->data);
            locate_marker(cluster->tracker[0],cluster->data);
        }
        else
        {
            /* blob detection and extraction */
            nc = bw_detect_blobs(cluster->tracker[1],cluster->data);
//            printf("%d\n",nc);
            if(nc!=cluster->data->NUM_OF_MARKERS)
            {
                cluster->state = OFF_TRACK;
                return;
            }
            if(cluster->data->NUM_OF_MARKERS==1)
            {
                cluster->tracker[1]->marker[0]->pos_measured.x = cluster->tracker[1]->marker[0]->blob_pos.x;
                cluster->tracker[1]->marker[0]->pos_measured.y = cluster->tracker[1]->marker[0]->blob_pos.y;
                cluster->tracker[1]->marker[0]->pos_is_set = 1;
                cluster->tracker[1]->state = ON_TRACK;
                cluster->state = ON_TRACK;
                return;
            }
            else
            {  
                /* marker not located yet, use fundamental matrix to classify blobs */
                bw_track_blobs(cluster);
            }
            
        }
    
    }

    if(cluster->tracker[0]->state==ON_TRACK && cluster->tracker[1]->state==ON_TRACK) cluster->state = ON_TRACK;
    else cluster->state = OFF_TRACK;


}



void bw_track_blobs(StereoCluster *cluster)
{

    int *free_blobs = (int *)calloc(cluster->data->NUM_OF_MARKERS,sizeof(int));
    double *d_epi = (double *)calloc(cluster->data->NUM_OF_MARKERS,sizeof(int));
    double *d_pred = (double *)calloc(cluster->data->NUM_OF_MARKERS,sizeof(int));


    for(int k=0; k<cluster->data->NUM_OF_MARKERS; ++k)
    {
        free_blobs[k] = 1;
    }


    double nx,ny;
    double lx,ly,lz;
   
    

    double d_min;
    double d_thresh = 5.0;
    int id;
    int d_cum;
    double d;

    CvPoint2D32f old_marker_pos;


    /* compute distances between markers, if markers too close, no stable tracking possible */
    for(int m=0; m<cluster->data->NUM_OF_MARKERS-1; ++m)
    {
        for(int n=m+1; n<cluster->data->NUM_OF_MARKERS; ++n)
        {
            d = sqrt( (cluster->tracker[1]->marker[m]->blob_pos.x-cluster->tracker[1]->marker[n]->blob_pos.x)*(cluster->tracker[1]->marker[m]->blob_pos.x-cluster->tracker[1]->marker[n]->blob_pos.x) + 
                (cluster->tracker[1]->marker[m]->blob_pos.y-cluster->tracker[1]->marker[n]->blob_pos.y)*(cluster->tracker[1]->marker[m]->blob_pos.y-cluster->tracker[1]->marker[n]->blob_pos.y) );
            if(d<4.0)
            {
                cluster->tracker[1]->state = OFF_TRACK;
                cluster->state = OFF_TRACK;
                return;
            } 
        }
    }


    for(int mc=0; mc<cluster->data->NUM_OF_MARKERS; ++mc)
    {
        /* read out pixel coordinates from color camera */
        nx = cluster->tracker[0]->marker[mc]->pos_measured.x;
        ny = cluster->tracker[0]->marker[mc]->pos_measured.y;
        
        /* compute epipolar line for right image*/
        lx = cluster->FundamentalMatrix[0][0]*nx + cluster->FundamentalMatrix[0][1]*ny + cluster->FundamentalMatrix[0][2];
        ly = cluster->FundamentalMatrix[1][0]*nx + cluster->FundamentalMatrix[1][1]*ny + cluster->FundamentalMatrix[1][2];
        lz = cluster->FundamentalMatrix[2][0]*nx + cluster->FundamentalMatrix[2][1]*ny + cluster->FundamentalMatrix[2][2];
        
        lx /= ly;
        lz /= ly;
        ly = 1.0;

        if(mc==0)
        {
            double x0,x1,y0,y1;
            x0 = 0.0;
            x1 = 779.0;
            y0 = -(lx*x0+lz);
            y1 = -(lx*x1+lz);
//          printf("%f %f %f %f\n",x0,y0,x1,y1);
            cvLine(cluster->tracker[1]->frame,cvPoint((int)x0,(int)y0),cvPoint((int)x1,(int)y1),cvScalarAll(255),1,CV_AA);
        }
#if 0
        if(cluster->tracker[1]->marker[mc]->pos_is_set)
        {

            old_marker_pos.x = cluster->tracker[1]->marker[mc]->pos_measured.x;
            old_marker_pos.y = cluster->tracker[1]->marker[mc]->pos_measured.y;


    //      printf("m = (%f %f)\n",tracker->marker[nm]->pos_measured.x,tracker->marker[nm]->pos_measured.y);

            if(cluster->tracker[1]->marker[mc]->vel_is_set)
            {
                /* if marker velocity is known predict new position */
                cluster->tracker[1]->marker[mc]->pos_predicted.x = cluster->tracker[1]->marker[mc]->pos_measured.x + cluster->tracker[1]->marker[mc]->vel.x;
                cluster->tracker[1]->marker[mc]->pos_predicted.y = cluster->tracker[1]->marker[mc]->pos_measured.y + cluster->tracker[1]->marker[mc]->vel.y;
            }
            else
            {
                /* otherwise take last known position for the center of the ROI as best guess */
                cluster->tracker[1]->marker[mc]->pos_predicted.x = cluster->tracker[1]->marker[mc]->pos_measured.x;
                cluster->tracker[1]->marker[mc]->pos_predicted.y = cluster->tracker[1]->marker[mc]->pos_measured.y;
            }

            

            for(int blob=0; blob<cluster->data->NUM_OF_MARKERS; ++blob)
            {
                 /* distance of blob to epipolar line */
                d_epi[blob] = abs( (lx*cluster->tracker[1]->marker[blob]->blob_pos.x + cluster->tracker[1]->marker[blob]->blob_pos.y + lz) / sqrt(1.0 + lx*lx) );
                /*
                d_pred[blob] = sqrt( (cluster->tracker[1]->marker[mc]->pos_predicted.x-cluster->tracker[1]->marker[blob]->blob_pos.x)*(cluster->tracker[1]->marker[mc]->pos_predicted.x-cluster->tracker[1]->marker[blob]->blob_pos.x) + 
                    (cluster->tracker[1]->marker[mc]->blob_pos.y-cluster->tracker[1]->marker[blob]->pos_predicted.y)*(cluster->tracker[1]->marker[mc]->blob_pos.y-cluster->tracker[1]->marker[blob]->pos_predicted.y) );
                */
            }

        }
#endif
        
        d_cum = 0;
        d_min = 1e6;
        id = 77;
        for(int blob=0; blob<cluster->data->NUM_OF_MARKERS; ++blob)
        {
             /* distance of blob to epipolar line */
            d_epi[blob] = abs( (lx*cluster->tracker[1]->marker[blob]->blob_pos.x + cluster->tracker[1]->marker[blob]->blob_pos.y + lz) / sqrt(1.0 + lx*lx) );
//          printf("%f\n",d_epi[blob]);
            if(d_epi[blob] < d_thresh)
            {
                d_cum++;
            }
            if(d_cum > 1)
            {
                cluster->state = OFF_TRACK;
                cluster->tracker[1]->state = OFF_TRACK;
                return;
            }
            if(d_epi[blob]<d_min)
            {
                d_min = d_epi[blob];
                id = blob;
            }

        }
        
        if(id == 77)
        {
            cluster->state = OFF_TRACK;
            cluster->tracker[1]->state = OFF_TRACK;
            return;
        }
        else
        {
            cluster->tracker[1]->marker[mc]->pos_measured.x = cluster->tracker[1]->marker[id]->blob_pos.x;
            cluster->tracker[1]->marker[mc]->pos_measured.y = cluster->tracker[1]->marker[id]->blob_pos.y;
            cluster->tracker[1]->marker[mc]->pos_is_set = 1;
        }

    }

    cluster->tracker[1]->state = ON_TRACK;
    cluster->state = ON_TRACK;


    
}



int bw_detect_blobs(Tracker *tracker, struct StaticData *data)
{

    /* circular kernel for dilation */
    IplConvKernel *kernel = cvCreateStructuringElementEx(3,3,1,1,CV_SHAPE_ELLIPSE);

    /* temporary image to hold thresholded camera frame */
    IplImage *thresh = cvCreateImage(cvGetSize(tracker->frame),IPL_DEPTH_8U,1);

    /* variables for contour finding */
    CvMemStorage *mem = cvCreateMemStorage(0);
    CvSeq *contour;
    CvMoments moments;
    int it;


    /**
     * preprocessing 
    **/
    /* threshold image, reasonably stable since frame is highly underexposed and LEDs are very bright */
    cvThreshold(tracker->frame,thresh,180,255,CV_THRESH_BINARY);

    /* Dilate image to increase size of responses from thresholding, gives more stable result in contour finding*/
    cvDilate(thresh,thresh,kernel,2);


//  cvShowImage("thresh",thresh);


    /**
     * blob extraction (connected component finding)
    **/
    /* find contours in image, should give one contour for each markers */
    int nc = cvFindContours(thresh,mem,&contour,sizeof(CvContour),CV_RETR_LIST,CV_CHAIN_APPROX_SIMPLE);

//    printf("nc = %d\n",nc);

    it = 0;
    /* if NUM_OF_MARKERS contours detected, compute mean position of each contour */
    if(nc==data->NUM_OF_MARKERS)
    {
        if(contour)
        {
//            cvDrawContours(thresh,contour,cvScalarAll(255),cvScalarAll(0),100);
            CvSeq *c;
            for(c=contour; c!=NULL; c=c->h_next)
            {
                /* compute moments for each contour */
                cvContourMoments(c,&moments);
                /* make sure the contour encloses some area */
                if(moments.m00>0.0)
                {
                    /* compute center of mass -> mean blob position */
                    /* even though the computed position is stored in the marker structs, it doesn't neccessarily correspond to that specific marker */
                    tracker->marker[it]->blob_pos.x = moments.m10/moments.m00;
                    tracker->marker[it]->blob_pos.y = moments.m01/moments.m00;
//                    printf("(%f %f)\n",tracker->marker[it]->blob_pos.x,tracker->marker[it]->blob_pos.y);
                }
                else
                {
                    /* for stable marker recognition all markers must have been detected */
                    tracker->state = OFF_TRACK;
                    break;
                }
                it++;

            }
        }
    }
    else
    {
        tracker->state = OFF_TRACK;
        for(int nm=0; nm<data->NUM_OF_MARKERS; ++nm)
        {
            tracker->marker[nm]->pos_is_set = 0;
            tracker->marker[nm]->blob_pos.x = 0;
            tracker->marker[nm]->blob_pos.y = 0;
        } 
    }

    /* clean up memory */
    cvReleaseMemStorage(&mem);
    cvReleaseImage(&thresh);


    return nc;
}


/* computes the color difference map for the region of interest */
void compute_color_diff_map(Tracker *tracker, struct StaticData *data)
{

    CvSize sz;

    /* for all markers (LEDs) */
    for(int nm=0; nm<data->NUM_OF_MARKERS; ++nm)
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
int locate_marker(Tracker *tracker, struct StaticData *data)
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
        if(marker_counter==data->NUM_OF_MARKERS)
        {
            tracker->state = ON_TRACK;
        }
        /* otherwise set tracker state to off_track */
        else
        {
            tracker->state = OFF_TRACK;
        }


        /* clean up memory */
        cvReleaseImage(&tracker->marker[nm]->result);


    }
    return 0;
}

