/* Barrel un-distortion */
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <stdio.h> 
#include <gtk/gtk.h>
#include <string.h>

#define K1_MAX 0.0006
#define K2_MAX 0.0003

char* default_filename;

typedef struct _app_data AppData;
struct _app_data
{
	/* gtk widgets */
	GtkWidget *main_win;
	GtkImage  *input_image;
	GtkImage  *output_image;
	/* opencv image containers */	
	IplImage  *cv_input_img;
	IplImage  *cv_output_img;
	/* de-distortion params */
	double k1;
	double k2;
};

void undistort_image( AppData *data)
{
	GdkPixbuf *px;
	IplImage  *in, *out;
	
	in = data->cv_input_img;
	out = data->cv_output_img;

	/* barell-distort  processing */
	double centerX = ((out->width)/2);
	double centerY = ((out->height)/2);
	double ex = 0, ey = 0, r = 0, f = 0;
	double newX = 0.0f, newY = 0.0f;
	
	/* go through the input image pixels */
	for(int i = 0; i < out->height; i++){
		for(int j = 0; j < out->width; j++){
			ex = j - centerX;
			ey = i - centerY;
			r = sqrt(ex*ex + ey*ey);
			f = (data->k2*r*r + data->k1*r + 1);			
			newX = centerX + ex*f;
			newY = centerY + ey*f;
			if(newY > 0 && newY <= out->height){
				if(newX > 0 && newX <= out->width){
					cvSet2D(out, newY, newX, cvGet2D(in, i, j));
				}
			}
		}
	}
	
	/* as opencv is using BGR convert to RGB for gtk */
	cvCvtColor(out, out, CV_BGR2RGB);
	/* transform output iplimage to gtkimage */
	px = gdk_pixbuf_new_from_data(
		(guchar*)out->imageData,
		GDK_COLORSPACE_RGB,
		FALSE,
		out->depth,
		out->width,
		out->height,
		out->widthStep,
		NULL,
		NULL);
	/* update the image in the window */
	gtk_image_set_from_pixbuf(data->output_image, px);
}

/* callback for the k1 factor adjustment */
G_MODULE_EXPORT void on_adjustment1_value_changed(GtkAdjustment *adj, AppData *data)
{
	gtk_adjustment_set_upper(adj, K1_MAX);
	/* check data */
	if(data==NULL){
		fprintf(stderr, "WARNING: No app data !\n");
		return;
	}
	/* get the value and set the parameter for the output */
	data->k1 = (double)gtk_adjustment_get_value(adj);	
	undistort_image(data);
}

/* callback for the k2 factor adjustment */
G_MODULE_EXPORT void on_adjustment2_value_changed(GtkAdjustment *adj, AppData *data)
{
        gtk_adjustment_set_upper(adj, K2_MAX);
	/* check data */
        if(data==NULL){
                fprintf(stderr, "WARNING: No app data !\n");
                return;
        }       
        /* get the value and set the parameter for the output */
        data->k2 = (double)gtk_adjustment_get_value(adj);
	undistort_image(data);
}

/* callback for quiting the app and saving the output img */
G_MODULE_EXPORT void on_window_destroy(GtkObject *object, AppData *data)
{
	/* params dump into a file */
	char *param_file = (char*)calloc(30, sizeof(char));
	strcpy(param_file, default_filename);
	FILE *f = fopen(strcat(param_file, "-undistort-params"), "w");
	/* setup the save params */
        int* save_params = (int *)calloc(3, sizeof(int));
	save_params[0] = CV_IMWRITE_JPEG_QUALITY;
	save_params[1] = 100;
	save_params[2] = 0;

	/* check data */
        if(data==NULL){
                fprintf(stderr, "WARNING: No app data !\n");
                return;
        }
        /* write output to disk after converting to the proper color system */
	cvCvtColor(data->cv_output_img, data->cv_output_img, CV_BGR2RGB);
	cvSaveImage(strcat(default_filename, "-undistorted"), data->cv_output_img, save_params);
	fprintf(f, "k1=%10.20lf\nk2=%10.20lf\n", data->k1, data->k2);
	/* quit the main event loop */
	gtk_main_quit();
}

int 
main( int argc, char **argv )
{
	default_filename = argv[1];

	/* interface to create GUI, error and app data */
	GtkBuilder *builder;
	GError *err = NULL;
	AppData *data;

	/* init gtk system */
	gtk_init(&argc, &argv);
		
	/* create data, load input image, resize it and create output image */
	data = g_slice_new(AppData);
	data->cv_input_img = cvLoadImage(default_filename, CV_LOAD_IMAGE_COLOR);
	if(data->cv_input_img == NULL){
		fprintf(stderr, "ERROR: Couldn't load image! \n");
		return EXIT_FAILURE;
	}

	data->cv_output_img = cvCreateImage(cvGetSize(data->cv_input_img), data->cv_input_img->depth, data->cv_input_img->nChannels);
	if(data->cv_output_img == NULL){
		fprintf(stderr, "ERROR: Couldn't create output image! \n");
		return EXIT_FAILURE;
	}
	
	/* create the gtk builder and load the gui from the glade file */
	builder = gtk_builder_new();
	if(!gtk_builder_add_from_file(builder, "cam_track_undistort.glade", &err)){
		g_warning("%s", err->message);
		g_free(err);
		return EXIT_FAILURE;
	}
	
	/* get the main window pointer from the gui */
	data->main_win = GTK_WIDGET(gtk_builder_get_object(builder, "window"));
	data->input_image = GTK_IMAGE(gtk_builder_get_object(builder, "image_in"));
	data->output_image = GTK_IMAGE(gtk_builder_get_object(builder, "image_out"));

	/* init both input and output with the same image */
	gtk_image_set_from_file(data->input_image, default_filename);
	gtk_image_set_from_file(data->output_image, default_filename);

	/* set the default params */
	data->k1 = 0.0;
	data->k2 = 0.0;
	
	/* execute first de-distortion run */
	undistort_image(data);

	/* connect callbacks for params changes and destroy builder */
	gtk_builder_connect_signals(builder, data);
	g_object_unref(G_OBJECT(builder));

	/* show window */
	gtk_widget_show(data->main_win);

	/* start the gtk event loop */
	gtk_main();

	return  EXIT_SUCCESS;
}

