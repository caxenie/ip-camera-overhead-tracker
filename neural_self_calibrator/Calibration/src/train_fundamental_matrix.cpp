#include <iostream>
#include <fstream>
#include <string.h>

#include "../../src/global.h"
#include <fann.h>
#include <sys/time.h>


using namespace std;

typedef struct _Point
{
	double u;
	double v;
}Point;

/**
 * Position set structure:
 * Image pixel coordinates of two markers from two distinct cameras are aggregated in this struct.
**/
typedef struct _PositionSet
{
	/* marker[no of sensors] */
	Point marker[2];
}PositionSet;

/* read logged marker coordinates from file and store as vector of Position Sets*/
int readPixelCoordinatesFromFile(char *file1, char *file2, CvMat *pts1, CvMat *pts2);

void norm_coord(CvMat *pts, CvMat *pts_n, CvMat *T, int N);

int line_count(char *file1);

void print_weights(struct fann *ann);
void get_weights(struct fann *ann, double *a);
void create_fundmat_from_weights(struct fann *ann, CvMat *f);

struct fann_train_data *create_training_data(CvMat *pts1, CvMat *pts2, int no_of_possets);



/* main entry point */
int main(int argc, char *argv[])
{


	/* check for valid command line parameters */
	if(argc!=3)
	{
		cout << "./neural_calib <file 1> <file 2>" << endl;
		return 0;
	}

	int no_of_possets;
	no_of_possets = line_count(argv[1]);

	CvMat *pts1 = cvCreateMat(2,no_of_possets,CV_64FC1);
	CvMat *pts2 = cvCreateMat(2,no_of_possets,CV_64FC1);
	
	/* create position sets -> read pixel coordinates from file*/
	readPixelCoordinatesFromFile(argv[1], argv[2], pts1, pts2);
	
	printf("%d samples read from file\n",no_of_possets);


	/* scaling matrices due to coordinate normalization */
	CvMat *T1;
	CvMat *T2;
	T1 = cvCreateMat(3,3,CV_64FC1);
	T2 = cvCreateMat(3,3,CV_64FC1);

	CvMat *pts1_norm = cvCreateMat(2,no_of_possets,CV_64FC1);
	CvMat *pts2_norm = cvCreateMat(2,no_of_possets,CV_64FC1);

	/* normalize coordinates */
	norm_coord(pts1,pts1_norm,T1,no_of_possets);
	norm_coord(pts2,pts2_norm,T2,no_of_possets);

#if 0
	cout << "T1" << endl;
	for(int rows=0; rows<3; rows++)
	{	
		for(int cols=0; cols<3; cols++)
		{
			printf("%12.9f ",cvGetReal2D(T1,rows,cols));
		}
		printf("\n");
	}
	cout << endl;
	cout << "T2" << endl;
	for(int rows=0; rows<3; rows++)
	{	
		for(int cols=0; cols<3; cols++)
		{
			printf("%12.9f ",cvGetReal2D(T2,rows,cols));
		}
		printf("\n");
	}
#endif

	CvMat *fundamental;
	CvMat *fundamental_tilde;
	CvMat *tmp;
	CvMat *T2_trans;

	fundamental_tilde = cvCreateMat(3,3,CV_64FC1);
	fundamental = cvCreateMat(3,3,CV_64FC1);
	tmp = cvCreateMat(3,3,CV_64FC1);
	T2_trans = cvCreateMat(3,3,CV_64FC1);

	
	/* compute fundamental matrix */
	int r = cvFindFundamentalMat(pts1_norm,pts2_norm,fundamental_tilde,CV_FM_RANSAC,1.0,0.99);
	printf("%d\n",r);

	cvTranspose(T2,T2_trans);
	cvMatMul(T2_trans,fundamental_tilde,tmp);
	cvMatMul(tmp,T1,fundamental);



	
	double scale = cvGetReal2D(fundamental,2,2);
	for(int rows=0; rows<3; rows++)
	{	
		for(int cols=0; cols<3; cols++)
		{
			printf("%12.9f ",cvGetReal2D(fundamental,rows,cols)/scale);
		}
		printf("\n");
	}

	cvTranspose(fundamental,fundamental);
	double e = 0.0;
	for(int p=0; p<no_of_possets; ++p)
	{
		/*
		e +=	fabs(
				cvGetReal2D(pts1,0,p)*cvGetReal2D(pts2,0,p)*cvGetReal2D(fundamental,0,0) + 
				cvGetReal2D(pts1,1,p)*cvGetReal2D(pts2,0,p)*cvGetReal2D(fundamental,0,1) + 
				cvGetReal2D(pts2,0,p)*cvGetReal2D(fundamental,0,2) + 
				cvGetReal2D(pts1,0,p)*cvGetReal2D(pts2,1,p)*cvGetReal2D(fundamental,1,0) + 
				cvGetReal2D(pts1,1,p)*cvGetReal2D(pts2,1,p)*cvGetReal2D(fundamental,1,1) + 
				cvGetReal2D(pts2,1,p)*cvGetReal2D(fundamental,1,2) + 
				cvGetReal2D(pts1,0,p)*cvGetReal2D(fundamental,2,0) + 
				cvGetReal2D(pts1,1,p)*cvGetReal2D(fundamental,2,1) + 
				cvGetReal2D(fundamental,2,2)
				);
		*/
		e +=	fabs(
				cvGetReal2D(pts1,0,p)*
					( cvGetReal2D(pts2,0,p)*cvGetReal2D(fundamental,0,0) + cvGetReal2D(pts2,1,p)*cvGetReal2D(fundamental,0,1) + cvGetReal2D(fundamental,0,2) ) + 
				cvGetReal2D(pts1,1,p)*
					( cvGetReal2D(pts2,0,p)*cvGetReal2D(fundamental,1,0) + cvGetReal2D(pts2,1,p)*cvGetReal2D(fundamental,1,1) + cvGetReal2D(fundamental,1,2) ) + 
				cvGetReal2D(pts2,0,p)*cvGetReal2D(fundamental,2,0) + cvGetReal2D(pts2,1,p)*cvGetReal2D(fundamental,2,1) + cvGetReal2D(fundamental,2,2)
				);
		
	}
	e /= no_of_possets;
	printf("%lf\n",e);


	struct fann *ann = fann_create_standard(2,8,1);

	fann_set_activation_function_output(ann, FANN_LINEAR);

	fann_set_activation_steepness_output(ann,0.5);

	fann_randomize_weights(ann,0.1,1.1);

	fann_set_training_algorithm(ann,FANN_TRAIN_RPROP);

	struct fann_train_data *data = create_training_data(pts1_norm, pts2_norm, no_of_possets);

	fann_train_on_data(ann,data,50000,500,5e-7);

	/*

	fann_type *input = (fann_type *)calloc(9,sizeof(fann_type));
	fann_type *output = (fann_type *)calloc(1,sizeof(fann_type));;


	double mse = 0;
	for(int it=0;;it++)
	{
		for(int s=0; s<no_of_possets; ++s)
		{
			input[0] = cvGetReal2D(pts1_norm,0,s) * cvGetReal2D(pts2_norm,0,s);
			input[1] = cvGetReal2D(pts1_norm,1,s) * cvGetReal2D(pts2_norm,0,s);
			input[2] = cvGetReal2D(pts2_norm,0,s);
			input[3] = cvGetReal2D(pts1_norm,0,s) * cvGetReal2D(pts2_norm,1,s);
			input[4] = cvGetReal2D(pts1_norm,1,s) * cvGetReal2D(pts2_norm,1,s);
			input[5] = cvGetReal2D(pts2_norm,1,s);
			input[6] = cvGetReal2D(pts1_norm,0,s);
			input[7] = cvGetReal2D(pts1_norm,1,s);
			input[8] = 1;
			output[0] = 0;
			fann_train(ann,input,output);
		}
		if(it%1000==0)
		{
			mse = fann_get_MSE(ann);
			cout << it << ": " << mse << endl;
			if(mse<1e-8) break;
		}
	}
	*/


	create_fundmat_from_weights(ann, fundamental_tilde);

	cvTranspose(T2,T2_trans);
	cvMatMul(T2_trans,fundamental_tilde,tmp);
	cvMatMul(tmp,T1,fundamental);


	FILE *fun = fopen("fundamental.txt","w");

	scale = cvGetReal2D(fundamental,2,2); //cvGetReal2D(fundamental,2,2);
	for(int rows=0; rows<3; rows++)
	{	
		for(int cols=0; cols<3; cols++)
		{
			printf("%12.9f ",cvGetReal2D(fundamental,rows,cols)/scale);
			fprintf(fun,"%.10f\n",cvGetReal2D(fundamental,rows,cols)/scale);
		}
		printf("\n");
	}

	e = 0;
	for(int p=0; p<no_of_possets; ++p)
	{
		e +=	fabs(
				cvGetReal2D(pts1,0,p)*cvGetReal2D(pts2,0,p)*cvGetReal2D(fundamental,0,0) + 
				cvGetReal2D(pts1,1,p)*cvGetReal2D(pts2,0,p)*cvGetReal2D(fundamental,0,1) + 
				cvGetReal2D(pts2,0,p)*cvGetReal2D(fundamental,0,2) + 
				cvGetReal2D(pts1,0,p)*cvGetReal2D(pts2,1,p)*cvGetReal2D(fundamental,1,0) + 
				cvGetReal2D(pts1,1,p)*cvGetReal2D(pts2,1,p)*cvGetReal2D(fundamental,1,1) + 
				cvGetReal2D(pts2,1,p)*cvGetReal2D(fundamental,1,2) + 
				cvGetReal2D(pts1,0,p)*cvGetReal2D(fundamental,2,0) + 
				cvGetReal2D(pts1,1,p)*cvGetReal2D(fundamental,2,1) + 
				cvGetReal2D(fundamental,2,2)
				);
	}
	e /= no_of_possets;
	printf("%lf\n",e);


	fann_destroy(ann);


	exit(0);
	
}

struct fann_train_data *create_training_data(CvMat *pts1, CvMat *pts2, int no_of_possets)
{	
	int num_input = 8;
	int num_output = 1;

	struct fann_train_data *data = fann_create_train(no_of_possets, num_input, num_output);

	for(int s=0; s<no_of_possets; ++s)
	{
		data->input[s][0] = cvGetReal2D(pts1,0,s) * cvGetReal2D(pts2,0,s);
		data->input[s][1] = cvGetReal2D(pts1,1,s) * cvGetReal2D(pts2,0,s);
		data->input[s][2] = cvGetReal2D(pts2,0,s);
		data->input[s][3] = cvGetReal2D(pts1,0,s) * cvGetReal2D(pts2,1,s);
		data->input[s][4] = cvGetReal2D(pts1,1,s) * cvGetReal2D(pts2,1,s);
		data->input[s][5] = cvGetReal2D(pts2,1,s);
		data->input[s][6] = cvGetReal2D(pts1,0,s);
		data->input[s][7] = cvGetReal2D(pts1,1,s);
		data->output[s][0] = 0;

	}

	return data;
}



int line_count(char *file1)
{
	std::string in_line;

	/* open files and check for errors */
	ifstream cam0_data_f(file1);
	if(!cam0_data_f.good() )
	{
		std::cout << "Not able to open file 1" << std::endl;
		exit(-1);
	}

	int lines = 0;
	/* count number of lines */
	while(std::getline(cam0_data_f,in_line))
		lines++;

	cam0_data_f.close();

	return lines;

}


int readPixelCoordinatesFromFile(char *file1, char *file2, CvMat *pts1, CvMat *pts2)
{
	
	std::string in_line;

	/* open files and check for errors */
	ifstream cam0_data_f(file1);
	if(!cam0_data_f.good() )
	{
		std::cout << "Not able to open file 1" << std::endl;
		exit(-1);
	}
	ifstream cam1_data_f(file2);
	if(!cam1_data_f.good() )
	{
		std::cout << "Not able to open file 2" << std::endl;
		exit(-1);
	}

	int sample_counter = 0;
	size_t ofs_1st;
	size_t ofs_2nd;
	size_t start;


	/**
	 * Synthetic data:	left cam ~ cam0_log 
	 					right cam ~ cam1_log

	 * Real data: 		left cam ~ cam1_log 
	 					right cam ~ cam0 log
	**/
	while(cam0_data_f.good() && cam1_data_f.good())
	{

		std::getline(cam1_data_f,in_line);
		if(!cam1_data_f.eof())
		{
			start = in_line.find_first_not_of(' ');
			ofs_1st = in_line.find(' ',start+1);
			ofs_2nd = in_line.find(' ',ofs_1st+1);

			cvSetReal2D( pts1,0,sample_counter,
				std::strtof(in_line.substr(0,ofs_1st).c_str(),NULL) );
			cvSetReal2D( pts1,1,sample_counter,
				std::strtof(in_line.substr(ofs_1st+1,ofs_2nd).c_str(),NULL) );
		}
		
		std::getline(cam0_data_f,in_line);
		if(!cam0_data_f.eof())
		{
			start = in_line.find_first_not_of(' ');
			ofs_1st = in_line.find(' ',start+1);
			ofs_2nd = in_line.find(' ',ofs_1st+1);

			cvSetReal2D(pts2,0,sample_counter,std::strtof(in_line.substr(0,ofs_1st).c_str(),NULL));
			cvSetReal2D(pts2,1,sample_counter,std::strtof(in_line.substr(ofs_1st+1,ofs_2nd).c_str(),NULL));
		}
			
		sample_counter ++;

	}


	return 1;
}

void norm_coord(CvMat *pts, CvMat *pts_n, CvMat *T, int N)
{
	double u_m = 0;
	double v_m = 0;

	for(int n=0; n<N; ++n)
	{
		u_m += cvGetReal2D(pts,0,n);
		v_m += cvGetReal2D(pts,1,n);
	}

	u_m /= (double)N;
	v_m /= (double)N;

	CvMat *pts_m = cvCreateMat(2,N,CV_64FC1);

	for(int n=0; n<N; ++n)
	{
		cvSetReal2D(pts_m,0,n,cvGetReal2D(pts,0,n)-u_m);
		cvSetReal2D(pts_m,1,n,cvGetReal2D(pts,1,n)-v_m);
	}

	double d = 0;

	for(int n=0; n<N; ++n)
	{
		d += sqrt( cvGetReal2D(pts_m,0,n)*cvGetReal2D(pts_m,0,n) + cvGetReal2D(pts_m,1,n)*cvGetReal2D(pts_m,1,n) );
	}
	d /= (double)N;

	double s = sqrt(2)/d;

	for(int n=0; n<N; ++n)
	{
		cvSetReal2D(pts_n,0,n,s*cvGetReal2D(pts_m,0,n));	
		cvSetReal2D(pts_n,1,n,s*cvGetReal2D(pts_m,1,n));	
	}

	cvSetReal2D(T,0,0,s);
	cvSetReal2D(T,0,1,0.0);
	cvSetReal2D(T,0,2,-s*u_m);
	cvSetReal2D(T,1,0,0.0);
	cvSetReal2D(T,1,1,s);
	cvSetReal2D(T,1,2,-s*v_m);
	cvSetReal2D(T,2,0,0.0);
	cvSetReal2D(T,2,1,0.0);
	cvSetReal2D(T,2,2,1.0);

}



void get_weights(struct fann *ann, double *a)
{
	fann_type *weights = ann->weights;
	fann_type *last_weight = weights + ann->total_connections;

	int i = 0;

	for(;weights!=last_weight;weights++)
	{
		a[i] = *weights;
		i++;
	}
}

void print_weights(struct fann *ann)
{
	FILE *fd = fopen("fundamental.txt","w");
	fann_type *weights = ann->weights;
	fann_type *last_weight = weights + ann->total_connections;

	for(;weights!=last_weight;weights++)
	{
		printf("%f\n",*weights);
		fprintf(fd,"%f\n",*weights);
	}
}

void create_fundmat_from_weights(struct fann *ann, CvMat *f)
{
	fann_type *weights = ann->weights;
	fann_type *last_weight = weights + ann->total_connections;

	int i = 0;

	int a = 0;
	int b = 0;

	for(;weights!=last_weight;weights++)
	{
		if(i==9) break;

		a = i/3;
		b = i%3;

		cvSetReal2D(f,a,b,*weights);
		i++;
	}
}