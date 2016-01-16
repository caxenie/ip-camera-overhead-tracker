#include <iostream>
#include <fstream>

#include "../../src/global.h"
#include <fann.h>
#include <sys/time.h>

#define TARGET_DISTANCE 0.157

using namespace std;

typedef struct _Point
{
	double u;
	double v;
}Point;

/**
 * Position set structure:
 * Image pixel coordinate of two markers from two distinct cameras are aggregated in this struct.
**/
typedef struct _PositionSet
{
	/* marker[no of markers][no of sensors] */
	Point marker[2][2];
}PositionSet;

/* read logged marker coordinates from file and store as vector of Position Sets*/
vector<PositionSet*> createPositionSetFromFile(char *file1, char *file2);


/* main entry point */
int main(int argc, char *argv[])
{
	
	/* check for valid command line parameters */
	if(argc!=3)
	{
		cout << "./neural_calib <file 1> <file 2>" << endl;
		return 0;
	}


	/* create position sets */
	vector<PositionSet*> positionSet = createPositionSetFromFile(argv[1], argv[2]);
	

	/* determine total amount of position sets */
	int no_of_possets = positionSet.size();
	printf("%d\n",no_of_possets);
	
	/* create and init neural network */
	struct fann *ann = fann_create_from_file("cam_tracker.net");
	if(!ann)
	{
		printf("Unabel to open ANN\n");
		return 0;
	}
	


	FILE *acc_fd = fopen("accuracy.txt","w");
	if(!acc_fd)
	{
		printf("Unable to open file\n");
		return 0;
	}
	
	/* create variables to store in- and output of network */
	fann_type *input_marker1;
	fann_type *input_marker2;
	fann_type *output_marker1;
	fann_type *output_marker2;
	/* allocate memory */
	input_marker1 = (fann_type *) calloc(4,sizeof(fann_type));		// 4 inputs for marker 1
	input_marker2 = (fann_type *) calloc(4,sizeof(fann_type));		// 4 inputs for marker 2
	output_marker1 = (fann_type *) calloc(3,sizeof(fann_type)); 	// 3 outputs for marker 1
	output_marker2 = (fann_type *) calloc(3,sizeof(fann_type)); 	// 3 outputs for marker 2


	double u0,v0,u1,v1,d,acc;
	double x_m1,y_m1,z_m1,x_m2,y_m2,z_m2;

	/* for every position set */
	for(int i=0; i<no_of_possets; ++i)
	{
		input_marker1[0] = positionSet[i]->marker[0][0].u;
		input_marker1[1] = positionSet[i]->marker[0][0].v;
		input_marker1[2] = positionSet[i]->marker[0][1].u;
		input_marker1[3] = positionSet[i]->marker[0][1].v;

		input_marker2[0] = positionSet[i]->marker[1][0].u;
		input_marker2[1] = positionSet[i]->marker[1][0].v;
		input_marker2[2] = positionSet[i]->marker[1][1].u;
		input_marker2[3] = positionSet[i]->marker[1][1].v;
		
		
		/* apply coordinates of marker 1 and store output */
		output_marker1 = fann_run(ann,input_marker1);
		x_m1 = output_marker1[0];
		y_m1 = output_marker1[1];
		z_m1 = output_marker1[2];
				
		/* apply coordinates of marker 2 and store output */
		output_marker2 = fann_run(ann,input_marker2);
		x_m2 = output_marker2[0];
		y_m2 = output_marker2[1];
		z_m2 = output_marker2[2];

		
		
		/* compute (input/target output) set such that the target output of the neural net reflects the actual target distance */
		/* distance between markers as computed by the network*/
		d = sqrt(
			(x_m2-x_m1)*(x_m2-x_m1) +			//(x2-x1)^2
			(y_m2-y_m1)*(y_m2-y_m1) +			//(y2-y1)^2
			(z_m2-z_m1)*(z_m2-z_m1) 			//(z2-z1)^2
			);
		acc = sqrt((d-TARGET_DISTANCE)*(d-TARGET_DISTANCE));
		u0 = 0.5*(positionSet[i]->marker[0][0].u + positionSet[i]->marker[0][1].u);
		v0 = 0.5*(positionSet[i]->marker[0][0].v + positionSet[i]->marker[0][1].v);
		u1 = 0.5*(positionSet[i]->marker[1][0].u + positionSet[i]->marker[1][1].u);
		v1 = 0.5*(positionSet[i]->marker[1][0].v + positionSet[i]->marker[1][1].v);
		fprintf(acc_fd,"%f %f %f %f %f\n",u0,v0,u1,v1,acc);

	}

	
	//clean up
	fann_destroy(ann);

	exit(0);
	
}

vector<PositionSet*> createPositionSetFromFile(char *file1, char *file2)
{
	
	/** 
	 *  data is stored in vector of PositionSet, where one position set consists of m x n elements
	 *  where m is number of markers and n is number of sensors, i.e. the pixel coordinates of each marker in each frame
	**/
	vector<PositionSet*> positionSet;
	PositionSet *temp;

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

	/* write data from file into position set vector */
	while(cam0_data_f.good() && cam1_data_f.good())
	{
	
		temp = new PositionSet;

		/* cam0 */
		cam0_data_f >> temp->marker[0][0].u >> temp->marker[0][0].v
				>> temp->marker[1][0].u >> temp->marker[1][0].v;
		/* cam1 */
		cam1_data_f >> temp->marker[0][1].u >> temp->marker[0][1].v
				>> temp->marker[1][1].u >> temp->marker[1][1].v;

		/* push back data into vector */
		positionSet.push_back(temp);

		if(cam1_data_f.eof()) break;
	}

	//remove last element of vector since  the previous code reads the last line of the file twice
	positionSet.pop_back();

	return positionSet;
}


