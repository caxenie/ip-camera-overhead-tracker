#include <iostream>
#include <fstream>

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
 * Image pixel coordinate of two markers from two distinct cameras are aggregated in this struct.
**/
typedef struct _PositionSet
{
	/* marker[no of markers][no of sensors] */
	Point marker[2][2];
}PositionSet;

/* read logged marker coordinates from file and store as vector of Position Sets*/
vector<PositionSet*> createPositionSetFromFile(char *file1, char *file2);

/* compute new set of input/output patterns for training */
struct fann_train_data *createData(struct fann *ann, vector<PositionSet*> positionSet, int no_of_possets);

/* apply simulated annealing every n-th epoch to avoid getting stuck into local minimum */
void simulated_annealing(struct fann *ann, int epoch);

/* helper function for simulated annealing */
double get_weight(struct fann *ann, unsigned int from_neuron, unsigned int to_neuron);


void create_accuracy_map(struct fann *ann, vector<PositionSet *> positionSet, int no_of_possets);


/* main entry point */
int main(int argc, char *argv[])
{
	/* init random seed for simulated annealing */
	srand(time(NULL));

	/* check for valid command line parameters */
	if(argc!=3)
	{
		cout << "./neural_calib <file 1> <file 2>" << endl;
		return 0;
	}

	/* timing variables */
	struct timeval start, end, tmp;
	uint64_t t_diff;

	/* create position sets */
	vector<PositionSet*> positionSet = createPositionSetFromFile(argv[1], argv[2]);

	/* determine total amount of position sets */
	int no_of_possets = positionSet.size();


	
	/* create and init neural network */
	struct fann *ann = fann_create_standard(4,4,20,20,3);
	/* during training the network with the smallest mse is stored in min_ann */
	struct fann *min_ann;

	/* set hidden activation functions to sigmoidal and output activation functions to linear */
	fann_set_activation_function_hidden(ann, FANN_SIGMOID_SYMMETRIC);
	fann_set_activation_function_output(ann, FANN_LINEAR);


	/* set bit fail thresh: no output neuron should diverge more than 1 cm from expected one */	
	fann_set_bit_fail_limit(ann,0.01);

	int i = 0;

	double max_mse = 0.5e-7;
	double mse = 1e6;
	double min_mse = 1e6;
	int bit_fail;
	int min_bit_fail;
	int min_epoch_counter = 0;

	/**
	 * Each data point will create two position sets, 
	 * i.e. the total amount of training data sets is twice the size of the position set vector.
	 * This data set is split up into 70% training data and 30% evaluation data 
	**/
	int len_learn_data = (int) (2.0*0.7*(double)no_of_possets);
	int len_eval_data = 2*no_of_possets - len_learn_data;

	/* amount of maximum allowed bit fails is set to 1% of size of evaluation data set*/
	int no_of_bit_fails = (int)(3.0*(double)len_eval_data/100.0);

	
	
	/* start timer */
	gettimeofday(&start,NULL);
	

	printf("Calibrate with target distance %f\n",TARGET_DISTANCE);
	/* enter training loop */
	while(1)
	{
					
		/**
		 * Create data sets
		 * These have to be allocated in every loop iteration since the input/output patterns 
		 * change on every iteration because the network itself changes on every iteration.
		**/
		struct fann_train_data *data = createData(ann, positionSet, no_of_possets);
		struct fann_train_data *training_data; 
		struct fann_train_data *evaluation_data;

		/* shuffle the data set, i.e. training data is applied in random order */
		fann_shuffle_train_data(data);

		/* split data set into training (70%) and evaluation set (30%) */
		training_data = fann_subset_train_data(data,0,len_learn_data);
		evaluation_data = fann_subset_train_data(data,len_learn_data,len_eval_data);

		/* train network for one epoch with training data set */
		fann_train_epoch(ann,training_data);

		/* evaluate ann with evaluation set */
		mse = fann_test_data(ann,evaluation_data);
		bit_fail = fann_get_bit_fail(ann);

		/* store the minimum seen mse and its network */
		if(mse<min_mse)
		{
			min_mse = mse;
			min_bit_fail = bit_fail;
			min_ann = ann;
			/* since a new minimum mse has been computed, reset epoch counter */
			min_epoch_counter = 0;
		}
		else
		{
			/* if mse larger than min mse increase epoch counter */
			min_epoch_counter++;
		}



		/**
		 * stop learning when constraints as computed by the evaluation set are reached.
		 * -> Maximum mse and not more than 1% of outputs of output neurons are larger than threshold for the whole evaluation set
 		 * -> Output of networks produces no output larger than given threshold (bit_fail==0)
		 * -> The minimum MSE has not decreased in the last n epochs
		**/
		if((mse<max_mse && bit_fail<=no_of_bit_fails) || bit_fail==0)
		{
			cout << "MSE or bit fail requirement reached" << endl;
			gettimeofday(&end,0);
			t_diff = end.tv_sec - start.tv_sec;
			fstream net_result;
			net_result.open("network_results.txt",ios::out|ios::app);
			net_result << no_of_possets << " " << " " << min_mse << " " << i << " " << bit_fail << " " << t_diff << "\n";
			net_result.close();			
			break;
		}

		/**
		 * If epoch counter reached maximum, i.e. mse is not decreasing anymore,
		 * break training as long as a minimum accuracy in terms of mse and bit fail has been reached
		**/
		if(min_epoch_counter>=400 && min_mse<1e-5 && min_bit_fail<no_of_bit_fails)
		{
			cout << "Maximum amount of epochs reached where the mse has not decreased" << endl;
			//take the best network so far
			cout << "Epoch counter reached maximum" << endl;
			ann = min_ann;
			mse = min_mse;
			gettimeofday(&end,0);
			t_diff = end.tv_sec - start.tv_sec;
			fstream net_result;
			net_result.open("network_results.txt",ios::out|ios::app);
			net_result << no_of_possets << " " << " " << min_mse << " " << i << " " << bit_fail << " " << t_diff << "\n";
			net_result.close();
			break;

		}

		/* debug output on every 100th epoch */
		if(i%100==0) 
		{
			gettimeofday(&tmp,NULL);
			t_diff = tmp.tv_sec - start.tv_sec;
			printf("N = %06d, MSE = %04.12f @ epoch %5d (%06d), %lu s\n",no_of_possets,mse,i,bit_fail,t_diff);
		}
	
		/* apply simulated annealing every 200th epoch to avoid getting stuck in local minima */
		if(i%200==0)
		{
			simulated_annealing(ann,i);
		}

		i++;

		/* Free memory to avoid memory leakage */
		fann_destroy_train(data);
		fann_destroy_train(training_data);
		fann_destroy_train(evaluation_data);

	}
	

	/* get time to compute total runtime */
	gettimeofday(&end,NULL);
	t_diff = end.tv_sec - start.tv_sec;
	/* Debug output */
	std::cout << "Training for N = " << no_of_possets << 
				" finished at epoch " << i << ": Learning runtime: " << t_diff << " s" <<
				" MSE = " << mse << "bit fail: " << bit_fail << std::endl;
	
	/* store network to file */
	char ann_name[32];
	sprintf(ann_name,"cam_tracker.net");
	fann_save(ann, ann_name);
	printf("Trained network saved to file.\n");


	create_accuracy_map(ann,positionSet,no_of_possets);

	//clean up
	fann_destroy(ann);

	exit(0);
	
}

void create_accuracy_map(struct fann *ann, vector<PositionSet *> positionSet, int no_of_possets)
{

	FILE *acc_fd = fopen("accuracy.txt","w");
	if(!acc_fd)
	{
		printf("Unable to open file\n");
		return;
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

}

struct fann_train_data *createData(struct fann *ann, vector<PositionSet*> positionSet, int no_of_possets)
{	

	struct fann_train_data *data;

	unsigned int num_input = 4;
	unsigned int num_output = 3;
	unsigned int num_data = 2*no_of_possets;

	/* create new empty data struct for num_input input neurons and num_output output neurons */
	data = fann_create_train(num_data, num_input, num_output);
	if(!data) exit(-1);

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

	/* the computed target outputs */
	fann_type *marker1_target_out;
	fann_type *marker2_target_out;
	marker1_target_out = (fann_type *) calloc(3,sizeof(fann_type));
	marker2_target_out = (fann_type *) calloc(3,sizeof(fann_type));


	/* actual Euclidean distance between markers */
	double d;
	/* target distance between markers */
	const double dT = TARGET_DISTANCE;


	double rx,ry,rz;	
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
	
		/* direction vector pointing from marker 1 to marker 2 normalized to length 1 */
		if(d>1e-9)
		{
			rx = (x_m2 - x_m1) / d;
			ry = (y_m2 - y_m1) / d;
			rz = (z_m2 - z_m1) / d;

		}
		else
		{
			/* network produces same output for both inputs */
			fprintf(stderr,"Training failed, both inputs produce same output");
			return 0;
 		}

		/* compute actual target output with respect to target distance */
		marker1_target_out[0] = x_m1 + 0.5*(d-dT)*rx;
		marker1_target_out[1] = y_m1 + 0.5*(d-dT)*ry;
		marker1_target_out[2] = z_m1 + 0.5*(d-dT)*rz;

		marker2_target_out[0] = x_m2 - 0.5*(d-dT)*rx;
		marker2_target_out[1] = y_m2 - 0.5*(d-dT)*ry;
		marker2_target_out[2] = z_m2 - 0.5*(d-dT)*rz;

		/* store computed input/output patterns in train_data struct */
		for(unsigned int m=0; m<num_input; ++m)
		{
			data->input[2*i][m] = input_marker1[m];
			data->input[2*i+1][m] = input_marker2[m];
		}
		for(unsigned int m=0; m<num_output; ++m)
		{
			data->output[2*i][m] = marker1_target_out[m];
			data->output[2*i+1][m] = marker2_target_out[m];
		}


	}

	return data;
	
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
		temp->marker[0][0].u = 2.0*(temp->marker[0][0].u/(double)FRAME_WIDTH)-1.0;
		temp->marker[0][0].v = 2.0*(temp->marker[0][0].v/(double)FRAME_HEIGHT)-1.0;
		temp->marker[1][0].u = 2.0*(temp->marker[1][0].u/(double)FRAME_WIDTH)-1.0;
		temp->marker[1][0].v = 2.0*(temp->marker[1][0].v/(double)FRAME_HEIGHT)-1.0;

		/* cam1 */
		cam1_data_f >> temp->marker[0][1].u >> temp->marker[0][1].v
				>> temp->marker[1][1].u >> temp->marker[1][1].v;
		temp->marker[0][1].u = 2.0*(temp->marker[0][1].u/(double)FRAME_WIDTH)-1.0;
		temp->marker[0][1].v = 2.0*(temp->marker[0][1].v/(double)FRAME_HEIGHT)-1.0;
		temp->marker[1][1].u = 2.0*(temp->marker[1][1].u/(double)FRAME_WIDTH)-1.0;
		temp->marker[1][1].v = 2.0*(temp->marker[1][1].v/(double)FRAME_HEIGHT)-1.0;		

		/* push back data into vector */
		positionSet.push_back(temp);

		if(cam1_data_f.eof()) break;
	}

	//remove last element of vector since  the previous code reads the last line of the file twice
	positionSet.pop_back();

	return positionSet;
}

/**
 * Change all weights of the network slightly after certain amount of epochs to avoid to get stuck into local minima
**/
void simulated_annealing(struct fann *ann, int epoch)
{
	if(epoch==0) return;

	/* scaling factor to decrease amount of simulated annealing for increasing epochs
		i.e. influence of simulated annealing decreases over time */
	double alpha = exp(-(double)epoch/500.0-0.2);

	/* do not apply weight change, when amount is too small */
	if(alpha<1e-9) return;

	
	fann_type *last_weight;
	fann_type *weights = ann->weights;

	last_weight = weights + ann->total_connections;

	double u_weight;
	double rand_add;
	double r;

	/* for all weights of the networks */
	for(;weights!=last_weight;weights++)
	{
		/* store the current weight */
		u_weight = *weights;
		/* compute random value between -1 and 1 */
		r = 2.0*((double)rand() / (double)RAND_MAX - 0.5);
		/* weight random value with scaling factor such that amount decreases over time */
		rand_add = alpha * r * u_weight/10;
		/* change the weight */
		*weights += rand_add;
	}
}

