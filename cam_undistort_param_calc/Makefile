all:
	gcc -Wall -export-dynamic -o cam_undistort_param_calc main.c -std=gnu99 -g `pkg-config --cflags --libs gtk+-2.0 opencv`

clean:
	rm cam_undistort_param_calc
