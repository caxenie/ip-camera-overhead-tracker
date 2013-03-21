# Overhead ip cam tracker - calibrator makefile
all:
	g++ -o calibration_helper main.cpp -g -Wall `pkg-config --cflags --libs opencv`

clean:
	rm calibration_helper
	rm calibration-points.txt

run:
	./calibration_helper http://10.162.242.18/mjpg/video.mjpg
