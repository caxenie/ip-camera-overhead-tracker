CXX = g++
CFLAGS = -O0 -Wall -g `pkg-config --cflags opencv` -I/usr/local/include
LDFLAGS = `pkg-config --libs opencv` -lm -lpthread -ldc1394 -L/usr/local/lib -lfann -lconfig

all: neural_calib rec_calib_data test_cam_pair create_acc_map train_fundamental_matrix

neural_calib: ../src/global.h src/neural_calib.cpp 
	$(CXX) -o src/neural_calib.o -c src/neural_calib.cpp $(CFLAGS)
	$(CXX) -o neural_calib src/neural_calib.o $(LDFLAGS)

rec_calib_data: ../src/global.h ../src/ColorDiffTracker.cpp ../src/ColorDiffTracker.h src/rec_calib_data.cpp ../src/utils.cpp
	$(CXX) -o src/rec_calib_data.o -c src/rec_calib_data.cpp $(CFLAGS)
	$(CXX) -o src/ColorDiffTracker.o -c ../src/ColorDiffTracker.cpp $(CFLAGS)
	$(CXX) -o src/utils.o -c ../src/utils.cpp $(CFLAGS)
	$(CXX) -o rec_calib_data src/rec_calib_data.o src/ColorDiffTracker.o src/utils.o $(LDFLAGS)

train_fundamental_matrix: src/train_fundamental_matrix.cpp
	$(CXX) -o src/train_fundamental_matrix.o -c src/train_fundamental_matrix.cpp $(CFLAGS)
	$(CXX) -o train_fundamental_matrix src/train_fundamental_matrix.o $(LDFLAGS)
	
test_cam_pair: ../src/global.h ../src/ColorDiffTracker.cpp ../src/ColorDiffTracker.h src/test_cam_pair.cpp ../src/streamer.h ../src/streamer.cpp ../src/utils.cpp
	$(CXX) -o src/test_cam_pair.o -c src/test_cam_pair.cpp $(CFLAGS)
	$(CXX) -o src/ColorDiffTracker.o -c ../src/ColorDiffTracker.cpp $(CFLAGS)
	$(CXX) -o src/streamer.o -c ../src/streamer.cpp $(CFLAGS)
	$(CXX) -o src/utils.o -c ../src/utils.cpp $(CFLAGS)
	$(CXX) -o test_cam_pair src/test_cam_pair.o src/ColorDiffTracker.o src/streamer.o src/utils.o $(LDFLAGS)

create_acc_map: ../src/global.h src/create_acc_map.cpp
	$(CXX) -o src/create_acc_map.o -c src/create_acc_map.cpp $(CFLAGS)
	$(CXX) -o create_acc_map src/create_acc_map.o $(LDFLAGS)



clean:
	@rm neural_calib
	@rm rec_calib_data
	@rm test_cam_pair
	@rm create_acc_map
	@rm train_fundamental_matrix
	@rm src/*.o
	@rm -f *~
	@rm -f src/*~
