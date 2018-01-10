# Written by Lee Mracek and Min Hoo Lee
CC = gcc
CXX = g++
LD = g++
RM = rm
CP = cp
MV = mv
MKDIR = mkdir
TOUCH = touch
SED = sed
MAKEDEPEND = makedepend
VALGRIND = valgrind
SYMLINK = ln -sf
# Default arguments
PLOT_FPS_ARGS = -c 1 -n 60 -m 30
PLOT_VISION_ARGS = -c 4 -n 30 -m 60 -d 30_deg

LIB = `pkg-config --cflags --libs opencv protobuf` -lstdc++
CPPSRCS = $(wildcard y2018/*.cpp) $(wildcard y2018/*/*.cpp) $(wildcard y2018/*/*.proto) $(wildcard y2018/*/*.pb.cc*)
SRCS = $(CPPSRCS) $(CSRCS)

CFLAGS_DEBUG = -g -D_LAZER_DEBUG
OBJS_DEBUG = $(patsubst y2018/%.cpp,obj/%-dbg.o, $(SRCS))
CFLAGS_RELEASE = -O3 -Wno-unused-value
OBJS_RELEASE = $(patsubst y2018/%.cpp,obj/%.o,$(SRCS))

CFLAGS = -Iy2018 -static-libgcc -Wall -fno-use-linker-plugin -fno-exceptions -shared -fPIC
CXXFLAGS = -static-libstdc++ -std=c++11 -Wall -Wextra -fexceptions

deploy: .build_dir build/lazer-vision.so

debug: .build_dir build/lazer-vision-dbg.so

build/lazer-vision.so: $(OBJS_RELEASE)
	$(LD) $(CFLAGS_RELEASE) $(CFLAGS) $(CXXFLAGS) -o $@ $^ $(LIB)

build/lazer-vision-dbg.so: $(OBJS_DEBUG)
	$(LD) $(CFLAGS_DEBUG) $(CFLAGS) $(CXXFLAGS) -o $@ $^ $(LIB)

clean:
	$(RM) -rf build/ obj/ .build_dir

# Change camera's exposure and run vision
main: deploy
	$(SYMLINK) save_runs/best.yml logs/config.yml
	$(CXX) main.cpp y2018/vision_data.pb.cc aos/udp.cc aos/aos_strerror.cc build/lazer-vision.so -o build/main $(CFLAGS_RELEASE) $(CXXFLAGS) $(LIB);
	build/main ${MAIN_ARGS}

# Test protouf (necessary to run main without connecting to roboRIO)
protobuf_test: deploy
	$(CXX) protobuf_test.cpp y2018/vision_data.pb.cc aos/udp.cc aos/aos_strerror.cc build/lazer-vision.so -o build/protobuf_test $(CFLAGS_RELEASE) $(CXXFLAGS) $(LIB);
	build/protobuf_test

drive_camera:
	$(CXX) drive_camera.cpp build/lazer-vision.so -o build/drive_camera $(CFLAGS_RELEASE) $(CXXFLAGS) $(LIB);
	build/drive_camera

send_start_signal: deploy
	$(CXX) utilities/start_ping.cpp build/lazer-vision.so -o build/start_ping $(CFLAGS_RELEASE) $(CXXFLAGS) $(LIB);
	build/start_ping ${START_ARGS}

send_stop_signal: deploy
	$(CXX) utilities/stop_ping.cpp build/lazer-vision.so -o build/stop_ping $(CFLAGS_RELEASE) $(CXXFLAGS) $(LIB);
	build/stop_ping ${STOP_ARGS}

mjpg_streamer_instance:
	# Run the mjpg_streamer for port at localhost:8080 using the mjpeg file in ./images/
	if [ `ps ax | grep -v grep | grep mjpg_streamer | wc -l` -gt 0 ]; then \
		echo "Killing existing mjpg_streamer instances"; \
		killall mjpg_streamer; \
	fi; \
	sudo mjpg_streamer -i "/usr/local/lib/input_file.so -f ./images/mjpgs/" -o "/usr/local/lib/output_http.so -w /usr/local/www -p 443" 

gnuplot_vision:
	# ARGS is for gnuplot_auto_plotter.sh
	utilities/gnuplot_auto_plotter.sh -f logs/processed_data.log ${PLOT_VISION_ARGS}

gnuplot_fps:
	# ARGS is for gnuplot_auto_plotter.sh
	utilities/gnuplot_auto_plotter.sh -f logs/fps.log ${PLOT_FPS_ARGS}

camera_calib:
	$(CXX) utilities/camera_calibration.cpp -o build/camera_calib $(CFLAGS_RELEASE) $(CXXFLAGS) $(LIB);
	build/camera_calib logs/in_VID5.xml

### build objects ###
obj: $(OBJ_RELEASE)

obj-debug: $(OBJ_DEBUG)

obj/%-dbg.o: y2018/%.cpp
	$(CXX) $(CFLAGS_DEBUG) $(CFLAGS) $(CXXFLAGS) -c -o $@ $<

obj/%.o: y2018/%.cpp
	$(CXX) $(CFLAGS_RELEASE) $(CFLAGS) $(CXXFLAGS) -c -o $@ $<

### create file tree ###
.build_dir:
	$(MKDIR) obj obj/input obj/filters obj/logging obj/utils obj/common \
		build
	$(TOUCH) .build_dir
