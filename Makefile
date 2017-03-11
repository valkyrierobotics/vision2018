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
# Default arguments
PLOT_FPS_ARGS = -c 1 -n 60 -m 30
PLOT_VISION_ARGS = -c 4 -n 30 -m 60 -d 30_deg

LIB = -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_objdetect -lopencv_calib3d -lopencv_features2d 
# LIB = `pkg-config opencv --cflags --libs`
CPPSRCS = $(wildcard y2017/*.cpp) $(wildcard y2017/*/*.cpp)
SRCS = $(CPPSRCS) $(CSRCS)

CFLAGS_DEBUG = -g -D_LAZER_DEBUG
OBJS_DEBUG = $(patsubst y2017/%.cpp,obj/%-dbg.o, $(SRCS))
CFLAGS_RELEASE = -O3 -Wno-unused-value
OBJS_RELEASE = $(patsubst y2017/%.cpp,obj/%.o,$(SRCS))

CFLAGS = -Iy2017 -static-libgcc -Wall -fno-use-linker-plugin -fno-exceptions -shared -fPIC
CXXFLAGS = -static-libstdc++ -std=c++11 -Wall -Wextra -fexceptions

deploy: .build_dir build/lazer-vision.so

debug: .build_dir build/lazer-vision-dbg.so

build/lazer-vision.so: $(OBJS_RELEASE)
	$(LD) $(CFLAGS_RELEASE) $(CFLAGS) $(CXXFLAGS) -o $@ $^ $(LIB)

build/lazer-vision-dbg.so: $(OBJS_DEBUG)
	$(LD) $(CFLAGS_DEBUG) $(CFLAGS) $(CXXFLAGS) -o $@ $^ $(LIB)

clean:
	$(RM) -rf build/ obj/ .build_dir

main: deploy
	$(CXX) main.cpp build/lazer-vision.so -o build/main $(CFLAGS_RELEASE) $(CXXFLAGS) $(LIB);
	build/main ${MAIN_ARGS}

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
	mjpg_streamer -i "/usr/local/lib/input_uvc.so -d /dev/video0 -r 640x480 -f 10 -q 80" -o "/usr/local/lib/output_http.so -w /usr/local/www" 
	# mjpg_streamer -i "input_file.so -f ./images/mjpgs/" -o "output_http.so -w /usr/local/www" 

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

obj/%-dbg.o: y2017/%.cpp
	$(CXX) $(CFLAGS_DEBUG) $(CFLAGS) $(CXXFLAGS) -c -o $@ $<

obj/%.o: y2017/%.cpp
	$(CXX) $(CFLAGS_RELEASE) $(CFLAGS) $(CXXFLAGS) -c -o $@ $<

### create file tree ###
.build_dir:
	$(MKDIR) obj obj/input obj/filters obj/logging obj/utils \
		build
	$(TOUCH) .build_dir
