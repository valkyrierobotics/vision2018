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

LIB = -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_objdetect -lopencv_calib3d
# LIB = `pkg-config opencv --cflags --libs`
CPPSRCS = $(wildcard src/*.cpp) $(wildcard src/*/*.cpp)
SRCS = $(CPPSRCS) $(CSRCS)

CFLAGS_DEBUG = -g -D_LAZER_DEBUG
OBJS_DEBUG = $(patsubst src/%.cpp,obj/%-dbg.o, $(SRCS))
CFLAGS_RELEASE = -O3 -Wno-unused-value
OBJS_RELEASE = $(patsubst src/%.cpp,obj/%.o,$(SRCS))

CFLAGS = -Iinclude -static-libgcc -Wall -fno-use-linker-plugin -fno-exceptions -shared -fPIC
CXXFLAGS = -static-libstdc++ -std=c++11 -Wall -Wextra -fexceptions

deploy: .build_dir build/lazer-vision.so

debug: .build_dir build/lazer-vision-dbg.so

build/lazer-vision.so: $(OBJS_RELEASE)
	$(LD) $(CFLAGS_RELEASE) $(CFLAGS) $(CXXFLAGS) -o $@ $^ $(LIB)

build/lazer-vision-dbg.so: $(OBJS_DEBUG)
	$(LD) $(CFLAGS_DEBUG) $(CFLAGS) $(CXXFLAGS) -o $@ $^ $(LIB)

clean:
	$(RM) -rf build/ obj/ .build_dir

main:
	$(CXX) main.cpp build/lazer-vision.so -o build/main $(CFLAGS_RELEASE) $(CXXFLAGS) $(LIB);
	build/main ${ARGS}

send_start_signal:
	$(CXX) utilities/start_ping.cpp build/lazer-vision.so -o build/start_ping $(CFLAGS_RELEASE) $(CXXFLAGS) $(LIB);
	build/start_ping ${ARGS}

send_stop_signal:
	$(CXX) utilities/stop_ping.cpp build/lazer-vision.so -o build/stop_ping $(CFLAGS_RELEASE) $(CXXFLAGS) $(LIB);
	build/stop_ping ${ARGS}

mjpg_streamer_instance:
	# Run the mjpg_streamer for port at localhost:8080 using the mjpeg file in ./images/
	if [ `ps ax | grep -v grep | grep mjpg_streamer | wc -l` -gt 0 ]; then \
		echo "Killing existing mjpg_streamer instances"; \
		killall mjpg_streamer; \
	fi; \
	mjpg_streamer -i "input_file.so -f ./images/mjpgs/" -o "output_http.so -w /usr/local/www" 

gnuplot_vision:
	# GRAPH_DIR is the name of the directory inside images/gnuplot/
	# ARGS is for gnuplot_auto_plotter.sh
	utilities/gnuplot_auto_plotter.sh -f logs/processed_data.log -d ${GRAPH_DIR} ${ARGS}

gnuplot_fps:
	# GRAPH_DIR is the name of the directory inside images/gnuplot/
	# ARGS is for gnuplot_auto_plotter.sh
	utilities/gnuplot_auto_plotter.sh -f logs/fps.log -d ${GRAPH_DIR} ${ARGS}

### build objects ###
obj: $(OBJ_RELEASE)

obj-debug: $(OBJ_DEBUG)

obj/%-dbg.o: src/%.cpp
	$(CXX) $(CFLAGS_DEBUG) $(CFLAGS) $(CXXFLAGS) -c -o $@ $<

obj/%.o: src/%.cpp
	$(CXX) $(CFLAGS_RELEASE) $(CFLAGS) $(CXXFLAGS) -c -o $@ $<

### create file tree ###
.build_dir:
	$(MKDIR) obj obj/input obj/filters obj/logging obj/utils \
		build
	$(TOUCH) .build_dir
