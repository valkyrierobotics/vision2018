#! /bin/bash

# Run the mjpg_streamer for port at localhost:8080 using the mjpeg file in ./images/
if [ `ps ax | grep -v grep | grep mjpg_streamer | wc -l` -gt 0 ]; then
    echo -e "Killing existing mjpg_streamer instances"
    killall mjpg_streamer
fi
mjpg_streamer -i "input_file.so -f ./images/mjpgs/" -o "output_http.so -w /usr/local/www"
