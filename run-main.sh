#! /bin/bash
# Compile and run main.cpp as a background process

# echo -e "\nStarted background process\n"

if [ `ps ax | grep -v grep | grep vision | wc -l` -gt 0 ]; then
    echo -e "Killing existing vision instances\n"
    killall vision
fi

if g++ main.cpp build/lazer-vision.so -o vision -L/usr/local/cuda-6.5/lib/ `pkg-config opencv --cflags --libs` --std=c++11; then
	./vision > images/fps.png
fi

exit
