#! /bin/bash
if g++ startPing.cpp build/lazer-vision.so -o ./build/startPing -L/usr/local/cuda-6.5/lib/ `pkg-config opencv --cflags --libs` --std=c++11; then
    ./build/startPing localhost 5810
fi
