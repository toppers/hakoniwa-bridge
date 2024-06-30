#!/bin/bash

if [ ! -d cmake-build ]
then
    mkdir cmake-build
fi

cd cmake-build

if [ "$OS" = "Linux" -o "$OS" = "Darwin"  ]
then
    cmake ..
    make
else
    cmake -DCMAKE_BUILD_TYPE=Release ..
    cmake --build . --target ALL_BUILD --config Release
fi