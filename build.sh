#!/bin/bash

# Sanity check: Check that we're in the root of the Git repo
# Sanity check: Make sure we have the essentials for building

# Getting the Pico-SDK
git clone https://github.com/raspberrypi/pico-sdk
cd pico-sdk
git checkout 4fe995d0ec984833a7ea9c33bac5c67a53c04178
git submodule update --init
cd ..
export PICO_SDK_PATH=`pwd`/pico-sdk/

# Building tamarin-firmwaer
mkdir build
cd build
cmake ..
make

cd ..
