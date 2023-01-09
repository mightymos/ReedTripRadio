#!/bin/bash

sudo apt-get install -y git sdcc cmake


# flashing tool
#git clone https://github.com/area-8051/stcgal-patched.git

# hardware abstraction layer for stc
git clone https://github.com/area-8051/uni-STC.git
cd uni-STC/demos/

# this repository
git clone https://github.com/mightymos/ReedTripRadio.git
cd ReedTripRadio/
echo "Building firmware..."
make
