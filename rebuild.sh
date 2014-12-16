#!/bin/bash
#This should be launched from /home/travis/build/<username>/
set -ev

export PATH=$(pwd)/microchip/xc16/v1.23/bin:$PATH
export PATH=$(pwd)/microchip/mplabx/mplab_ide/bin:$PATH

# Open project in local MPLABX, forcing regeneration of nbproject/Makefile-default.mk
# Run as background process, then wait for file creation below.
export DISPLAY=":10"
#mplab_ide --jdkhome=./microchip/mplabx/sys/java/jre1.7.0_67/ --nosplash --open roach/firmware &
mplab_ide --open roach/firmware &

# wait until file exists
while [ ! -f roach/firmware/nbproject/Makefile-default.mk ]
do
  sleep 2
  echo "Waiting on nbproject/Makefile-default.mk"
done

# wait for completion
# todo: this may not be necessary
sleep 5

# initiate build
cd roach/firmware
make
