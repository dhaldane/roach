RoACH
==========
This repository contains firmware and python control code for the VelociRoACH robots, or other derived robots equipped with an ImageProc 2.5 board.

The 'master' branch is intended to provide a working configuration for the VelociRoACH robot.  
This branch should be expected to always build, work, and function.

Repositories imageproc-lib and imageproc-settings are required to build.  
Relative paths are set so that you need to clone each repository into the same root directory.

Build status: [![Build Status](https://travis-ci.org/biomimetics/roach.svg?branch=master)](https://travis-ci.org/biomimetics/roach)  
Built against biomimetics/roach 'master branch:
https://github.com/biomimetics/roach


Files:
---------
 firmware/   -  contains the C code firmware for the robot, and the MPLAB IDE project files.
 lib/		 -  C code library of modules for the octoroach firmware
 python/ 	 -	python code for controlling the robot from a PC, and examples.
 doc/		 -  documentation on firmware and python code.

Instructions
-------------
To be updated