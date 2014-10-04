#!/usr/bin/env python
"""
authors: apullin

This script will run an experiment with one or several Velociroach robots.

The main function will send all the setup parameters to the robots, execute defined manoeuvres, and record telemetry.

"""
from lib import command
import time,sys,os
import serial
import shared

from velociroach import *

###### Operation Flags ######
RESET_R1 = True  
SAVE_DATA_R1 = False 

####### Wait at exit? #######
EXIT_WAIT   = False


def main():    
    xb = setupSerial(shared.BS_COMPORT, shared.BS_BAUDRATE)
    
    R1 = Velociroach('\x21\x02', xb)
    
    shared.ROBOTS = [R1] #This is necessary so callbackfunc can reference robots
    shared.xb = xb           #This is necessary so callbackfunc can halt before exit

    # Send resets
    if RESET_R1:
        R1.reset()
        time.sleep(0.35)
    # Repeat this for other robots
    # TODO: move reset / telem flags inside robot class? (pullin)
    
    # Send robot a WHO_AM_I command, verify communications
    R1.queryRobot()
    
    #Verify all robots can be queried
    verifyAllQueried()  # exits on failure
    
    # Motor gains format:
    #  [ Kp , Ki , Kd , Kaw , Kff     ,  Kp , Ki , Kd , Kaw , Kff ]
    #    ----------LEFT----------        ---------_RIGHT----------
    motorgains = [1800,200,100,0,0, 1800,200,100,0,0]

    simpleAltTripod = GaitConfig(motorgains, rightFreq=5, leftFreq=5) # Parameters can be passed into object upon construction, as done here.
    simpleAltTripod.phase = PHASE_180_DEG                             # Or set individually, as here
    simpleAltTripod.deltasLeft = [0.25, 0.25, 0.25]
    simpleAltTripod.deltasRight = [0.25, 0.25, 0.25]
    #simpleAltTripod.deltasTime  = [0.25, 0.25, 0.25] # Not current supported by firmware; time deltas are always exactly [0.25, 0.25, 0.25, 0.25]
    
    # Configure intra-stride control
    R1.setGait(simpleAltTripod)

    # example , 0.1s lead in + 2s run + 0.1s lead out
    EXPERIMENT_RUN_TIME     = 2000 #ms
    EXPERIMENT_LEADIN_TIME  = 100  #ms
    EXPERIMENT_LEADOUT_TIME = 100  #ms
    
    # Some preparation is needed to cleanly save telemetry data
    if SAVE_DATA_R1:
        #This needs to be done to prepare the .telemtryData variables in each robot object
        R1.setupTelemetryData(experiment_runtime)
        R1.eraseFlashMem()
        
    # Pause and wait to start run, including lead-in time
    print ""
    print "  ***************************"
    print "  *******    READY    *******"
    print "  ***************************"
    raw_input("  Press ENTER to start run ...")
    print ""

    # Initiate telemetry recording; the robot will begin recording immediately when cmd is received.
    if SAVE_DATA_R1:
        R1.startTelemetrySave()
    
    # Sleep for a lead-in time before any motion commands
    time.sleep(EXPERIMENT_LEADIN_TIME)
    
    ######## Motion is initiated here! ########
    R1.startTimedRun( EXPERIMENT_RUN_TIME )
    time.sleep(EXPERIMENT_RUN_TIME)
    ######## End of motion commands   ########
    
    # Sleep for a lead-out time after any motion
    time.sleep(EXPERIMENT_LEADOUT_TIME)
    
    if SAVE_DATA1:
        time.sleep(0.25) #and a little extra, for system settle
        raw_input("Press Enter to start telemetry read-back ...")
        R1.downloadTelemetry()
    
    if EXIT_WAIT:  #Pause for a Ctrl + C , if desired
        while True:
            try:
                time.sleep(0.1)
            except KeyboardInterrupt:
                break

    print "Done"
    xb_safe_exit(xb)
    
#Provide a try-except over the whole main function
# for clean exit. The Xbee module should have better
# provisions for handling a clean exit, but it doesn't.
#TODO: provide a more informative exit here; stack trace, exception type, etc
if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print "\nRecieved Ctrl+C, exiting."
        shared.xb.halt()
        shared.ser.close()
    except Exception as args:
        print "\nGeneral exception:",args
        print "Attempting to exit cleanly..."
        shared.xb.halt()
        shared.ser.close()
        sys.exit()
    except serial.serialutil.SerialException:
        shared.xb.halt()
        shared.ser.close()
