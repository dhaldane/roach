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

def main():    
    xb = setupSerial(shared.BS_COMPORT, shared.BS_BAUDRATE)
    
    R1 = Velociroach('\x21\x02', xb)
    
    shared.ROBOTS = [R1] #This is neccesary so callbackfunc can reference robots
    shared.xb = xb           #This is neccesary so callbackfunc can halt before exit

    # Send robot a WHO_AM_I command, verify communications
    R1.queryRobot()
    #Motor gains format:
    #  [ Kp , Ki , Kd , Kaw , Kff     ,  Kp , Ki , Kd , Kaw , Kff ]
    #    ----------LEFT----------        ---------_RIGHT----------
    motorgains = [1800,200,100,0,0, 1800,200,100,0,0]

    params = hallParams(motorgains, duration=500, rightFreq=0, leftFreq=0, phase=0, telemetry=True, repeat=False)
    setMotorGains(motorgains)

    leadIn = 10
    leadOut = 10
    strideFreq = 5
    phase = 0x8000      # Alternating tripod
    useFlag = 0
    deltas = [.25, 0.25, 0.25, 0.25, 0.25, 0.25]

    manParams = manueverParams(leadIn, leadOut, strideFreq, phase, useFlag, deltas)

    while True:

        if not(params.repeat):
            settingsMenu(params, manParams)   

        if params.telemetry:
            R1.writeFileHeader()
            
            if manParams.useFlag == True:
                duration = 1.0/manParams.strideFreq * (manParams.leadIn + 1 + manParams.leadOut)
                numSamples = int(ceil(1000 * duration))
            else:
                numSamples = int(ceil(1000 * (params.duration + shared.leadinTime + shared.leadoutTime) / 1000.0))
            eraseFlashMem(numSamples)

        # Trigger telemetry save, which starts as soon as it is received
        if params.telemetry:
        # Pause and wait to start run, including leadin time
            raw_input("Press enter to start run ...") 
            startTelemetrySave(numSamples)
        #Start robot
        if manParams.useFlag == True:
            runManeuver(params, manParams)
        else:
            R1.setPhase(params.phase)
            R1.startTimedRun(params.duration)
            
        if params.telemetry and query_yes_no("Save Data?"):
            flashReadback(numSamples, params, manParams)

        repeatMenu(params)

    print "Done"

#Provide a try-except over the whole main function
# for clean exit. The Xbee module should have better
# provisions for handling a clean exit, but it doesn't.
if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print "\nRecieved Ctrl+C, exiting."
        shared.xb.halt()
        shared.ser.close()
    except Exception as args:
        print "\nGeneral exception:",args
        print "Attemping to exit cleanly..."
        shared.xb.halt()
        shared.ser.close()
        sys.exit()
    except serial.serialutil.SerialException:
        shared.xb.halt()
        shared.ser.close()
