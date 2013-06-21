#!/usr/bin/env python
"""
authors: stanbaek, apullin

"""
from lib import command
import time,sys
import serial
import shared

from hall_helpers import *

def main():    
    setupSerial()

    # Send robot a WHO_AM_I command, verify communications
    queryRobot()
    #Motor gains format:
    #  [ Kp , Ki , Kd , Kaw , Kff     ,  Kp , Ki , Kd , Kaw , Kff ]
    #    ----------LEFT----------        ---------_RIGHT----------
    motorgains = [1800,0,400,0,0, 1800,0,400,0,0]
    duration = 2000
    vel = 0
    turn_rate = 0
    telemetry = True
    repeat = False

    print turn_rate
    params = hallParams(motorgains, duration, vel, turn_rate, telemetry, repeat)
    setMotorGains(motorgains)

    while True:

        if not(params.repeat):
            settingsMenu(params)   

        if params.telemetry:
            # Construct filename
            path     = 'Data/'
            name     = 'trial'
            datetime = time.localtime()
            dt_str   = time.strftime('%Y.%m.%d_%H.%M.%S', datetime)
            root     = path + dt_str + '_' + name
            shared.dataFileName = root + '_imudata.txt'
            print "Data file:  ", shared.dataFileName
            eraseFlashMem(numSamples)


        numSamples = int(ceil(300 * (params.duration + shared.leadinTime + shared.leadoutTime) / 1000.0))
        shared.imudata = [ [] ] * numSamples
        

        # Pause and wait to start run, including leadin time
        raw_input("Press enter to start run ...") 
        # Trigger telemetry save, which starts as soon as it is received
        if params.telemetry:
            startTelemetrySave(numSamples)
        #Start robot
        xb_send(0, command.START_TIMED_RUN, pack('h',0))

        time.sleep(params.duration / 1000.0)
        

        if params.telemetry and query_yes_no("Save Data?"):
            flashReadback(numSamples, params)

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
