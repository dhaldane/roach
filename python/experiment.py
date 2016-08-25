#!/usr/bin/env python
"""
authors: stanbaek, apullin

"""
from lib import command
import time,sys,os,traceback
import serial

# Path to imageproc-settings repo must be added
sys.path.append(os.path.dirname("../../imageproc-settings/"))
sys.path.append(os.path.dirname("../imageproc-settings/"))      # Some projects have a single-directory structure
import shared

from hall_helpers import *

def main():    
    setupSerial()

    # Send robot a WHO_AM_I command, verify communications
    queryRobot()
    #Motor gains format:
    #  [ Kp , Ki , Kd , Kaw , Kff     ,  Kp , Ki , Kd , Kaw , Kff ]
    #    ----------LEFT----------        ---------_RIGHT----------
    motorgains = [300,0,25,0,0, 100,0,0,0,0]
    # motorgains = [0,0,0,0,0, 100,0,0,0,0]
    duration = 2000
    rightFreq = 0
    leftFreq = 0
    phase = 0
    telemetry = True
    repeat = False

    params = hallParams(motorgains, duration, rightFreq, leftFreq, phase, telemetry, repeat)
    setMotorGains(motorgains)

    leadIn = 10
    leadOut = 10
    strideFreq = 5
    phase = 0x8000      # Alternating tripod
    useFlag = 0
    deltas = [.25, 0.25, 0.25, 0.125, 0.125, 0.5]

    manParams = manueverParams(leadIn, leadOut, strideFreq, phase, useFlag, deltas)

    while True:

        if not(params.repeat):
            settingsMenu(params, manParams)   

        if params.telemetry:
            # Construct filename
            # path     = '/home/duncan/Data/'
            path     = 'Data/'
            name     = 'trial'
            datetime = time.localtime()
            dt_str   = time.strftime('%Y.%m.%d_%H.%M.%S', datetime)
            root     = path + dt_str + '_' + name
            shared.dataFileName = root + '_imudata.txt'
            print "Data file:  ", shared.dataFileName
            print os.curdir

            numSamples = int(ceil(1000 * (params.duration + shared.leadinTime + shared.leadoutTime) / 1000.0))
            eraseFlashMem(numSamples)
            raw_input("Press enter to start run ...") 
            startTelemetrySave(numSamples)

        #Start robot
        # xb_send(0, command.START_EXPERIMENT, "0")
        # time.sleep(params.duration / 1000.0)

        # Just jump
        pos = []
        pos.append(13 * 65536)
        pos.append(1)
        pwmDes = [0,0]
        # xb_send(0, command.SET_THRUST_OPEN_LOOP, pack('2h', *pwmDes))
        xb_send(0, command.PID_START_MOTORS, "0")
        # xb_send(0, command.SET_MOTOR_POS, pack('lh', *pos))
        print pos
        time.sleep(params.duration/1000.0)
        xb_send(0, command.PID_STOP_MOTORS, "0")

        if params.telemetry and query_yes_no("Save Data?"):
            flashReadback(numSamples, params, manParams)

        repeatMenu(params)

    print "Done"
    
    
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
        print "\n    ******    TRACEBACK    ******    "
        traceback.print_stack()
        print "    *****************************    \n"
        print "Attempting to exit cleanly..."
        shared.xb.halt()
        shared.ser.close()
        sys.exit()
    except serial.serialutil.SerialException:
        shared.xb.halt()
        shared.ser.close()
