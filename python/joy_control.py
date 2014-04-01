import sys
import numpy as np
from lib import command 
from struct import *
import time
from xbee import XBee
import serial
import pygame
from callbackFunc import xbee_received
import shared

DEST_ADDR = '\x21\x02'
imudata_file_name = 'imudata.txt'
statedata_file_name = 'statedata.txt'
dutycycle_file_name = 'dutycycle.txt'
motordata_file_name = 'motordata.txt'

imudata = []
statedata = []
dutycycles = []
motordata = []
gainsNotSet = True;

MAXPER = 1000


ser = serial.Serial(shared.BS_COMPORT, 230400,timeout=3, rtscts=1)
xb = XBee(ser, callback = xbee_received)

def xb_send(status, type, data):
    payload = chr(status) + chr(type) + ''.join(data)
    xb.tx(dest_addr = DEST_ADDR, data = payload)

def resetRobot():
    xb_send(0, command.SOFTWARE_RESET, pack('h',0))


def main():
    global MAXPER

    if ser.isOpen():
        print "Serial open."

    resetRobot()
    time.sleep(1)

    try:
        pygame.init()
        j = pygame.joystick.Joystick(0)
        j.init()
        print j.get_name()
    except:
        print 'No joystick'
        xb.halt()
        ser.close()
        sys.exit(-1)

    motorgains = motorgains = [1800,200,100,0,0, 1800,200,100,0,0]
    while not(shared.motor_gains_set):
        print "Setting motor gains..."
        xb_send(0, command.SET_PID_GAINS, pack('10h',*motorgains))
        time.sleep(1)
        xb_send(0, command.PID_START_MOTORS, "Start motor")

    try:    
        while True:

            value = []
            pygame.event.pump()
            left_per = -j.get_axis(1)
            right_per = -j.get_axis(3)
            if left_per < 0.01:
                left_per = 0
            if right_per < 0.01:
                right_per = 0
            left_per  = MAXPER -950*left_per
            right_per = MAXPER -950*right_per
            print "L: ",left_per,"  |   R: ",right_per
            sys.stdout.write(" "*60 + "\r")
            sys.stdout.flush()
            outstring = "L: {0:03.1f}  |   R: {1:03.1f} \r".format(left_per,right_per)
            sys.stdout.write(outstring)
            sys.stdout.flush()
            # pertle = [0 if t<0 else t for t in pertle]
            temp = [int(right_per), 0x1000, 0x1000, 0x1000, 0x1000, 0, \
                    int(left_per), 0x1000, 0x1000, 0x1000, 0x1000, 0]
            xb_send(0,command.SET_VEL_PROFILE, pack('12h',*temp))

            time.sleep(0.25)

    except:
        print
        print "closing"
        xb_send(0, command.PID_STOP_MOTORS, "Stop motor")

        try:
            xb.halt()
            ser.close()
        except serial.SerialException:
            print "Got SerialException."


#Provide a try-except over the whole main function
# for clean exit. The Xbee module should have better
# provisions for handling a clean exit, but it doesn't.
if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print "\nRecieved Ctrl+C, exiting."
        xb.halt()
        ser.close()
    except Exception as args:
        print "\nGeneral exception:",args
        print "Attemping to exit cleanly..."
        xb.halt()
        ser.close()
    except serial.serialutil.SerialException:
        xb.halt()
        ser.close()






