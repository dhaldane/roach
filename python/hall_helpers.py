import glob
import time
import sys
from lib import command
from callbackFunc import xbee_received
import datetime
import serial
import shared
from struct import pack
from xbee import XBee
from math import ceil,floor
import numpy as np

class _Getch:
    """Gets a single character from standard input.  Does not echo to the
screen."""
    def __init__(self):
        try:
            self.impl = _GetchWindows()
        except ImportError:
            self.impl = _GetchUnix()

    def __call__(self): return self.impl()


class _GetchUnix:
    def __init__(self):
        import tty, sys

    def __call__(self):
        import sys, tty, termios
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch


class _GetchWindows:
    def __init__(self):
        import msvcrt

    def __call__(self):
        import msvcrt
        return msvcrt.getch()


getch = _Getch()

class hallParams:
    motorgains = []
    duration = []
    rightFreq = []
    leftFreq = []
    phase = []
    telemetry = []
    repeat = []
    def __init__(self, motorgains, duration, rightFreq, leftFreq, phase, telemetry, repeat):
        self.motorgains = motorgains
        self.duration = duration
        self.rightFreq = rightFreq
        self.leftFreq = leftFreq
        self.phase = phase
        self.telemetry = telemetry
        self.repeat = repeat

class manueverParams:
    leadIn      = []
    leadOut     = []
    strideFreq  = []
    phase       = []
    useFlag     = []
    deltas      = []
    def __init__(self, leadIn, leadOut, strideFreq, phase, useFlag, deltas):
        self.leadIn =  leadIn     
        self.leadOut =  leadOut    
        self.strideFreq =  strideFreq 
        self.phase =  phase 
        self.useFlag =  useFlag    
        self.deltas =  deltas     
        


def xb_safe_exit():
    print "Halting xb"
    shared.xb.halt()
    print "Closing serial"
    shared.ser.close()
    print "Exiting..."
    sys.exit(1)

def xb_send(status, type, data):
    payload = chr(status) + chr(type) + ''.join(data)
    shared.xb.tx(dest_addr = shared.DEST_ADDR, data = payload)
    # print ":".join("{:02x}".format(ord(c)) for c in payload) # Debug printing

#send user selected command
def rawCommand():
    xb_send(0,0xff, pack('h',0))

def menu():
    print "-------------------------------------"
    print "Keyboard control Nov. 12, 2011"
    print "m:menu     |q:quit     |r:reset      |n: name?"
    print "w:left+    |s:left-    |x:left off   |b: bogus command"
    print "e:right+   |d:right-   |c:right off  |<sp>: all off"
    print "g:R gain   |l: L gain  |t:duration   |v: vel profile |p: proceed"

def settingsMenu(params, manParams):
    print "t:duration   |m:telemetry   |p = motion profile  |b = Motor position   |n = deltas"
    print "Proceed: Space Bar"
    while True:
        print '>',
        keypress = getch()
        if keypress == ' ':
            break
        elif keypress == 't':
            print 'current duration',params.duration,' new value:',
            params.duration = int(raw_input())
        elif keypress == 'm':
            params.telemetry = not(params.telemetry)
            print 'Telemetry recording', params.telemetry
        elif keypress == 'o':
            print 'Enter Gains (Roll P, Yaw P) : ',
            x = raw_input()
            if len(x):
                pwmDes = map(int,x.split(','))
                xb_send(0, command.SET_THRUST_OPEN_LOOP, pack('2h', *pwmDes))
                print 'Gains: ', pwmDes
            else:
                print 'narp'
        elif keypress == 'e':
                xb_send(0, command.GET_IMU_DATA, "0")

        elif keypress == 'd':
            import pdb
            pdb.set_trace()
        elif keypress == 'b':
            print 'Set Motor Position (revs)'
            x = raw_input()
            if len(x):
                temp = map(int,x.split(','))
            pos = []
            pos.append(temp[0] * 65536)
            pos.append(1)
            xb_send(0, command.PID_START_MOTORS, "0")
            xb_send(0, command.SET_MOTOR_POS, pack('lh', *pos))
            print pos
            time.sleep(params.duration/1000.0)
            xb_send(0, command.PID_STOP_MOTORS, "0")

        elif keypress == 'g':
            print 'Channel, motor gains [Kp Ki Kd Kanti-wind ff]=', params.motorgains[5:11]  
            x = None
            while not x:
                try:
                    print 'Enter motor gains ,<csv> [Kp, Ki, Kd, Kanti-wind, ff]',
                    x = raw_input()
                except ValueError:
                    print 'Invalid Number'
            if len(x):
                motor = map(int,x.split(','))
                if len(motor) == 6:
                    if motor[0]:
                        params.motorgains[5:11] = motor[1:]
                        shared.motor_gains_set = False
                        setMotorGains(params.motorgains)
                    else:
                        params.motorgains[0:5] = motor[1:]
                        shared.motor_gains_set = False
                        setMotorGains(params.motorgains)
                    print 'Gains set:', motor
                else:
                    print 'not enough values'            

        elif keypress == 'p':

            print 'Set Body Angle (degrees)'
            x = raw_input()
            if len(x):
                temp = map(int,x.split(','))
                temp[0] = temp[0]*16384
                xb_send(0, command.RESET_BODY_ANG, "0")
                xb_send(0, command.PID_START_MOTORS, "0")
                xb_send(0, command.SET_PITCH_SET, pack('l', *temp))
                time.sleep(params.duration/1000.0)
                xb_send(0, command.PID_STOP_MOTORS, "0")

        elif keypress == 'q': 
            print "Exit."
            shared.xb.halt()
            shared.ser.close()
            sys.exit(0)

def repeatMenu(params):
    print "SPACE: Repeat with same settings  |q:quit"
    print "s: Settings                       |z:zero motors"
    while True:
        print '>',
        keypress = getch()
        if keypress == ' ':
            params.repeat = True
            break
        elif keypress == 's':
            params.repeat = False
            break
        elif keypress == 'z':
             xb_send(0, command.ZERO_POS,  "Zero motor")
        elif keypress == 'q': 
            print "Exit."
            shared.xb.halt()
            shared.ser.close()
            sys.exit(0)

def setVelProfile(params, manParams, manFlag):
    p = 1000.0/manParams.strideFreq
    if manFlag == True:
        delta = manParams.deltas
        deltaConv = 0x4000
        lastLeftDelta = 1-sum(manParams.deltas[:3])
        lastRightDelta = 1-sum(manParams.deltas[3:])
        temp = [int(p), int(delta[0]*deltaConv), int(delta[1]*deltaConv), int(delta[2]*deltaConv), int(lastLeftDelta*deltaConv) , 1, \
                int(p), int(delta[3]*deltaConv), int(delta[4]*deltaConv), int(delta[5]*deltaConv), int(lastRightDelta*deltaConv), 1]
    # Alternating Tripod
    else: 
        temp = [int(1000.0/params.rightFreq), 0x1000, 0x1000, 0x1000, 0x1000, 0, \
                int(1000.0/params.leftFreq), 0x1000, 0x1000, 0x1000, 0x1000, 0]
    print temp
    xb_send(0,command.SET_VEL_PROFILE, pack('12h',*temp))

def runManeuver(params, manParams):
    p = 1.0/manParams.strideFreq
    temp = [100,0,0,0,0,0,100,0,0,0,0,0]
    xb_send(0,command.SET_VEL_PROFILE, pack('12h',*temp))
    time.sleep(0.01)
    xb_send(0, command.START_TIMED_RUN, pack('h',int(1000*p*(manParams.leadOut+manParams.leadIn+1))))
    time.sleep(0.01)
    setVelProfile(params,manParams,False)
    time.sleep(p*(manParams.leadIn - 0.5))
    setVelProfile(params,manParams,True)
    time.sleep(manParams.leadOut * p)


# rVel = 1043*vel + 80*turn_rate
# lVel = 1043*vel - 80*turn_rate
    
#get velocity profile
def getVelProfile(params):

    sum = 0
    print 'set points in degrees e.g. 60,90,180,360:',
    x = raw_input()
    if len(x):
        temp = map(int,x.split(','))
        params.delta[0] = (temp[0]*42)/360
        sum = params.delta[0]
        for i in range(1,3):
            params.delta[i] = ((temp[i]-temp[i-1])*42)/360
            sum = sum + params.delta[i]
        params.delta[3]=42-sum
    else:
        print 'not enough delta values'
        
    print 'current duration (ms)',params.duration,' new value:',
    params.duration = int(raw_input())
    print 'enter % time of each segment <csv>',
    x = raw_input()
    if len(x):
        params.intervals = map(int,x.split(','))
        sum = 0
        for i in range(0,4):
            params.intervals[i] = params.duration*params.intervals[i]/100  # interval in ms
            sum = sum + params.intervals[i]
            params.vel[i] = (params.delta[i] <<8)/params.intervals[i]
        #adjust to total duration for rounding
        params.intervals[3] = params.intervals[3] + params.duration - sum
    else:
        print 'not enough values'
 #  print 'intervals (ms)',intervals
    params.duration = params.duration -1 # end on current segment
    
    #assign locally calculated values to parameter object:
    #params.delta = delta
    #params.duration = duration
    #params.intervals = intervals
    #params.vel = vel
        
    
# set robot control gains
def setMotorGains(motorgains):
    count = 0
    while not(shared.motor_gains_set):
        print "Setting motor gains. Packet:",count
        count = count + 1
        xb_send(0, command.SET_PID_GAINS, pack('10h',*motorgains))
        time.sleep(0.3)
        if count > 8:
            xb_safe_exit()

# allow user to set robot gain parameters
def readinGains(lr, params):
    print 'Rmotor gains [Kp Ki Kd Kanti-wind ff]=', params.motorgains[0:5]
    print 'Lmotor gains [Kp Ki Kd Kanti-wind ff]=', params.motorgains[5:11]  
    x = None
    while not x:
        try:
            print 'Enter ',lr,'motor gains ,<csv> [Kp, Ki, Kd, Kanti-wind, ff]',
            x = raw_input()
        except ValueError:
            print 'Invalid Number'
    if len(x):
        motor = map(int,x.split(','))
        if len(motor) == 5:
            print lr,'motor gains', motor
# enable sensing gains again
            shared.motor_gains_set = False
            if lr == 'R':
                params.motorgains[0:5] = motor
# temp - set both gains same
                params.motorgains[5:11] = motor
            else:
                params.motorgains[5:11] = motor
        else:
            print 'not enough gain values'
            
    

# execute move command
def proceed(params):
    thrust = [params.throttle[0], params.duration, params.throttle[1], params.duration, 0]
    xb_send(0, command.SET_THRUST_CLOSED_LOOP, pack('5h',*thrust))
    print "Throttle = ",params.throttle,"duration =", params.duration
    time.sleep(0.1)

def setMotorMode(motorgains):
    count = 0
    while not(shared.motor_gains_set):
        print "Setting motor gains. Packet:",count
        count = count + 1
        xb_send(0, command.SET_MOTOR_MODE, pack('10h',*motorgains))


def queryRobot():
    shared.robotQueried = False
    queries = 1
    while not(shared.robotQueried) and (queries < shared.maxQueries):
        print "Querying robot, try ",queries,"/",shared.maxQueries
        xb_send(0, command.WHO_AM_I, "Robot Echo")
        time.sleep(0.3)
        queries = queries + 1
    if queries == shared.maxQueries:
        print "Unable to query robot."
        xb_safe_exit()
    #Otherwise, control falls through back to the program

def setupSerial():
    print "Setting up serial ..."
    try:
        shared.ser = serial.Serial(shared.BS_COMPORT, shared.BS_BAUDRATE, \
                    timeout=3, rtscts=0)
    except serial.serialutil.SerialException:
        print "Could not open serial port:",shared.BS_COMPORT
        print "Exiting ..."
        sys.exit(1)
        
    if shared.ser.isOpen():
        print "Serial open. Using port",shared.BS_COMPORT
    shared.xb = XBee(shared.ser, callback = xbee_received)


def getDstAddrString():
    return hex(256* ord(shared.DEST_ADDR[0])+ ord(shared.DEST_ADDR[1]))
    
def sendWhoAmI():
    xb_send(0, command.WHO_AM_I, "Robot Echo") 

def flashReadback(numSamples, params, manParams):
    delay = 0.01
    # raw_input("Press any key to start readback of %d packets ..." % numSamples)
    print "started readback"
    shared.imudata = [ [] ] * numSamples  # reset imudata structure
    shared.pkts = 0  # reset packet count???
    xb_send(0, command.FLASH_READBACK, pack('=h',numSamples))
    # While waiting, write parameters to start of file
    print shared.dataFileName
    writeFileHeader(shared.dataFileName, params, manParams)     
    time.sleep(delay*numSamples + 1)
    # while shared.pkts != numSamples:
    #     print "Retry"
    #     shared.imudata = [ [] ] * numSamples
    #     shared.pkts = 0
    #     xb_send(0, command.FLASH_READBACK, pack('=h',numSamples))
    #     time.sleep(delay*numSamples + 3)
    #     if shared.pkts > numSamples:
    #         print "too many packets"
    #         break
    raw_input("\nReadback Done?")
    fileout = open(shared.dataFileName, 'a')
    np.savetxt(fileout , np.array([e for e in shared.imudata if len(e)]), '%d', delimiter = ',') # Write non-empty lists in imudata to file
    fileout.close()
    print "data saved to ",shared.dataFileName
        
def writeFileHeader(dataFileName, params, manParams):
    fileout = open(dataFileName,'w')
    #write out parameters in format which can be imported to Excel
    today = time.localtime()
    date = str(today.tm_year)+'/'+str(today.tm_mon)+'/'+str(today.tm_mday)+'  '
    date = date + str(today.tm_hour) +':' + str(today.tm_min)+':'+str(today.tm_sec)
    fileout.write('"Data file recorded ' + date + '"\n')

    fileout.write('"%  Experiment.py "\n')
    fileout.write('"%  Motor Gains    = ' + repr(params.motorgains) + '\n')
    fileout.write('"% Columns: "\n')
    fileout.write('time (us), Tail Positon, Femur Position, Motor Position, Est. Pitch, Est. Roll, Est. Yaw, Commanded Pitch, Commanded Motor Angle, Tail DC, Rear Prop DC, Front Prop DC, BLDC DC, GyroX, GryoY, GryoZ, AX ,AY ,AZ\n')
    fileout.close()

def eraseFlashMem(numSamples):
    xb_send(0, command.ERASE_SECTORS, pack('L',numSamples))
    print "Started erase, Enter to continue"

def startTelemetrySave(numSamples):
    temp=[numSamples]
    print 'temp =',temp,'\n'
    xb_send(0, command.START_TELEMETRY, pack('h',*temp))

def query_yes_no(question, default="yes"):
    """Ask a yes/no question via raw_input() and return their answer.

    "question" is a string that is presented to the user.
    "default" is the presumed answer if the user just hits <Enter>.
        It must be "yes" (the default), "no" or None (meaning
        an answer is required of the user).

    The "answer" return value is one of "yes" or "no".
    """
    valid = {"yes":True,   "y":True,  "ye":True,
             "no":False,     "n":False}
    if default == None:
        prompt = " [y/n] "
    elif default == "yes":
        prompt = " [Y/n] "
    elif default == "no":
        prompt = " [y/N] "
    else:
        raise ValueError("invalid default answer: '%s'" % default)

    while True:
        sys.stdout.write(question + prompt)
        choice = raw_input().lower()
        if default is not None and choice == '':
            return valid[default]
        elif choice in valid:
            return valid[choice]
        else:
            sys.stdout.write("Please respond with 'yes' or 'no' "\
                             "(or 'y' or 'n').\n")