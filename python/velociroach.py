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

# TODO: check with firmware if this value is actually correct
PHASE_180_DEG = 0x8000

class GaitConfig:
    motorgains = None
    duration = None
    rightFreq = None
    leftFreq = None
    phase = None
    repeat = None
    deltasLeft = None
    deltasRight = None
    def __init__(self, motorgains, duration, rightFreq, leftFreq, phase, repeat):
        self.motorgains = motorgains
        self.duration = duration
        self.rightFreq = rightFreq
        self.leftFreq = leftFreq
        self.phase = phase
        self.repeat = repeat
        
        
class Velociroach:
    motor_gains_set = False
    robot_queried = False
    flash_erased = False
    motorGains = [0,0,0,0,0, 0,0,0,0,0]
    angRateDeg = 0;
    angRate = 0;
    dataFileName = ''
    imudata = [ [] ]
    numSamples = 0
    telemSampleFreq = 1000
    VERBOSE = True
    telemFormatString = '%d' # single type forces all data to be saved in this type

    def __init__(self, address, xb):
            self.DEST_ADDR = address
            self.DEST_ADDR_int = unpack('>h',self.DEST_ADDR)[0] #address as integer
            self.xb = xb
            print "Robot with DEST_ADDR = 0x%04X " % self.DEST_ADDR_int

    def clAnnounce(self):
        print "DST: 0x%02X | " % self.DEST_ADDR_int,
    
    def tx(self, status, type, data):
        payload = chr(status) + chr(type) + ''.join(data)
        self.xb.tx(dest_addr = self.DEST_ADDR, data = payload)
        
    def reset(self):
        self.clAnnounce()
        print "Resetting robot..."
        self.tx( 0, command.SOFTWARE_RESET, pack('h',1))
        
    def sendEcho(self, msg):
        self.tx( 0, command.ECHO, msg)
        
    def query(self, retries = 8):
        self.robot_queried = False
        tries = 1
        while not(self.robot_queried) and (tries <= retries):
            self.clAnnounce()
            print "Querying robot , ",tries,"/",retries
            self.tx( 0,  command.WHO_AM_I, "Robot Echo") #sent text is unimportant
            tries = tries + 1
            time.sleep(0.1)   
        
    def eraseFlashMem(self, timeout = 8):
        eraseStartTime = time.time()
        self.tx( 0, command.ERASE_SECTORS, pack('L',self.numSamples))
        self.clAnnounce()
        print "Started flash erase ..."
        while not (self.flash_erased):
            #sys.stdout.write('.')
            time.sleep(0.25)
            if (time.time() - eraseStartTime) > timeout:
                print"Flash erase timeout, retrying;"
                self.tx( 0, command.ERASE_SECTORS, pack('L',self.numSamples))
                eraseStartTime = time.time()    
        
    def setPhase(self, phase):
        self.tx( 0, command.SET_PHASE, pack('l', phase))
        time.sleep(0.01)        
    
    def startTimedRun(self, duration):
        self.tx( 0, command.START_TIMED_RUN, pack('h', phase))
        time.sleep(duration / 1000.0)
        
    def findFileName(self):   
        # Construct filename
        path     = 'Data/'
        name     = 'trial'
        datetime = time.localtime()
        dt_str   = time.strftime('%Y.%m.%d_%H.%M.%S', datetime)
        root     = path + dt_str + '_' + name
        self.dataFileName = root + '_imudata.txt'
        #self.clAnnounce()
        #print "Data file:  ", shared.dataFileName
        
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
        self.tx( 0, command.SET_VEL_PROFILE, pack('12h', phase))   
        
    def setMotorMode(motorgains, retries = 8 ):
        tries = 1
        self.motorGains = motorgains
        self.motor_gains_set = False
        while not(self.motor_gains_set) and (tries <= retries):
            self.clAnnounce()
            print "Setting motor gains...   ",tries,"/8"
            self.tx( 0, command.SET_MOTOR_MODE, pack('10h',*gains))
            tries = tries + 1
            time.sleep(0.1)
    
    ######TODO : sort out this function and flashReadback below
    def downloadTelemetry(self, timeout = 5, retry = True):
        #suppress callback output messages for the duration of download
        self.VERBOSE = False
        self.clAnnounce()
        print "Started telemetry download"
        self.tx( 0, command.FLASH_READBACK, pack('=L',self.numSamples))
                
        dlStart = time.time()
        shared.last_packet_time = dlStart
        #bytesIn = 0
        while self.imudata.count([]) > 0:
            time.sleep(0.02)
            dlProgress(self.numSamples - self.imudata.count([]) , self.numSamples)
            if (time.time() - shared.last_packet_time) > timeout:
                print ""
                #Terminal message about missed packets
                self.clAnnounce()
                print "Readback timeout exceeded"
                print "Missed", self.imudata.count([]), "packets."
                #print "Didn't get packets:"
                #for index,item in enumerate(self.imudata):
                #    if item == []:
                #        print "#",index+1,
                print "" 
                break
                # Retry telem download            
                if retry == True:
                    raw_input("Press Enter to restart telemetry readback ...")
                    self.imudata = [ [] ] * self.numSamples
                    self.clAnnounce()
                    print "Started telemetry download"
                    dlStart = time.time()
                    shared.last_packet_time = dlStart
                    self.tx( 0, command.FLASH_READBACK, pack('=L',self.numSamples))
                else: #retry == false
                    print "Not trying telemetry download."          

        dlEnd = time.time()
        dlTime = dlEnd - dlStart
        #Final update to download progress bar to make it show 100%
        dlProgress(self.numSamples-self.imudata.count([]) , self.numSamples)
        #totBytes = 52*self.numSamples
        totBytes = 52*(self.numSamples - self.imudata.count([]))
        datarate = totBytes / dlTime / 1000.0
        print '\n'
        #self.clAnnounce()
        #print "Got ",self.numSamples,"samples in ",dlTime,"seconds"
        self.clAnnounce()
        print "DL rate: {0:.2f} KB/s".format(datarate)
        
        #enable callback output messages
        self.VERBOSE = True

        print ""
        self.saveImudata()
        #Done with flash download and save

    def saveImudata(self):
        self.findFileName()
        self.writeFileHeader()
        fileout = open(self.dataFileName, 'a')
        np.savetxt(fileout , np.array(self.imudata), self.telemFormatString, delimiter = ',')
        fileout.close()
        self.clAnnounce()
        print "Telemetry data saved to", self.dataFileName
        
    def writeFileHeader(self, params, manParams):
        fileout = open(self.dataFileName,'w')
        #write out parameters in format which can be imported to Excel
        today = time.localtime()
        date = str(today.tm_year)+'/'+str(today.tm_mon)+'/'+str(today.tm_mday)+'  '
        date = date + str(today.tm_hour) +':' + str(today.tm_min)+':'+str(today.tm_sec)
        fileout.write('%  Data file recorded ' + date + '\n')

        if manParams.useFlag == True:
            fileout.write('%  Stride Frequency         = ' +repr(manParams.strideFreq) + '\n')
            fileout.write('%  Lead In /Lead Out         = ' +repr(manParams.leadIn) +','+repr(manParams.leadOut) + '\n')
            fileout.write('%  Deltas (Fractional)         = ' +repr(manParams.deltas) + '\n')
        else:    
            fileout.write('%  Right Stride Frequency         = ' +repr(params.rightFreq) + '\n')
            fileout.write('%  Left Stride Frequency         = ' +repr(params.leftFreq) + '\n')
            fileout.write('%  Phase (Fractional)         = ' +repr(params.phase) + '\n')

        fileout.write('%  Experiment.py \n')
        fileout.write('%  Motor Gains    = ' + repr(params.motorgains) + '\n')
        fileout.write('% Columns: \n')
        # order for wiring on RF Turner
        fileout.write('% time | Right Leg Pos | Left Leg Pos | Commanded Right Leg Pos | Commanded Left Leg Pos | DCR | DCL | GyroX | GryoY | GryoZ | AX | AY | AZ | RBEMF | LBEMF | VBatt\n')
        fileout.close()

    def setupImudata(self, numSamples = None, runtime = None):
        ''' This is NOT current for Velociroach! '''
        #TODO : update for velociroach
        
        # Take the longer number, between numSamples and runTime
        nrun = int(self.telemSampleFreq * runtime / 1000.0)
     
        self.numSamples = max([ nrun , numSamples ])
        
        #allocate an array to write the downloaded telemetry data into
        self.imudata = [ [] ] * self.numSamples
        self.clAnnounce()
        print "Telemetry samples to save: ",self.numSamples
    
    
    # execute move command
    def proceed(self, params):
        thrust = [params.throttle[0], params.duration, params.throttle[1], params.duration, 0]
        self.tx(0, command.SET_THRUST_CLOSED_LOOP, pack('5h',*thrust))
        print "Throttle = ",params.throttle,"duration =", params.duration
        time.sleep(0.01)


    def startTelemetrySave(self, numSamples):
        self.clAnnounce()
        print "Started telemetry save of", self.numSamples," samples".
        self.tx(0, command.START_TELEMETRY, pack('L',self.numSamples))


    def setGait(self, gaitConfig):
        
        
        
########## Helper functions #################

def setupSerial(COMPORT , BAUDRATE , timeout = 3, rtscts = 0):
    print "Setting up serial ..."
    try:
        ser = serial.Serial(port = COMPORT, baudrate = BAUDRATE, \
                    timeout=timeout, rtscts=rtscts)
    except serial.serialutil.SerialException:
        print "Could not open serial port:",shared.BS_COMPORT
        sys.exit(1)
    
    shared.ser = ser
    ser.flushInput()
    ser.flushOutput()
    return XBee(ser, callback = xbee_received)
    
    
def xb_safe_exit(xb):
    print "Halting xb"
    if xb is not None:
        xb.halt()
    else:
        shared.xb.halt()
        
    print "Closing serial"
    shared.ser.close()
    print "Exiting..."
    sys.exit(1)
    

   
def verifyAllMotorGainsSet():
    #Verify all robots have motor gains set
    for r in shared.ROBOTS:
        if not(r.motor_gains_set):
            print "CRITICAL : Could not SET MOTOR GAINS on robot 0x%02X" % r.DEST_ADDR_int
            xb_safe_exit()
            
def verifyAllTailGainsSet():
    #Verify all robots have motor gains set
    for r in shared.ROBOTS:
        if not(r.tail_gains_set):
            print "CRITICAL : Could not SET TAIL GAINS on robot 0x%02X" % r.DEST_ADDR_int
            xb_safe_exit()
            
def verifyAllQueried():            
    for r in shared.ROBOTS:
        if not(r.robot_queried):
            print "CRITICAL : Could not query robot 0x%02X" % r.DEST_ADDR_int
            xb_safe_exit()

    