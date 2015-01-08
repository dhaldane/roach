from lib import command
from struct import pack,unpack
import time,sys,os,traceback

# Path to imageproc-settings repo must be added
sys.path.append(os.path.dirname("../../imageproc-settings/"))
sys.path.append(os.path.dirname("../imageproc-settings/"))      # Some projects have a single-directory structure
import shared

#Dictionary of packet formats, for unpack()
pktFormat = { \
    command.TX_DUTY_CYCLE:          'l3f', \
    command.GET_IMU_DATA:           'l6h', \
    command.TX_SAVED_STATE_DATA:    'l3f', \
    command.SET_THRUST_OPEN_LOOP:   '', \
    command.PID_START_MOTORS:       '', \
    command.SET_PID_GAINS:          '10h', \
    command.GET_PID_TELEMETRY:      '', \
    command.GET_AMS_POS:            '=2l', \
    command.GET_IMU_LOOP_ZGYRO:     '='+2*'Lhhh', \
    command.SET_MOVE_QUEUE:         '', \
    command.SET_STEERING_GAINS:     '6h', \
    command.SOFTWARE_RESET:         '', \
    command.ERASE_SECTORS:          'L', \
    command.FLASH_READBACK:         '=LL' +'4l'+'11h', \
    command.SLEEP:                  'b', \
    command.ECHO:                   'c' ,\
    command.SET_VEL_PROFILE:        '8h' ,\
    command.WHO_AM_I:               '', \
    command.ZERO_POS:               '=2l', \
    }
               
#XBee callback function, called every time a packet is recieved
def xbee_received(packet):
    rf_data = packet.get('rf_data')
    #rssi = ord(packet.get('rssi'))
    #(src_addr, ) = unpack('H', packet.get('source_addr'))
    #id = packet.get('id')
    #options = ord(packet.get('options'))
   
    status = ord(rf_data[0])
    type = ord(rf_data[1])
    # print 'Received %d' % type
    data = rf_data[2:]   
    
    #Record the time the packet is received, so command timeouts
    # can be done
    shared.last_packet_time = time.time()
    
    try:
        pattern = pktFormat[type]
    except KeyError:
        print "Got bad packet type: ",type
        return
    
    try:
        # GET_IMU_DATA
        if type == command.GET_IMU_DATA:
            datum = unpack(pattern, data)
            if (datum[0] != -1):
                shared.imudata.append(datum)
                print "got datum:",datum
        # TX_SAVED_STATE_DATA
        elif type == command.TX_SAVED_STATE_DATA:
            datum = unpack(pattern, data)
            if (datum[0] != -1):
                statedata.append(datum)
         # TX_DUTY_CYCLE
        elif type == command.TX_DUTY_CYCLE:
            datum = unpack(pattern, data)
            if (datum[0] != -1):
                dutycycles.append(datum)
        # ECHO
        elif type == command.ECHO:
            print "echo: status = ",status," type=",type," data = ",data
        # SET_PID_GAINS
        elif type == command.SET_PID_GAINS:
            print "Set PID gains"
            gains = unpack(pattern, data)
            print gains
            shared.motor_gains_set = True 
        # SET_STEERING_GAINS
        elif type == command.SET_STEERING_GAINS:
            print "Set Steering gains"
            gains = unpack(pattern, data)
            print gains
            shared.steering_gains_set = True
        # GET_IMU_LOOP_ZGYRO
        elif type == command.GET_IMU_LOOP_ZGYRO:
            pp = 2;
            print "Z Gyro Data Packet"
            datum = unpack(pattern, data)
            if (datum[0] != -1):
                for i in range(pp):
                    shared.imudata.append(datum[4*i:4*(i+1)] )
        # FLASH_READBACK
        elif type == command.FLASH_READBACK:
            shared.pkts = shared.pkts + 1
            #print "Special Telemetry Data Packet, ",shared.pkts
            datum = unpack(pattern, data)
            datum = list(datum)
            telem_index = datum.pop(0)
            print "Special Telemetry Data Packet #", telem_index, '\r',
            # print datum
            if (datum[0] != -1) and (telem_index) >= 0: #valid index
                shared.imudata[telem_index] = datum
                shared.bytesIn = shared.bytesIn + (5*4 + 11*2)
        # ERASE_SECTORS
        elif type == command.ERASE_SECTORS:
            datum = unpack(pattern, data)
            #if datum[0] == 0:
            #    shared.flash_erased = True
            shared.flash_erased = datum[0]
        # SLEEP
        elif type == command.SLEEP:
            datum = unpack(pattern, data)
            print "Sleep reply: ",datum[0]
            if datum[0] == 0:
                shared.awake = True;
        # ZERO_POS
        elif type == command.ZERO_POS:
            print 'Hall zeros established; Previous motor positions:',
            motor = unpack(pattern,data)
            print motor
        # SET_VEL_PROFILE
        elif (type == command.SET_VEL_PROFILE):
            print "Set Velocity Profile readback:"
            temp = unpack(pattern, data)
            print temp
        # WHO_AM_I
        elif (type == command.WHO_AM_I):
            #print "whoami:",status, hex(type), data
            print "whoami:",data
            shared.robotQueried = True
        else:    
            pass
    
    except Exception as args:
        print "\nGeneral exception from callbackfunc:",args
        print "\n    ******    TRACEBACK    ******    "
        traceback.print_exc()
        print "    *****************************    \n"
        print "Attempting to exit cleanly..."
        shared.xb.halt()
        shared.ser.close()
        sys.exit()


