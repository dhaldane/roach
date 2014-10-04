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
    print "t:duration   |m:telemetry   |p = motion profile  |b = Motion queue   |n = deltas"
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
            print 'Enter Duty Cycle (Left,Right) : ',
            x = raw_input()
            if len(x):
                pwmDes = map(float,x.split(','))
            xb_send(0, command.SET_MOTOR_MODE, pack('2h', pwmDes))
            print 'Set Duty cycle: ', pwmDes
        elif keypress == 'b':
            print 'Manuever Enabled'
            manParams.useFlag = True
            print 'Enter movement string (#Lead in Strides, Lead Out, Stride Frequncy (Hz):',
            x = raw_input()
            if len(x):
                temp = map(float,x.split(','))
            manParams.leadIn = temp[0]
            manParams.leadOut = temp[1]
            manParams.strideFreq = temp[2]
        elif keypress == 'n':
            print 'Manuever Enabled'
            manParams.useFlag = True
            print 'Enter 6 fractional deltas (0-1) L-R : ',
            x = raw_input()
            if len(x):
                manParams.deltas = map(float,x.split(','))
            print 'Deltas: ', manParams.deltas
        elif keypress == 'p':
            print 'Manuever Disabled'
            manParams.useFlag = False
            print 'Right Leg Frequency, Left Leg Frequency, Phase (degrees): ',
            x = raw_input()
            if len(x):
                temp = map(float,x.split(','))
            params.rightFreq = temp[0]
            params.leftFreq = temp[1]
            params.phase = temp[2] * 65536.0/360
            setVelProfile(params, manParams, 0)

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
            xb.halt()
            ser.close()
            sys.exit(0)
            
            
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




def getDstAddrString():
    return hex(256* ord(shared.DEST_ADDR[0])+ ord(shared.DEST_ADDR[1])) 
    
    
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

# interactively get velocity profile from user
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
        
# execute move command
def proceed(self, params):
    thrust = [params.throttle[0], params.duration, params.throttle[1], params.duration, 0]
    self.tx(0, command.SET_THRUST_CLOSED_LOOP, pack('5h',*thrust))
    print "Throttle = ",params.throttle,"duration =", params.duration
    time.sleep(0.01)  
