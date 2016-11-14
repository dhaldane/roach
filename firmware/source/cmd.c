#include "cmd.h"
#include "cmd_const.h"
#include "dfmem.h"
#include "utils.h"
#include "ports.h"
#include "sclock.h"
#include "led.h"
#include "blink.h"
#include "payload.h"
#include "mac_packet.h"
#include "dfmem.h"
#include "radio.h"
#include "dfmem.h"
#include "tests.h"
#include "version.h"
#include "settings.h"
#include "timer.h"
#include "tih.h"
#include "pid-ip2.5.h"
#include "ams-enc.h"
#include "carray.h"
#include "telem.h"
#include "uart_driver.h"
#include "protocol.h"
#include "tail_ctrl.h"
#include "experiment.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

unsigned char (*cmd_func[MAX_CMD_FUNC])(unsigned char, unsigned char, unsigned char, unsigned char*, unsigned int);
void cmdError(void);

extern pidPos pidObjs[NUM_PIDS];
extern EncObj encPos[NUM_ENC];
extern EncObj motPos;
extern volatile CircArray fun_queue;

packet_union_t uart_tx_packet_cmd;

/*-----------------------------------------------------------------------------
 *          Declaration of static functions
-----------------------------------------------------------------------------*/
static unsigned char cmdNop(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static unsigned char cmdWhoAmI(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static unsigned char cmdGetAMSPos(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);

//Jumper functions
static unsigned char cmdSetPitchSetpoint(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static unsigned char cmdresetBodyAngle(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static unsigned char cmdSetMotorPos(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static unsigned char cmdStartExperiment(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static unsigned char cmdSetExperimentParams(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);

//Motor and PID functions
static unsigned char cmdSetThrustOpenLoop(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static unsigned char cmdSetMotorMode(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static unsigned char cmdSetPIDGains(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static unsigned char cmdPIDStartMotors(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static unsigned char cmdPIDStopMotors(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static unsigned char cmdSetVelProfile(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static unsigned char cmdZeroPos(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
//Experiment/Flash Commands
static unsigned char cmdStartTimedRun(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static unsigned char cmdStartTelemetry(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static unsigned char cmdEraseSectors(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
static unsigned char cmdFlashReadback(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr);
/*-----------------------------------------------------------------------------
 *          Public functions
-----------------------------------------------------------------------------*/
void cmdSetup(void) {

    unsigned int i;

    // initialize the array of func pointers with Nop()
    for(i = 0; i < MAX_CMD_FUNC; ++i) {
        cmd_func[i] = &cmdNop;
    }
    cmd_func[CMD_TEST_RADIO] = &test_radio;
    cmd_func[CMD_TEST_MPU] = &test_mpu;
    cmd_func[CMD_SET_THRUST_OPEN_LOOP] = &cmdSetThrustOpenLoop;
    cmd_func[CMD_SET_MOTOR_MODE] = &cmdSetMotorMode;
    cmd_func[CMD_PID_START_MOTORS] = &cmdPIDStartMotors;
    cmd_func[CMD_SET_PID_GAINS] = &cmdSetPIDGains;
    cmd_func[CMD_GET_AMS_POS] = &cmdGetAMSPos;
    cmd_func[CMD_START_TELEMETRY] = &cmdStartTelemetry;
    cmd_func[CMD_ERASE_SECTORS] = &cmdEraseSectors;
    cmd_func[CMD_FLASH_READBACK] = &cmdFlashReadback;
    cmd_func[CMD_SET_VEL_PROFILE] = &cmdSetVelProfile;
    cmd_func[CMD_WHO_AM_I] = &cmdWhoAmI;
    cmd_func[CMD_ZERO_POS] = &cmdZeroPos;   
    cmd_func[CMD_START_TIMED_RUN] = &cmdStartTimedRun;
    cmd_func[CMD_PID_STOP_MOTORS] = &cmdPIDStopMotors;

    cmd_func[CMD_SET_PITCH_SET] = &cmdSetPitchSetpoint;
    cmd_func[CMD_RESET_BODY_ANG] = &cmdresetBodyAngle;
    cmd_func[CMD_SET_MOTOR_POS] = &cmdSetMotorPos;
    cmd_func[CMD_START_EXP] = &cmdStartExperiment;
    cmd_func[CMD_SET_EXP_PARAMS] = &cmdSetExperimentParams;

}

void cmdPushFunc(MacPacket rx_packet) {
    Payload rx_payload;
    unsigned char command;

    rx_payload = macGetPayload(rx_packet);
    if(rx_payload != NULL) {
        command = payGetType(rx_payload);

        if(command < MAX_CMD_FUNC && cmd_func[command] != NULL) {
            rx_payload->test = cmd_func[command];
            carrayAddTail(fun_queue, rx_packet);
        } else {
            cmdError();   // halt on error - could also just ignore....
        }
    }
}


// send robot info when queried
unsigned char cmdWhoAmI(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr) {
    unsigned char i, string_length; unsigned char *version_string;
    // maximum string length to avoid packet size limit
    version_string = (unsigned char*)versionGetString();
    i = 0;
    while((i < 127) && version_string[i] != '\0') {
        i++;
    }
    string_length=i;
    radioSendData(src_addr, status, CMD_WHO_AM_I, //TODO: Robot should respond to source of query, not hardcoded address
            string_length, version_string, 0);
    return 1; //success
}

unsigned char cmdGetAMSPos(unsigned char type, unsigned char status,
        unsigned char length, unsigned char *frame, unsigned int src_addr) {
    long motor_count[2];
    motor_count[0] = pidObjs[0].p_state;
    motor_count[1] = pidObjs[1].p_state;

    // motor_count[0] = encPos[0].pos;
    // motor_count[1] = encPos[1].pos;

    radioSendData(src_addr, status, CMD_GET_AMS_POS,  //TODO: Robot should respond to source of query, not hardcoded address
            sizeof(motor_count), (unsigned char *)motor_count, 0);
    return 1;
}
// ==== Jumper Commands ==============================================================================
// =============================================================================================================


unsigned char cmdStartExperiment(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr){
    uint8_t mode = frame[0];
    expStart(mode);
    return 1;
}

unsigned char cmdSetExperimentParams(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr){
    int32_t mode = frame[0];

    switch(mode) {
        case EXP_WALL_JUMP: ;
            int32_t params[5];
            int i = 0;
            int j = 0;
            int idx;
            for (i = 0; i < 5; i=i+1)
            {
                params[i] = 0;
                for (j = 0; j < 4; j=j+1)
                {
                    idx = j+4*i+4;
                    params[i] += ((long)frame[idx] << 8*j );
                }
            }
            exp_set_params_wj(params[0],params[1],params[2],params[3],params[4]);
            break;
        case EXP_SINGLE_JUMP: ;
            int16_t duration = frame[2] + (frame[3] << 8);
            int16_t leg_extension = frame[4] + (frame[5] << 8);
            int32_t conv_leg_extension;
            conv_leg_extension = (long)(leg_extension)*6554; //1/10 radian to 15.16 radians representation;
            exp_set_params_sj(duration, conv_leg_extension);
            break;
        default:
            break;
    }
    return 1;
}

unsigned char cmdSetPitchSetpoint(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr){
    long pos = 0;
    int i;
    
    for (i = 0; i < 4; i++)
    {
        pos += ((long)frame[i] << 8*i );
    }
    setPitchControlFlag(1);
    setPitchSetpoint(pos);
    return 1;
}

unsigned char cmdresetBodyAngle(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr){
    resetBodyAngle();
    return 1;
}

unsigned char cmdSetMotorPos(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr){
    long pos = 0;
    //int relative_flag = frame[4] + (frame[5] << 8);

    int i;
    for (i = 0; i < 4; i++)
    {
        pos += ((long)frame[i] << 8*i );
    }

    
    send_command_packet(&uart_tx_packet_cmd, pos, 0, 2); 

    return 1;
}

// ==== Flash/Experiment Commands ==============================================================================
// =============================================================================================================
unsigned char cmdStartTimedRun(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr){
    unsigned int run_time = frame[0] + (frame[1] << 8);
    int i;
    for (i = 0; i < NUM_PIDS; i++){
        pidObjs[i].timeFlag = 1;
        pidSetInput(i, 0);
        checkSwapBuff(i);
        pidOn(i);
    }
    pidObjs[0].mode = 0;
    pidStartTimedTrial(run_time);


    return 1;
}

unsigned char cmdStartTelemetry(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr){
    unsigned int numSamples = frame[0] + (frame[1] << 8);
    if (numSamples != 0) {
        telemSetStartTime(); // Start telemetry samples from approx 0 time
        telemSetSamplesToSave(numSamples);
    }
    return 1;
}
unsigned char cmdEraseSectors(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr){
    unsigned int numSamples = frame[0] + (frame[1] << 8);
    telemErase(numSamples);
    
    //Send confirmation packet; this only happens when flash erase is completed.
    radioSendData(src_addr, 0, CMD_ERASE_SECTORS, length, frame, 0);

    LED_RED = ~LED_RED;
    return 1;
}
unsigned char cmdFlashReadback(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr){
    unsigned int numSamples = frame[0] + (frame[1] << 8);
    telemReadbackSamples(numSamples, src_addr);
    return 1;
}

// ==== Motor PID Commands =====================================================================================
// =============================================================================================================

unsigned char cmdSetThrustOpenLoop(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr) {
    // int thrust1 = frame[0] + (frame[1] << 8);
    // int thrust2 = frame[2] + (frame[3] << 8);
    // unsigned int run_time_ms = frame[4] + (frame[5] << 8);



    // DisableIntT1;   // since PID interrupt overwrites PWM values

    // tiHSetDC(1, thrust1);
    // tiHSetDC(2, thrust2);
    // LED_3 = 1;
    // delay_ms(run_time_ms);
    // tiHSetDC(1,0);
    // tiHSetDC(2,0);

    // EnableIntT1;
    /// HIGHJACKING FUNCTION 8/3/2016 FOR PROP GAINS
    int Kpr = frame[0] + (frame[1] << 8);
    int Kdr = frame[2] + (frame[3] << 8);
    pidSetGains(2,Kpr,0,Kdr,0,0);
    // pidSetGains(3,Kpy,0,Kdy,0,0);
    return 1;
 } 

 unsigned char cmdSetMotorMode(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr) {

    int thrust1 = frame[0] + (frame[1] << 8);
    int thrust2 = frame[2] + (frame[3] << 8);

    pidObjs[0].pwmDes = thrust1;
    pidObjs[1].pwmDes = thrust2;

    pidObjs[0].mode = 1;
    return 1;
 }

unsigned char cmdSetPIDGains(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr) {
    int Kp, Ki, Kd, Kaw, ff;
    int idx = 0;

    Kp = frame[idx] + (frame[idx+1] << 8); idx+=2;
    Ki = frame[idx] + (frame[idx+1] << 8); idx+=2;
    Kd = frame[idx] + (frame[idx+1] << 8); idx+=2;
    Kaw = frame[idx] + (frame[idx+1] << 8); idx+=2;
    ff = frame[idx] + (frame[idx+1] << 8); idx+=2;
    pidSetGains(0,Kp,Ki,Kd,Kaw, ff);
    Kp = frame[idx] + (frame[idx+1] << 8); idx+=2;
    Ki = frame[idx] + (frame[idx+1] << 8); idx+=2;
    Kd = frame[idx] + (frame[idx+1] << 8); idx+=2;
    Kaw = frame[idx] + (frame[idx+1] << 8); idx+=2;
    ff = frame[idx] + (frame[idx+1] << 8); idx+=2;
    pidSetGains(1,Kp,Ki,Kd,Kaw, ff);

    radioSendData(src_addr, status, CMD_SET_PID_GAINS, 20, frame, 0); //TODO: Robot should respond to source of query, not hardcoded address
    //Send confirmation packet
    // WARNING: Will fail at high data throughput
    //radioConfirmationPacket(RADIO_DEST_ADDR, CMD_SET_PID_GAINS, status, 20, frame);
    return 1; //success
}

unsigned char cmdSetVelProfile(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr) {
    int interval[NUM_VELS], delta[NUM_VELS], vel[NUM_VELS], period, onceFlag;
    int idx = 0, i = 0;
    // Packet structure [Period, delta[NUM_VELS], FLAG, Period, delta[NUM_VELS], FLAG]
    period = frame[idx] + (frame[idx + 1]<<8);
    idx+=2;
    for(i = 0; i < NUM_VELS; i ++) {
        interval[i] = period/NUM_VELS;
        delta[i] = (frame[idx]+ (frame[idx+1]<<8));
            if(delta[i]>=8192){
                delta[i] = 8191;
            } else if(delta[i] < -8192){
                delta[i] = -8192;
            }
        delta[i] = delta[i]<<2;
        vel[i] = delta[i]/interval[i];
        idx+=2;
    }
    onceFlag = frame[idx] + (frame[idx + 1]<<8);
    idx+=2;    

    setPIDVelProfile(0, interval, delta, vel, onceFlag);
    
    period = frame[idx] + (frame[idx + 1]<<8);
    idx+=2;
    for(i = 0; i < NUM_VELS; i ++) {
        interval[i] = period/NUM_VELS;
        delta[i] = (frame[idx]+ (frame[idx+1]<<8));
            if(delta[i]>=8192){
                delta[i] = 8191;
            } else if(delta[i] < -8192){
                delta[i] = -8192;
            }
        delta[i] = delta[i]<<2;
        vel[i] = delta[i]/interval[i];
        idx+=2;
        }
    onceFlag = frame[idx] + (frame[idx + 1]<<8);

    setPIDVelProfile(1, interval, delta, vel, onceFlag);

    //Send confirmation packet
    // TODO : Send confirmation packet with packet index
    return 1; //success
}

unsigned char cmdPIDStartMotors(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr) {
    
    int i;
    for(i=0;i<NUM_PIDS;i++){  
    pidObjs[i].timeFlag = 0;
    pidSetInput(i, 0);
    pidObjs[i].p_input = pidObjs[i].p_state;
    pidOn(i);
    }
    setPitchControlFlag(1);

    return 1;
}

unsigned char cmdPIDStopMotors(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr) {
    int i;
    for(i=0;i<NUM_PIDS;i++){  
        pidObjs[i].onoff = 0;
    }
    return 1;
}

unsigned char cmdZeroPos(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr) {
    long motor_count[2];
    motor_count[0] = pidObjs[0].p_state;
    motor_count[1] = pidObjs[1].p_state;

    radioSendData(src_addr, status, CMD_ZERO_POS, 
        sizeof(motor_count), (unsigned char *)motor_count, 0);
    pidZeroPos(0);
    pidZeroPos(1);
    return 1;
}

void cmdError() {
    int i;
    EmergencyStop();
    for(i= 0; i < 10; i++) {
        LED_1 ^= 1;
        delay_ms(200);
        LED_2 ^= 1;
        delay_ms(200);
        LED_3 ^= 1;
        delay_ms(200);
    }
}

static unsigned char cmdNop(unsigned char type, unsigned char status, unsigned char length, unsigned char *frame, unsigned int src_addr) {
    return 1;
}