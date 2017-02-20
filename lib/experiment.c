#include "experiment.h"
#include "as5047.h"
#include "pid-ip2.5.h"
#include "ams-enc.h"
#include "tail_ctrl.h"
#include "led.h"
#include "mpu6000.h"
#include "uart_driver.h"
#include "protocol.h"
// #include "math.h"
#include "lut.h"


#define ABS(my_val) ((my_val) < 0) ? -(my_val) : (my_val)
#define LSB2ENCRAD 0.000095873799
#define ENCRAD2LSB 10430

#define EXP_IDLE            0

#define EXP_WJ_START        1
#define EXP_WJ_JUMP_TRIG    2
#define EXP_WJ_JUMP         3
#define EXP_WJ_RETRACT      4
#define EXP_WJ_READY_LEG    5
#define EXP_WJ_STOP         6

#define EXP_SJ_START        7
#define EXP_SJ_STOP         8


volatile unsigned char mj_state = MJ_IDLE;
#define UART_PERIOD     10
#define FULL_EXTENSION  14000


volatile unsigned char  exp_state = EXP_IDLE;
volatile unsigned long t_start;

experiment_params_sj_t sj_params;
experiment_params_wj_t wj_params;

packet_union_t uart_tx_packet_global;

#define MOT_FLAG_STOP 0x80

extern EncObj motPos;
extern EncObj encPos[NUM_ENC];
extern pidPos pidObjs[NUM_PIDS];
extern unsigned long t1_ticks;
extern long body_angle[3];
volatile long legSetpoint;
volatile long pushoffCmd;


void setLegSetpoint(long length){
    legSetpoint = length << 8;
}

void setPushoffCmd(long cmd){
    pushoffCmd = cmd << 8;
}


long transition_time = 0;
void multiJumpFlow() {
    int gdata[3];
    mpuGetGyro(gdata);
    switch(mj_state) {
        case MJ_START:
            pidObjs[0].timeFlag = 0;
            setPitchControlFlag(1);
            pidOn(0);
            pidOn(2);
            pidOn(3);

            mj_state = MJ_GND;
            t_start = 0; // TODO: fix this initialization
            break;

        case MJ_STOP:
            pidObjs[0].onoff = 0;
            tiHSetDC(1,0);
            pidObjs[2].onoff = 0;
            tiHSetDC(3,0);
            pidObjs[3].onoff = 0;
            tiHSetDC(4,0);
            if(t1_ticks - t_start > UART_PERIOD) {
                send_command_packet(&uart_tx_packet_global, 0, 0, 0);
                t_start = t1_ticks;
                mj_state = MJ_IDLE;
            }
            break;

        case MJ_AIR:
            if(t1_ticks - t_start > UART_PERIOD) {
                //pidSetGains(2,0,0,100,0,0); // for debugging states
                send_command_packet(&uart_tx_packet_global, legSetpoint, 0, 2);
                t_start = t1_ticks; //TODO: build command rate limit into send_command_packet function
            }

            // Ground contact transition out of air to ground
            if (t_start - transition_time > 200 && (footContact() == 1)) {
                mj_state = MJ_GND;
                transition_time = t1_ticks;
            }
            break;

        case MJ_GND:
            if(t1_ticks - t_start > UART_PERIOD) {
                //pidSetGains(2,0,0,0,0,0); // for debugging states
                send_command_packet(&uart_tx_packet_global, pushoffCmd, 0, 2);
                t_start = t1_ticks;
            }

            // Liftoff transition from ground to air
            if (t_start - transition_time > 50 && (footTakeoff() == 1 || calibPos(2) > FULL_EXTENSION)) {
                mj_state = MJ_AIR;
                transition_time = t1_ticks;
            }
            break;

        default:
            mj_state = MJ_IDLE;
            break;

    }

}

void expFlow() {
    int gdata[3];
    mpuGetGyro(gdata);
// Wall Jump
    switch(exp_state) {

        case EXP_WJ_STOP:
            if((t1_ticks-t_start) > 600){
                pidObjs[0].onoff = 0;
                pidObjs[2].onoff = 0;
                pidObjs[3].onoff = 0;
                send_command_packet(&uart_tx_packet_global, 0, 0, 0);
                exp_state = EXP_IDLE;
            } else {
                exp_state = EXP_WJ_STOP;
            }
            break;
        case EXP_WJ_READY_LEG:
            exp_state = EXP_WJ_READY_LEG;
            // if(footContact() == 1 || gdata[0] < -6000){
            if(gdata[0] < -6000){
                LED_2 = !LED_2;
                send_command_packet(&uart_tx_packet_global, wj_params.leg_extension, 0, 2);
                setPitchSetpoint(900000);  
                exp_state = EXP_WJ_STOP;
            }
            break;
        case EXP_WJ_RETRACT:
            exp_state = EXP_WJ_RETRACT;
            if((t1_ticks-t_start) > 280){exp_state = EXP_WJ_READY_LEG;}
            break;     
        case EXP_WJ_JUMP:
            exp_state = EXP_WJ_JUMP;
            if((t1_ticks-t_start) > UART_PERIOD){
                setPitchSetpoint(wj_params.landing_angle); // 1000000
                send_command_packet(&uart_tx_packet_global, wj_params.leg_retraction, 0, 2); // 411774
                exp_state = EXP_WJ_RETRACT;
            }
            break;     
        case EXP_WJ_JUMP_TRIG:
            exp_state = EXP_WJ_JUMP_TRIG;
            if(body_angle[2] < wj_params.launch_threshold){ // -175000
                send_command_packet(&uart_tx_packet_global, wj_params.leg_extension, 0, 2); //4941297 Move to 12 revs forward
                exp_state = EXP_WJ_JUMP; 
                t_start = t1_ticks;
            }
            break;
        case EXP_WJ_START:
            pidObjs[0].timeFlag = 0;
            resetBodyAngle();
            setPitchControlFlag(1);
            setPitchSetpoint(wj_params.launch_angle); //-551287 25 degrees forward
            pidSetInput(0, 0);
            pidOn(0);
            exp_state = EXP_WJ_JUMP_TRIG;
            break;
// Single Jump
        case EXP_SJ_STOP:
            if((t1_ticks-t_start) > sj_params.duration){
                pidObjs[0].onoff = 0;
                pidObjs[2].onoff = 0;
                pidObjs[3].onoff = 0;
                send_command_packet(&uart_tx_packet_global, 0, 0, 0);
                exp_state = EXP_IDLE;
            } else {
                exp_state = EXP_SJ_STOP;
            }
            break;

        case EXP_SJ_START:
             pidObjs[0].timeFlag = 0;
             resetBodyAngle();
             setPitchControlFlag(1);
             setPitchSetpoint(0); //25 degrees forward
             pidSetInput(0, 0);
             pidOn(0);
            send_command_packet(&uart_tx_packet_global, sj_params.leg_extension, 0, 2); // send command packet
            exp_state = EXP_SJ_STOP;
            t_start = t1_ticks;
            break;

        default:
            exp_state = EXP_IDLE;
            break;
        }

}

void expStart(uint8_t mode) {
    switch(mode){
        case EXP_WALL_JUMP:
            exp_state = EXP_WJ_START;
            break;

        case EXP_SINGLE_JUMP:
            exp_state = EXP_SJ_START;
            break;

        case EXP_VICON:
            mj_state = MJ_START;
            break;

        default:
            exp_state = EXP_IDLE;
            break;

    }
}

void expStop(uint8_t stopSignal) {
    mj_state = MJ_STOP;
}


void exp_set_params_sj(int16_t duration, int32_t leg_extension) {
    sj_params.duration = duration;
    sj_params.leg_extension = leg_extension;
}

void exp_set_params_wj(int32_t launch_angle, int32_t launch_threshold, int32_t landing_angle, int32_t leg_extension, int32_t leg_retraction) {
    wj_params.launch_angle = launch_angle;
    wj_params.launch_threshold = launch_threshold;
    wj_params.landing_angle = landing_angle;
    wj_params.leg_extension = leg_extension;
    wj_params.leg_retraction = leg_retraction;
}


void send_command_packet(packet_union_t *uart_tx_packet, int32_t position, uint32_t current, uint8_t flags){
    // Create dummy UART TX packet
    uart_tx_packet->packet.header.start = PKT_START_CHAR;
    uart_tx_packet->packet.header.type = PKT_TYPE_COMMAND;
    uart_tx_packet->packet.header.length = sizeof(header_t) + sizeof(command_data_t) + 1;
    command_data_t* command_data = (command_data_t*)&(uart_tx_packet->packet.data_crc);
    
    // Settable things
    uart_tx_packet->packet.header.flags = flags;
    command_data->position_setpoint = position;
    command_data->current_setpoint = current;
    uartSend(uart_tx_packet->packet.header.length, (unsigned char*)&(uart_tx_packet->raw));

}

extern packet_union_t* last_bldc_packet;
extern uint8_t last_bldc_packet_is_new;


#define MOTOR_OFFSET    500
char footContact(void) {
    int eps = 1000;
    unsigned int mot, femur;
    sensor_data_t* sensor_data = (sensor_data_t*)&(last_bldc_packet->packet.data_crc);
    mot = (unsigned int)(sensor_data->position/111);//*motPos_to_femur_crank_units); //UNITFIX
    femur = crankFromFemur(); //TODO: put into Scripts/lut.m (lib/lut.h) constant
    if ( mot-MOTOR_OFFSET>femur && (mot-MOTOR_OFFSET - eps) > femur)
    {
        LED_1 = 1;
        return 1;
    } else {
        LED_1 = 0;
        return 0;
    }
}

char footTakeoff(void) {
    int eps = 1000;
    unsigned int mot, femur;
    sensor_data_t* sensor_data = (sensor_data_t*)&(last_bldc_packet->packet.data_crc);
    mot = (unsigned int)(sensor_data->position/111);//*motPos_to_femur_crank_units); //UNITFIX
    femur = crankFromFemur();
    if ( (mot-MOTOR_OFFSET + eps) < femur){
        return 1;
    } else {
        return 0;
    }
}

long calibPos(char idx){
    long temp;
    // idx: 0 = motor, 1 = tail, 2 = femur
    if (idx == 0)
    {
    return pidObjs[1].p_state;
    }
    else if (idx == 1)
    {
    temp = (long)(encPos[0].pos << 2);       // pos 14 bits 0x0 -> 0x3fff
    return temp + (encPos[0].oticks << 16);
    }
    else if (idx == 2)
    {
    temp = -(((long)encPos[1].pos - (long)encPos[1].offset) << 2);       // pos 14 bits 0x0 -> 0x3fff
    return temp + (encPos[1].oticks << 16);
    }
    else{
        return -1;
    }
}

unsigned int crankFromFemur(void) { 
    unsigned int femur;
    femur = calibPos(2) / 64; // Scale position to 8 bits
    if(femur>255){femur=255;}
    if(femur<0){femur=0;} 
    return crank_femur_256lut[femur];
}
