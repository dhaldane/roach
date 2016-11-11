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
#define EXP_WJ_READY_LEG    4
#define EXP_WJ_STOP         5

#define EXP_SJ_START        6
#define EXP_SJ_STOP         7

volatile unsigned char  exp_state = EXP_IDLE;
volatile unsigned long t_start;

packet_union_t uart_tx_packet_global;

#define MOT_FLAG_STOP 0x80

extern EncObj motPos;
extern EncObj encPos[NUM_ENC];
extern pidPos pidObjs[NUM_PIDS];
extern unsigned long t1_ticks;
extern long body_angle[3];


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
            // exp_state = EXP_WJ_READY_LEG;
            exp_state = EXP_WJ_READY_LEG;
            if(footContact() == 1 || gdata[2] < -6000){
                send_command_packet(&uart_tx_packet_global, 4941297, 0, 2);
                setPitchSetpoint(900000);
                exp_state = EXP_WJ_STOP;
            }
            break;
        case EXP_WJ_JUMP:
            send_command_packet(&uart_tx_packet_global, 4941297, 0, 2); // Move to 12 revs forward
            exp_state = EXP_WJ_JUMP;
            if((t1_ticks-t_start) > 100){
                setPitchSetpoint(1000000);
                send_command_packet(&uart_tx_packet_global, 411774, 0, 2);
            }
            // if((t1_ticks-t_start) > 200){exp_state = EXP_WJ_READY_LEG;}
            if((t1_ticks-t_start) > 280){exp_state = EXP_WJ_READY_LEG;}
            break;          
        case EXP_WJ_JUMP_TRIG:
            exp_state = EXP_WJ_JUMP_TRIG;
            if(body_angle[2] < -175000 ){
                exp_state = EXP_WJ_JUMP; 
                t_start = t1_ticks;
            }
            break;
        case EXP_WJ_START:
            pidObjs[0].timeFlag = 0;
            resetBodyAngle();
            setPitchControlFlag(1);
            setPitchSetpoint(-551287); //25 degrees forward
            pidSetInput(0, 0);
            pidOn(0);
            send_command_packet(&uart_tx_packet_global, 0, 0, 0); // send command packet

            exp_state = EXP_WJ_JUMP_TRIG;
            break;
// Single Jump
        case EXP_SJ_STOP:
            if((t1_ticks-t_start) > 300){
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
            send_command_packet(&uart_tx_packet_global, 6173491, 0, 2); // send command packet
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

        default:
            exp_state = EXP_IDLE;
            break;

    }
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

char footContact(void) {
    int eps = 3000;
    unsigned int mot, femur;
    sensor_data_t* sensor_data = (sensor_data_t*)&(last_bldc_packet->packet.data_crc);
    mot = (unsigned int)(sensor_data->position*motPos_to_femur_crank_units); //UNITFIX
    femur = crankFromFemur();
    if ( mot+800>femur && (mot+800 - femur) > eps)
    {
        LED_1 = 1;
        return 1;
    } else {
        LED_1 = 0;
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
    temp = (long)((encPos[1].pos) << 2)- (long)(encPos[1].offset << 2);       // pos 14 bits 0x0 -> 0x3fff
    return temp + (encPos[1].oticks << 16);
    }
    else{
        return -1;
    }
}

unsigned int crankFromFemur(void) { 
    unsigned int femur;
    femur = calibPos(2) / 256; // Scale position to 8 bits
    if(femur>255){femur=255;}
    if(femur<0){femur=0;} 
    return crank_femur_256lut[femur];
}
