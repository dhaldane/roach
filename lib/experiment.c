#include "experiment.h"
#include "as5047.h"
#include "pid-ip2.5.h"
#include "ams-enc.h"
#include "tail_ctrl.h"
#include "led.h"
#include "math.h"


#define ABS(my_val) ((my_val) < 0) ? -(my_val) : (my_val)
#define LSB2ENCRAD 0.000095873799
#define ENCRAD2LSB 10430

#define EXP_IDLE        0
#define EXP_START       1
#define EXP_JUMP_TRIG   2
#define EXP_JUMP        3
#define EXP_READY_LEG   4
#define EXP_STOP        5

volatile unsigned char  exp_state = EXP_IDLE;
volatile unsigned long t_start;



extern EncObj motPos;
extern EncObj encPos[NUM_ENC];
extern pidPos pidObjs[NUM_PIDS];
extern unsigned long t1_ticks;
extern long body_angle;


void expFlow() {
    switch(exp_state) {
        case EXP_STOP:
            if((t1_ticks-t_start) > 500){
                pidObjs[0].onoff = 0;
                pidObjs[1].onoff = 0;
                exp_state = EXP_IDLE;
            } else {
                exp_state = EXP_STOP;
            }
            break;
        case EXP_READY_LEG:
            // exp_state = EXP_READY_LEG;
            pidObjs[1].p_input = ((long)5 << 16); // Move to 10 revs forward
            setPitchSetpoint(5000000); //Full reverse!
            exp_state = EXP_STOP;
            // if(footContact() == 1){exp_state = EXP_JUMP;}
            break;
        case EXP_JUMP:
            pidObjs[1].p_input = ((long)10 << 16); // Move to 10 revs forward
            t_start = t1_ticks;
            exp_state = EXP_READY_LEG;
            break;          
        case EXP_JUMP_TRIG:
            exp_state = EXP_JUMP_TRIG;
            if(body_angle < -160000 ){exp_state = EXP_JUMP;}
            break;
        case EXP_START:
            pidObjs[0].timeFlag = 0;
            pidObjs[1].timeFlag = 0;
            resetBodyAngle();
            setPitchControlFlag(1);
            setPitchSetpoint(-160384); //10 degrees forward
            pidSetInput(0, 0);
            pidOn(0);
            pidSetInput(1, 0);
            pidObjs[1].p_input = pidObjs[1].p_state;
            pidOn(1);
            exp_state = EXP_JUMP_TRIG;
            break;
        default:
            exp_state = EXP_IDLE;
            break;

    }
}

void expStart() {
    exp_state = EXP_START;
}


char footContact(void) {
    int eps = 50;
    long mot, femur;
    mot = calibPos(0)/25;
    femur = crankFromFemur();
    if ((mot - femur) > eps)
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
    temp = (long)((encPos[1].pos-encPos[1].offset) << 2);       // pos 14 bits 0x0 -> 0x3fff
    return temp + (encPos[1].oticks << 16);
    }
    else{
        return -1;
    }
}

long crankFromFemur(void) { // TODO: Avoid float conversion
    float femur, crank;
    femur = calibPos(2) * LSB2ENCRAD;
    crank = 2.003*powf(femur,5.0) - 7.434*powf(femur,4.0) + 9.429*powf(femur,3.0) - 2.691*powf(femur,2.0) + 0.3893*femur - 0.001175;
    return (long) (crank * ENCRAD2LSB);
}
