/*
 * Name: tail_ctrl.c
 * Desc: Control code for pitch control of hopping robot
 * Date: 6/10/2015
 * Author: DWH
 */
#include "tail_ctrl.h"
#include "led.h"
#include "mpu6000.h"
#include "timer.h"
#include "ports.h"
#include "pid-ip2.5.h"


#define GYRO_LSB2_DEG (0.061035156) // +-2000 deg/s in 16 bits

static volatile unsigned char interrupt_count = 0;
static int interval[NUM_VELS];
long set_pt = 0;

long body_angle;

extern pidPos pidObjs[NUM_PIDS];

char pitchControlFlag;
float pitchSetpoint;


void tailCtrlSetup(){
    int i;
    for(i=0; i<NUM_VELS; i++){
        interval[i] = 2; //2 ms duration for in
    }
    body_angle = 0;
    initPIDObjPos( &(pidObjs[0]), 1800,0,0,0,0);
    SetupTimer5();
    EnableIntT5;
    pitchControlFlag = 0;
    pitchSetpoint = 0;

    pidObjs[0].timeFlag = 0;
    pidObjs[0].mode = 0;
    pidSetInput(0, 0);
    pidObjs[0].p_input = pidObjs[0].p_state;
    pidOn(0);
}

void setPitchControlFlag(char state){
    pitchControlFlag = state;
}


void setPitchSetpoint(int setpoint){
    pitchSetpoint = setpoint;
}

void resetBodyAngle(){
    body_angle = 0;
}

///////        Tail control ISR          ////////
//////  Installed to Timer5 @ 1000hz  ////////
void __attribute__((interrupt, no_auto_psv)) _T5Interrupt(void) {
    interrupt_count++;
    

    if(interrupt_count <= 5) {
        // Do signal processing on gyro
        int gdata[3];   
        mpuGetGyro(gdata);
        // body_angle += gdata[2]*GYRO_LSB2_DEG*0.001;
        body_angle += gdata[2]*0.001;

    }
    if(interrupt_count == 5) 
    {
        interrupt_count = 0;

        if(pitchControlFlag == 0){
            LED_1 = 0;
            // Control/motor off
            pidObjs[0].onoff = 0;
        } else {
            LED_1 = 1;
            // Control pitch
            pidObjs[0].onoff = 1;
            pidObjs[0].p_input = pitchSetpoint;
        }
    }

    _T5IF = 0;
}


void SetupTimer5() {
    ///// Timer 5 setup, Steering ISR, 300Hz /////
    // period value = Fcy/(prescale*Ftimer)
    unsigned int T5CON1value, T5PERvalue;
    // prescale 1:64
    T5CON1value = T5_ON & T5_IDLE_CON & T5_GATE_OFF & T5_PS_1_64 & T5_SOURCE_INT;
    T5PERvalue = 625; //1Khz
    OpenTimer5(T5CON1value, T5PERvalue);
}
