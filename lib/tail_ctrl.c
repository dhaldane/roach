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
#define hall_sense PORTBbits.RB3

static volatile unsigned char interrupt_count = 0;
static volatile unsigned char jump_flag = 0;
static float body_angle = 0;
static float body_angle_setpoint = 0;
static int interval[NUM_VELS];
static long mid_pt = 0;
static long uppr_bnd = 28000;
static long lower_bnd = -28000;

extern pidPos pidObjs[NUM_PIDS];


void tailCtrlSetup(){
    int i;
    for(i=0; i<NUM_VELS; i++){
        interval[i] = 2; //2 ms duration for in
    }
    initPIDObjPos( &(pidObjs[0]), 1800,200,100,0,0);
    SetupTimer5();
    EnableIntT5;

    pidObjs[0].timeFlag = 0;
    pidObjs[0].mode = 0;
    pidSetInput(0, 0);
    pidObjs[0].p_input = pidObjs[0].p_state;
    pidOn(0);
    pidOn(1);

}

///////        Tail control ISR          ////////
//////  Installed to Timer5 @ 1000hz  ////////
void __attribute__((interrupt, no_auto_psv)) _T5Interrupt(void) {
    interrupt_count++;
    

    if(interrupt_count <= 5) {
        // Do signal processing on gyro
        int gdata[3];   
        mpuGetGyro(gdata);
        body_angle += gdata[2]*GYRO_LSB2_DEG*0.001;

        if (hall_sense == 1){
            jump_flag = 1;
            body_angle_setpoint = body_angle;
        }
    }
    if(interrupt_count == 5) 
    {
        interrupt_count = 0;
        // Update control parameters
        int delta[NUM_VELS], vel[NUM_VELS];

        if(hall_sense == 1){
            LED_1 = 1;
            // Control to set position
            pidObjs[0].p_input = mid_pt;
        } else {
            LED_1 = 0;
            // Control pitch
            long c_pos;
            c_pos = 333.0*(body_angle - body_angle_setpoint) - mid_pt;
            if(c_pos > uppr_bnd) {c_pos = uppr_bnd;}
            if(c_pos < lower_bnd) {c_pos = lower_bnd;}
            pidObjs[0].p_input = c_pos;


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
