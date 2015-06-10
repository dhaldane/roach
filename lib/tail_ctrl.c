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


#define GYRO_LSB2_DEG (0.061035156) // +-2000 deg/s in 16 bits

static volatile unsigned char interrupt_count = 0;
static float body_angle = 0;

void tailCtrlSetup(){
    SetupTimer5();
    EnableIntT5;

}

///////        Telemtry ISR          ////////
//////  Installed to Timer5 @ 1000hz  ////////
void __attribute__((interrupt, no_auto_psv)) _T5Interrupt(void) {
    interrupt_count++;

    if(interrupt_count <= 5) {
        // Do signal processing on gyro
        int gdata[3];   //gyrodata
        mpuGetGyro(gdata);
        body_angle += gdata[2]*GYRO_LSB2_DEG*0.001;
    }
    if(interrupt_count == 5) 
    {
        interrupt_count = 0;
        // Update control parameters
        if(body_angle > 90.0){
            LED_1 = 1;
        } else{
            LED_1 = 0;
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
