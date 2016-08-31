
// Contents of this file are copyright Andrew Pullin, 2013

//or_telem.c , OctoRoACH specific telemetry packet format




#include <xc.h>
#include "vr_telem.h"
#include "ams-enc.h"
#include "mpu6000.h"
#include "adc_pid.h"
#include "tih.h"
#include "pid-ip2.5.h"

// TODO (apullin) : Remove externs by adding getters to other modules
//extern pidObj motor_pidObjs[NUM_MOTOR_PIDS];
//extern int bemf[NUM_MOTOR_PIDS];

//externs added back in for VR telem porting (pullin 10/9/14)
extern int bemf[NUM_PIDS];
extern pidPos pidObjs[NUM_PIDS];

extern EncObj motPos;

//void vrTelemGetData(unsigned char* ptr) {
void vrTelemGetData(vrTelemStruct_t* ptr) {
    
    //vrTelemStruct_t* tptr;
    //tptr = (vrTelemStruct_t*) ptr;

    int gdata[3];   //gyrodata
    int xldata[3];  // accelerometer data
    /////// Get XL data
    mpuGetGyro(gdata);
    mpuGetXl(xldata);

    //Motion control
    ptr->posTail = (long)(encPos[0].pos << 2) + (encPos[0].oticks << 16);
    ptr->posFemur = (long)((encPos[1].pos-encPos[1].offset) << 2)+ (encPos[1].oticks << 16);
    ptr->posMotor = -(motPos.oticks << 16) - (long)(motPos.pos << 2);
    ptr->pitch = pidObjs[0].p_state;
    ptr->roll = pidObjs[2].p_state;
    ptr->yaw = pidObjs[3].p_state;
    ptr->pitchSet = pidObjs[0].p_input + pidObjs[0].interpolate;
    ptr->motorSet = pidObjs[1].p_input + pidObjs[1].interpolate;
    ptr->dcTail = pidObjs[0].output; // left
    ptr->dcBLDC = pidObjs[1].output; // right
    ptr->dcProp1 = pidObjs[2].output; // Rear
    ptr->dcProp2 = pidObjs[3].output; // Fore

    //gyro and XL
    ptr->gyroX = gdata[0];
    ptr->gyroY = gdata[1];
    ptr->gyroZ = gdata[2];
    ptr->accelX = xldata[0];
    ptr->accelY = xldata[1];
    ptr->accelZ = xldata[2];
}

//This may be unneccesary, since the telemtry type isn't totally anonymous

unsigned int orTelemGetSize() {
    return sizeof (vrTelemStruct_t);
}