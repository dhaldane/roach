
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

void vrTelemGetData(unsigned char* ptr) {
    
    vrTelemStruct_t* tptr;
    tptr = (vrTelemStruct_t*) ptr;

    int gdata[3];   //gyrodata
    int xldata[3];  // accelerometer data
    /////// Get XL data
    mpuGetGyro(gdata);
    mpuGetXl(xldata);

    tptr->posL = pidObjs[0].p_state;
    tptr->posR = pidObjs[1].p_state;
    tptr->composL = pidObjs[0].p_input + pidObjs[0].interpolate;
    tptr->composR = pidObjs[1].p_input + pidObjs[1].interpolate;
    tptr->dcL = pidObjs[0].output; // left
    tptr->dcR = pidObjs[1].output; // right
    tptr->bemfL = bemf[0];
    tptr->bemfR = bemf[1];

    mpuGetGyro(gdata);
    mpuGetXl(xldata);

    tptr->gyroX = gdata[0];
    tptr->gyroY = gdata[1];
    tptr->gyroZ = gdata[2];
    tptr->accelX = xldata[0];
    tptr->accelY = xldata[1];
    tptr->accelZ = xldata[2];
    tptr->Vbatt = (int) adcGetVbatt();
}

//This may be unneccesary, since the telemtry type isn't totally anonymous

unsigned int orTelemGetSize() {
    return sizeof (vrTelemStruct_t);
}