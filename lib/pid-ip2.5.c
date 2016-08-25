/*
 * Name: UpdatePID.c
 * Desc: Control code to compute the new input to the plant
 * Date: 2009-04-03
 * Author: AMH
   modified to include hall effect sensor by RSF.
 * modified Dec. 2011 to include telemetry capture
 * modified Jan. 2012 to include median filter on back emf
 * modified Jan. 2013 to include AMS Hall encoder, and MPU 6000 gyro
 */
#include "pid-ip2.5.h"
#include "dfmem.h"
#include "timer.h"
#include "adc_pid.h"
#include "pwm.h"
#include "led.h"
#include "adc.h"
#include "p33Fxxxx.h"
#include "sclock.h"
#include "ams-enc.h"
#include "tih.h"
#include "mpu6000.h"
#include "uart_driver.h"
#include "ppool.h"
#include "dfmem.h"
#include "telem.h"

#include <stdlib.h> // for malloc
#include "init.h"  // for Timer1

#include "as5047.h"
#include "tail_ctrl.h"


#define MC_CHANNEL_PWM1     1
#define MC_CHANNEL_PWM2     2
#define MC_CHANNEL_PWM3     3
#define MC_CHANNEL_PWM4     4

//#define HALFTHROT 10000
#define HALFTHROT 2000
#define FULLTHROT 2*HALFTHROT
// MAXTHROT has to allow enough time at end of PWM for back emf measurement
// was 3976
#define MAXTHROT 3800

#define ABS(my_val) ((my_val) < 0) ? -(my_val) : (my_val)

// PID control structure
pidPos pidObjs[NUM_PIDS];

// structure for reference velocity for leg
pidVelLUT  pidVel[NUM_PIDS*NUM_BUFF];
pidVelLUT* activePID[NUM_PIDS];     //Pointer arrays for stride buffering
pidVelLUT* nextPID[NUM_PIDS];

#define T1_MAX 0xffffff  // max before rollover of 1 ms counter
// may be glitch in longer missions at rollover
volatile unsigned long t1_ticks;
unsigned long lastMoveTime;
int seqIndex;

//for battery voltage:
char calib_flag = 0;   // flag is set if doing calibration
long offsetAccumulatorL, offsetAccumulatorR;
unsigned int offsetAccumulatorCounter;



// 2 last readings for median filter
int measLast1[NUM_PIDS];
int measLast2[NUM_PIDS];
int bemf[NUM_PIDS];


// -------------------------------------------
// called from main()
void pidSetup()
{
	int i;
	for(i = 0; i < NUM_PIDS; i++){
		initPIDObjPos( &(pidObjs[i]), DEFAULT_KP, DEFAULT_KI, DEFAULT_KD, DEFAULT_KAW, DEFAULT_FF); 
	}
	initPIDVelProfile();
	SetupTimer1();  // main interrupt used for leg motor PID

	lastMoveTime = 0;
//  initialize PID structures before starting Timer1
	pidSetInput(0,0);
	pidSetInput(1,0);
	
	EnableIntT1; // turn on pid interrupts

	// calibBatteryOffset(100); //???This is broken for 2.5
}


//Returns pointer to non-active buffer
pidVelLUT* otherBuff(pidVelLUT* array, pidVelLUT* ptr){
    if( ptr >= &(array[NUM_PIDS])){
        return ptr - NUM_PIDS;
    } else {
        return ptr + NUM_PIDS;
    }
}

// ----------   all the initializations  -------------------------
// set expire time for first segment in pidSetInput - use start time from MoveClosedLoop
// set points and velocities for one revolution of leg
// called from pidSetup()
void initPIDVelProfile(){
    int i,j;
    pidVelLUT* tempPID;
    for(j = 0; j < NUM_PIDS; j++){
        pidObjs[j].index = 0;  // point to first velocity
        pidObjs[j].interpolate = 0; 
        pidObjs[j].leg_stride = 0;  // set initial leg count
        activePID[j] = &(pidVel[j]);    //Initialize buffer pointers
        nextPID[j] = NULL;
        tempPID = otherBuff(pidVel, activePID[j]);
        for(i = 0; i < NUM_VELS; i++){
            tempPID->interval[i]= 100;
            tempPID->delta[i]= 0;
            tempPID->vel[i]= 0;   
        }
        tempPID->onceFlag = 0;
        nextPID[j] = tempPID;
        pidObjs[j].p_input = 0; // initialize first set point 
        pidObjs[j].v_input = (int)(((long) pidVel[j].vel[0] * K_EMF) >>8);  //initialize first velocity, scaled
    }
}


// called from cmd.c
void setPIDVelProfile(int pid_num, int *interval, int *delta, int *vel, int onceFlag){
    pidVelLUT* tempPID;
    int i;
    tempPID = otherBuff(pidVel, activePID[pid_num]);
    for (i = 0; i < NUM_VELS; i++)
    {
        tempPID->interval[i]= interval[i];
        tempPID->delta[i]= delta[i];
        tempPID->vel[i]= vel[i];
    }
    tempPID->onceFlag = onceFlag;
    if (activePID[pid_num]->onceFlag == 0){
        nextPID[pid_num] = tempPID;
    }
}


// called from pidSetup()
void initPIDObjPos(pidPos *pid, int Kp, int Ki, int Kd, int Kaw, int ff)
{
    pid->p_input = 0;
    pid->v_input = 0;
    pid->p = 0;
    pid->i = 0;
    pid->d = 0;
    pid->Kp = Kp;
    pid->Ki= Ki;
    pid->Kd = Kd;
    pid->Kaw = Kaw; 
	pid->feedforward = 0;
    pid->output = 0;
    pid->onoff = 0;
	pid->p_error = 0;
	pid->v_error = 0;
	pid->i_error = 0;
}



// called from set thrust closed loop, etc. Thrust 
void pidSetInput(int pid_num, int input_val){
unsigned long temp;	
/*      ******   use velocity setpoint + throttle for compatibility between Hall and Pullin code *****/
/* otherwise, miss first velocity set point */
    pidObjs[pid_num].v_input = input_val + (int)(( (long)pidVel[pid_num].vel[0] * K_EMF) >> 8);	//initialize first velocity ;
    pidObjs[pid_num].start_time = t1_ticks;
    //zero out running PID values
    pidObjs[pid_num].i_error = 0;
    pidObjs[pid_num].p = 0;
    pidObjs[pid_num].i = 0;
    pidObjs[pid_num].d = 0;
	//Seed the median filter
	measLast1[pid_num] =input_val;
	measLast2[pid_num] =input_val;

// set initial time for next move set point 
/*   need to set index =0 initial values */
/* position setpoints start at 0 (index=0), then interpolate until setpoint 1 (index =1), etc */
	temp = 0;
	pidObjs[pid_num].expire = temp + (long) pidVel[pid_num].interval[0];   // end of first interval
	pidObjs[pid_num].interpolate = 0;	
/*	pidObjs[pid_num].p_input += pidVel[pid_num].delta[0];	//update to first set point
***  this should be set only after first .expire time to avoid initial transients */
	pidObjs[pid_num].index =0; // reset setpoint index
// set first move at t = 0
//	pidVel[0].expire = temp;   // right side
//	pidVel[1].expire = temp;   // left side

}

void pidStartTimedTrial(unsigned int run_time){
    unsigned long temp;
    int i;
    temp = t1_ticks;  // need atomic read due to interrupt  
    for(i=0;i<NUM_PIDS;i++){
        pidObjs[i].run_time = run_time;
        pidObjs[i].start_time = temp;       
    }
    if ((temp + (unsigned long) run_time) > lastMoveTime)
    { lastMoveTime = temp + (unsigned long) run_time; }  // set run time to max requested time
}

// from cmd.c  PID set gains
void pidSetGains(int pid_num, int Kp, int Ki, int Kd, int Kaw, int ff){
    pidObjs[pid_num].Kp  = Kp;
    pidObjs[pid_num].Ki  = Ki;
    pidObjs[pid_num].Kd  = Kd;
    pidObjs[pid_num].Kaw = Kaw;
	pidObjs[pid_num].feedforward = ff;
}

void pidOn(int pid_num){
	pidObjs[pid_num].onoff = 1;
	t1_ticks = 0;
}

// zero position setpoint for both motors (avoids big offset errors)
void pidZeroPos(int pid_num){ 
// disable interrupts to reset state variables
	DisableIntT1; // turn off pid interrupts
	amsEncoderResetPos(); //  reinitialize rev count and relative zero encoder position for both motors
	pidObjs[pid_num].p_state = 0;
// reset position setpoint as well
	pidObjs[pid_num].p_input = 0;
	pidObjs[pid_num].v_input = 0;
	pidObjs[pid_num].leg_stride = 0; // strides also reset 
	EnableIntT1; // turn on pid interrupts
}


// calibrate A/D offset, using PWM synchronized A/D reads inside 
// timer 1 interrupt loop
// BATTERY CHANGED FOR IP2.5 ***** need to fix
void calibBatteryOffset(int spindown_ms){
	long temp;  // could be + or -
	unsigned int battery_voltage;
// save current PWM config
	int tempPDC1 = PDC1;
	int tempPDC2 = PDC2;
	PDC1 = 0; PDC2 = 0;  /* SFR for PWM? */

// save current PID status, and turn off PID control
	short tempPidObjsOnOff[NUM_PIDS];
	tempPidObjsOnOff[0] = pidObjs[0].onoff;
	tempPidObjsOnOff[1] = pidObjs[1].onoff;
	pidObjs[0].onoff = 0; pidObjs[1].onoff = 0;

	delay_ms(spindown_ms); //motor spin-down
	LED_RED = 1;
	offsetAccumulatorL = 0;
	offsetAccumulatorR = 0; 
	offsetAccumulatorCounter = 0; // updated inside servo loop
	calib_flag = 1;  // enable calibration
	while(offsetAccumulatorCounter < 100); // wait for 100 samples
	calib_flag = 0;  // turn off calibration
	battery_voltage = adcGetVbatt();
	//Left
	temp = offsetAccumulatorL;
	temp = temp/(long)offsetAccumulatorCounter;
	// pidObjs[0].inputOffset = (int) temp;

	//Right
	temp = offsetAccumulatorR;
	temp = temp/(long)offsetAccumulatorCounter;
	// pidObjs[1].inputOffset = (int) temp;

	LED_RED = 0;
// restore PID values
	PDC1 = tempPDC1;
	PDC2 = tempPDC2;
	pidObjs[0].onoff = tempPidObjsOnOff[0];
	pidObjs[1].onoff = tempPidObjsOnOff[1];
}


/*****************************************************************************************/
/*****************************************************************************************/
/*********************** Stop Motor and Interrupts *********************************************/
/*****************************************************************************************/
/*****************************************************************************************/
void EmergencyStop(void)
{
	pidSetInput(0, 0);
	pidSetInput(1, 0);
	DisableIntT1; // turn off pid interrupts
       SetDCMCPWM(MC_CHANNEL_PWM1, 0, 0);    // set PWM to zero
       SetDCMCPWM(MC_CHANNEL_PWM2, 0, 0); 
}
	

// -------------------------   control  loop section  -------------------------------

/*********************** Motor Control Interrupt *********************************************/
/*****************************************************************************************/
/*****************************************************************************************/

/* update setpoint  only leg which has run_time + start_time > t1_ticks */
/* turn off when all PIDs have finished */
static volatile unsigned char interrupt_count = 0;
static volatile unsigned char telemetry_count = 0;
extern volatile MacPacket uart_tx_packet;
extern volatile unsigned char uart_tx_flag;

void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void) {
    int j,i;
    LED_3 = 1;
    interrupt_count++;

    //Telemetry save, at 1Khz
    //TODO: Break coupling between PID module and telemetry triggering
    if(interrupt_count == 3) {
        telemSaveNow();
    }
    //Update IMU
    //TODO: Break coupling between PID module and IMU update
    if(interrupt_count == 4) {
        mpuBeginUpdate();
        amsEncoderStartAsyncRead();
        // as5047EncoderUpdatePos();
    }
    //PID controller update
    else if(interrupt_count == 5)
    {
        interrupt_count = 0;

        if (t1_ticks == T1_MAX) t1_ticks = 0;
        t1_ticks++;
        pidGetState();	// always update state, even if motor is coasting
        for (j = 0; j< NUM_PIDS; j++) {
        // only update tracking setpoint if time has not yet expired
            if (pidObjs[j].onoff) {
                if (pidObjs[j].timeFlag){
                    if (pidObjs[j].start_time + pidObjs[j].run_time >= t1_ticks){
                        pidGetSetpoint(j);
                    }
                    if(t1_ticks > lastMoveTime){ // turn off if done running all legs
                        for(i=0;i<NUM_PIDS;i++){
                        pidObjs[i].onoff = 0;
                    }
                    } 
                } 
                else {                 
                    pidGetSetpoint(j);
                }
            }
        }
        if (pidObjs[0].mode == 0){
            pidSetControl();
        } else if (pidObjs[0].mode == 1)
        {
            tiHSetDC(1, pidObjs[0].pwmDes);
            tiHSetDC(2, pidObjs[1].pwmDes);
        }
    }
    LED_3 = 0;
    _T1IF = 0;
}

// update desired velocity and position tracking setpoints for each leg
void pidGetSetpoint(int j){
    int index; 
    index = pidObjs[j].index;		
	// update desired position between setpoints, scaled by 256
	pidObjs[j].interpolate += (long)activePID[j]->vel[index];

	if (t1_ticks >= pidObjs[j].expire){ // time to reach previous setpoint has passed
		pidObjs[j].interpolate = 0;	
		pidObjs[j].p_input += activePID[j]->delta[index];	//update to next set point
        pidObjs[j].index++;
            
        if (pidObjs[j].index >= NUM_VELS) {
             pidObjs[j].index = 0;
             pidObjs[j].leg_stride++;  // one full leg revolution
    /**** maybe need to handle round off in position set point ***/
             checkSwapBuff(j);
        }  
		pidObjs[j].expire += activePID[j]->interval[pidObjs[j].index];  // expire time for next interval
        pidObjs[j].v_input = (activePID[j]->vel[pidObjs[j].index]);    //update to next velocity 
	}
}   

void checkSwapBuff(int j){
    if(nextPID[j] != NULL){    //Swap pointer if not null
       if(nextPID[j]->onceFlag == 1){
           pidVelLUT* tempPID;
           CRITICAL_SECTION_START;
           tempPID = activePID[j];
           activePID[j] = nextPID[j];
           nextPID[j] = tempPID;
           CRITICAL_SECTION_END;
       } else {
       CRITICAL_SECTION_START;
       activePID[j] = nextPID[j];
       nextPID[j] = NULL;
       CRITICAL_SECTION_END;
       }
    }
}

 // select either back emf or backwd diff for vel est

#define VEL_BEMF 0

/* update state variables including motor position and velocity */
extern long body_angle[3];
extern EncObj motPos;
extern EncObj encPos;
long oldTailPos;

void pidGetState()
{   int i;
    long p_state, tail_pos;
    long oldpos[NUM_PIDS], velocity;
    
    for(i=0; i<NUM_PIDS; i++)
    { oldpos[i] = pidObjs[i].p_state; }
    
    p_state = (long)(motPos.pos << 2);		// pos 14 bits 0x0 -> 0x3fff
    p_state = p_state + (motPos.oticks << 16);
    p_state = p_state - (long)(motPos.offset <<2); 	// subtract offset to get zero position
    pidObjs[0].p_state = body_angle[2];
    pidObjs[2].p_state = body_angle[1];
    pidObjs[3].p_state = body_angle[0];
    pidObjs[1].p_state = -p_state;

    velocity = (pidObjs[1].p_state - oldpos[1]) / 64;  // Encoder ticks per ms
    if (velocity > 0x7fff) velocity = 0x7fff; // saturate to int
    if(velocity < -0x7fff) velocity = -0x7fff;	
    pidObjs[1].v_state = (int) velocity;

    tail_pos = (long)(encPos[0].pos << 2) + (encPos[0].oticks << 16);

    velocity = (tail_pos - oldTailPos) / 64;  // Encoder ticks per ms
    if (velocity > 0x7fff) velocity = 0x7fff; // saturate to int
    if(velocity < -0x7fff) velocity = -0x7fff;  
    pidObjs[0].extraVel = (int) velocity;
    oldTailPos = tail_pos;
    
    int gdata[3];   
    mpuGetGyro(gdata);
    pidObjs[0].v_state = gdata[2];
    pidObjs[2].v_state = gdata[1];
    pidObjs[3].v_state = gdata[0];
}


long min_pos = -3000;
long max_pos = 1114112;

void pidSetControl()
{ int i,j;
// 0 = right side
    for(j=0; j < NUM_PIDS; j++)
   {  //pidobjs[0] : right side
	// p_input has scaled velocity interpolation to make smoother
	// p_state is [16].[16]
        	pidObjs[j].p_error = pidObjs[j].p_input + pidObjs[j].interpolate  - pidObjs[j].p_state;
            pidObjs[j].v_error = pidObjs[j].v_input - pidObjs[j].v_state;  // v_input should be revs/sec
            //Update values
            UpdatePID(&(pidObjs[j]),j);
       } // end of for(j)
   for(i=0;i<NUM_PIDS;i++){
        if(pidObjs[i].onoff) {tiHSetDC(i+1, pidObjs[i].output); }
        else {tiHSetDC(i+1,0);} // turn off motor if PID loop is off
    }
}

void UpdatePID(pidPos *pid, int num)
{
    if(num < 3){
        pid->p = ((long)pid->Kp * pid->p_error) >> 12 ;  // scale so doesn't over flow
        pid->i = (long)pid->Ki  * pid->i_error  >> 12 ;
        pid->d=  (long)pid->Kd *  (long) pid->v_error;
        // better check scale factors

        pid->preSat = pid->feedforward + pid->p +
    		 ((pid->i ) >> 4) +  // divide by 16
    		  (pid->d >> 4); // divide by 16
    	pid->output = pid->preSat;
 
    /* i_error say up to 1 rev error 0x10000, X 256 ms would be 0x1 00 00 00  
        scale p_error by 16, so get 12 bit angle value*/
    	pid-> i_error = (long)pid-> i_error + ((long)pid->p_error >> 4); // integrate error
    // Mixing for Thruster control
    if (num == 2)
    {
        pidPos *yaw = &(pidObjs[3]);
        yaw->p = ((long)yaw->Kp * yaw->p_error) >> 12 ;  // scale so doesn't over flow
        yaw->i = (long)yaw->Ki  * yaw->i_error  >> 12 ;
        yaw->d=  (long)yaw->Kd *  (long) yaw->v_error;
        // better check scale factors

        yaw->preSat = yaw->feedforward + yaw->p +
             ((yaw->i ) >> 4) +  // divide by 16
              (yaw->d >> 4); // divide by 16
        yaw->output = yaw->preSat;
        int temp_roll, temp_yaw;
        temp_roll = -pid->output;
        temp_yaw = yaw->output;
        pid->output = (2*temp_roll + temp_yaw)/2;
        yaw->output = (2*temp_roll - temp_yaw)/2;
        // TODO: Saturate correctly for control effort in yaw/roll
        if (yaw->output < -MAXTHROT){yaw->output = -MAXTHROT;} 
        if (pid->output < -MAXTHROT){pid->output = -MAXTHROT;} 
        if (yaw->output > MAXTHROT){yaw->output = MAXTHROT;} 
        if (pid->output > MAXTHROT){pid->output = MAXTHROT;} 

    }
// saturate output - assume only worry about >0 for now
// apply anti-windup to integrator  
    if(num==0){
        pid->preSat = -pid->preSat;
        pid->output = -pid->output;
    	if (pid->preSat > MAXTHROT) 
    	{ 	      pid->output = MAXTHROT; 
    			pid->i_error = (long) pid->i_error + 
    				(long)(pid->Kaw) * ((long)(MAXTHROT) - (long)(pid->preSat)) 
    				/ ((long)GAIN_SCALER);		
    	}      
    	if (pid->preSat < -MAXTHROT)
          { 	      pid->output = -MAXTHROT; 
    			pid->i_error = (long) pid->i_error + 
    				(long)(pid->Kaw) * ((long)(MAXTHROT) - (long)(pid->preSat)) 
    				/ ((long)GAIN_SCALER);		
    	}
    } else if(num==1){ // BLDC motor
        pid->preSat = pid->feedforward + (pid->p ) +
             ((pid->i ) >> 6) +  // divide by 64
             (pid->d >> 12); // divide by 64
        pid->output = pid->preSat;
        int max = getMotMax();
        int min = getMotMin();
        long pos = pidObjs[1].p_state;
        if(pos<min_pos && pid->preSat < 0){
            pid->output = 0; LED_1=1;
        }
        if(pos>max_pos && pid->preSat > 0){
            pid->output = 0; LED_2 = 1;
        }
        if (pid->preSat > max) 
        {       pid->output = max; 
                pid->i_error = (long) pid->i_error + 
                    (long)(pid->Kaw) * ((long)(max) - (long)(pid->preSat)) 
                    / ((long)GAIN_SCALER);      
        }      
        if (pid->preSat < min)
          {     pid->output = min; 
                pid->i_error = (long) pid->i_error + 
                    (long)(pid->Kaw) * ((long)(min) - (long)(pid->preSat)) 
                    / ((long)GAIN_SCALER);      
        }
    }
    }   
}


