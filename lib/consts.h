#ifndef __CONSTS_H
#define __CONSTS_H

#define ADC_MAX             1023
#define M_PI                3.14159265

//Limits for numerical values of different types
#define INT_MIN             -32768
#define INT_MAX             32767
#define UINT_MAX            65535
#define UINT_MIN            0

//Status field identifiers (currently unused)
#define STATUS_UNUSED       0

//Command identifiers
#define RADIO_TEST          0
#define GYRO_TEST           1
#define ACCEL_TEST          2
#define DFLASH_TEST         3
#define MOTOR_TEST          4
#define SMA_TEST            5
#define MPU_TEST            6

//Camera constants
#define IMCOLS 160
#define IMROWS 100

//Communication constants
#define TX_PAYLOAD_SIZE     128

//Timer constants
#define FT1                 100

#define ON                  1
#define OFF                 0

#define REVERSE             0
#define FORWARD             1
#define BRAKE               2

//Test configuration parameters
#define TEST_PACKET_INTERVAL_MS 200
#define MOTOR_TEST_DUTY_CYCLE   50
#define MOTOR_TEST_DURATION     10 //How long to run the motor (in seconds) during the test
#define MD_LED_1                _LATB8
#define MD_LED_2                _LATB11
#define SMA_1                   1
#define SMA_2                   2


//Motor PWM configuration parameters
//#define PTPER_VALUE             4999 // = FCY / (F_pwm * time_base_input_prescaler) - 1 eg. 2kHz pwm = 40e6 / ((((PTPER_VALUE = 4999) + 1)) * 4X prescaler)
#define PTPER_VALUE             624// = FCY / (F_pwm * time_base_input_prescaler) - 1 eg. 2kHz pwm = 40e6 / ((((PTPER_VALUE = 4999) + 1)) * 4X prescaler)
#endif
