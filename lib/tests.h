/*********************************************************************************************************
* Name: tests.h
* Desc: CrawlerProc functionality test suite 
* Date: 2011-04-16
* Author: AMH
*********************************************************************************************************/
#include "payload_queue.h"
#include "mac_packet.h"

#define RX_QUEUE_LEN    6
#define TX_QUEUE_LEN    12
#define READ_CAM_REG    1
#define WRITE_CAM_REG   2

#define FUN_Q_LEN       12

#ifndef __TESTS_H
#define __TESTS_H

extern unsigned char argument, regaddr, regvalue;
extern unsigned int i, j, rowcnt, imcnt;

// A function pointer type definition for all testing functions
// All testing functions must take 3 char args and one char array
typedef unsigned char (*test_function)(unsigned char, unsigned char,\
                             unsigned char, unsigned char*, unsigned int);

unsigned char test_radio(unsigned char type, unsigned char status,\
                         unsigned char length, unsigned char* data, unsigned int);

unsigned char test_gyro(unsigned char type, unsigned char status,\
                        unsigned char length, unsigned char* data, unsigned int);

unsigned char test_accel(unsigned char type, unsigned char status,\
                         unsigned char length, unsigned char* data, unsigned int);

unsigned char test_dflash(unsigned char type, unsigned char status,\
                         unsigned char length, unsigned char* data, unsigned int);

unsigned char test_motor(unsigned char type, unsigned char status, \
                          unsigned char length, unsigned char* data, unsigned int);

unsigned char test_sma(unsigned char type, unsigned char status, \
                          unsigned char length, unsigned char* data, unsigned int);

unsigned char test_mpu(unsigned char type, unsigned char status, \
                          unsigned char length, unsigned char* data, unsigned int);

unsigned char set_motor_direction(unsigned char chan_num, unsigned char\
                            direction);

extern unsigned char argument, regaddr, regvalue;
extern unsigned int i, j, rowcnt, imcnt;

typedef struct {
    MacPacket packet;
    test_function tf;
} Test;

#endif
