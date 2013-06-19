/*
 * Interrupt handlers. All tests are handled through timer 2 interrupt.
 * Timer 2 runs at 100Hz and checks to see if a command packet has
 * been received. If so, it pushes the appropriate test routine onto
 * a function queue to be popped and executed in the main loop.
 *
 * Created: 2011-04-16
 * Author: AMH
 */
#include "interrupts.h"
#include "consts.h"
#include "tests.h"
#include "timer.h"
#include "radio.h"
#include <stdlib.h>
#include "led.h"
#include "utils.h"
#include "cmd.h"

unsigned char* rxPacketData;
unsigned char type, status, length;

volatile unsigned int t2_ticks;

volatile PayQueue rx_pay_queue;

void __attribute__((interrupt, no_auto_psv)) _INT0Interrupt(void) {

    _INT0IF = 0;    // Clear the interrupt flag
}

void __attribute__((interrupt, no_auto_psv)) _T2Interrupt(void) {
    /*MacPacket rx_packet;
    Payload rx_payload;
    unsigned char command;

    if (!radioRxQueueEmpty())
    {
        // Check for unprocessed packet
        rx_packet = radioDequeueRxPacket();
        rx_payload = macGetPayload(rx_packet);
        command = payGetType(rx_payload);
        if(rx_packet == NULL) return;
        LED_2 = ~LED_2;
        cmdPushFunc(rx_packet);   
    }*/

    _T2IF = 0;
}

void setupTimer6(unsigned int fs) {

    unsigned int con_reg, period;

    con_reg =   T6_ON &         // Timer on
                T6_IDLE_STOP &  // Stop timer when idle
                T6_GATE_OFF &   // Gated mode off
                T6_PS_1_8 &     // Prescale 1:8
                T6_SOURCE_INT &  // Internal clock source
                T6_32BIT_MODE_OFF; // 16 bit mode

    // period value = Fcy/(prescale*Ftimer)
    period = FCY/(8*fs);

    OpenTimer6(con_reg, period);
    ConfigIntTimer6(T6_INT_PRIOR_3 & T6_INT_ON);

}

void __attribute__((interrupt, no_auto_psv)) _T6Interrupt(void) {
    //radioProcess();
    _T6IF = 0;
}
