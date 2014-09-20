/*********************************************************************************************************
* Name: main.c
* Desc: A test suite for the ImageProc 2.2 system. These tests should not be
* considered rigorous, exhaustive tests of the hardware, but rather
* "smoke tests" - ie. turn on the functionality and make sure the 
* hardware/software doesn't start "smoking."
*
* The architecture is based on a function pointer queue scheduling model. The
* meat of the testing logic resides in test.c. If the radio has received a 
* command packet during the previous timer interval for Timer2, the appropriate
* function pointer is added to a queue in the interrupt service routine for 
* Timer2 (interrupts.c). The main loop simply pops the function pointer off
* the top of the queue and executes it. 
*
* Date: 2011-04-13
* Author: AMH, Ryan Julian
*********************************************************************************************************/
#include "p33Fxxxx.h"
#include "init.h"
#include "init_default.h"
#include "timer.h"
#include "utils.h"
#include "radio.h"
#include "tih.h"
#include "ams-enc.h"
#include "settings.h"
#include "tests.h" // TODO (fgb) : define/includes need to live elsewhere
#include "dfmem.h"
#include "telem.h"
#include "interrupts.h"
#include "mpu6000.h"
#include "sclock.h"
#include "spi_controller.h"
#include "interrupts.h"
#include "pid-ip2.5.h"
#include "adc_pid.h"
#include "cmd.h"
#include "uart_driver.h"
#include "ppool.h"
#include "carray.h"

#include <stdlib.h>

static Payload rx_payload;
static MacPacket rx_packet;
static test_function rx_function;

volatile MacPacket uart_tx_packet;
volatile unsigned char uart_tx_flag;

volatile CircArray fun_queue;

int main() {

    // Processor Initialization
    SetupClock();
    SwitchClocks();
    SetupPorts();
    sclockSetup();

    LED_1 = 1;
    LED_2 = 1;
    LED_3 = 1;

    // Message Passing
    fun_queue = carrayCreate(FUN_Q_LEN);
    cmdSetup();

    // Radio setup
    radioInit(RADIO_RXPQ_MAX_SIZE, RADIO_TXPQ_MAX_SIZE);
    radioSetChannel(RADIO_CHANNEL);
    radioSetSrcAddr(RADIO_SRC_ADDR);
    radioSetSrcPanID(RADIO_PAN_ID);

    uart_tx_packet = NULL;
    uart_tx_flag = 0;
    uartInit(&cmdPushFunc);

    // Need delay for encoders to be ready
    delay_ms(100);
    amsEncoderSetup();
    mpuSetup();
    tiHSetup();
    dfmemSetup();
    telemSetup();
    adcSetup();
    pidSetup();



    LED_1 = 0;
    LED_3 = 1;
    while(1){
        // Send outgoing radio packets
        radioProcess();

        // Send outgoing uart packets
        if(uart_tx_flag) {
            uartSendPacket(uart_tx_packet);
            uart_tx_flag = 0;
        }


        // move received packets to function queue
        while (!radioRxQueueEmpty()) {
            // Check for unprocessed packet
            rx_packet = radioDequeueRxPacket();
            if(rx_packet != NULL) {
                cmdPushFunc(rx_packet);
            }
        }

        // process commands from function queue
        while(!carrayIsEmpty(fun_queue)) {
            rx_packet = carrayPopHead(fun_queue);
            if(rx_packet != NULL) {
               rx_payload = macGetPayload(rx_packet);
               if(rx_payload != NULL) {
                   rx_function = (test_function)(rx_payload->test);
                   if(rx_function != NULL) {
                       LED_2 = ~LED_2;
                       (rx_function)(payGetType(rx_payload), payGetStatus(rx_payload), payGetDataLength(rx_payload), payGetData(rx_payload));
                   }
               }
               ppoolReturnFullPacket(rx_packet);
            }
        }
    }
    return 0;
}
