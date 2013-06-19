#ifndef __INTERRUPTS_H
#define __INTERRUPTS_H

#define TX_QUEUE_SIZE   128
#define RX_QUEUE_SIZE   128

#define FCY                         (40000000)  // 40 MIPS  
#define RADIO_FCY                   (200)       // 200 Hz

void __attribute__((interrupt, no_auto_psv)) _INT0Interrupt(void);
void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void);
void __attribute__((interrupt, no_auto_psv)) _T2Interrupt(void);
void __attribute__((interrupt, no_auto_psv)) _T6Interrupt(void);

void setupTimer6(unsigned int fs);

extern volatile unsigned int t1_ticks;
extern unsigned char t1_cycles;
extern volatile unsigned int pwm_period_ticks;
extern volatile unsigned int pwm_periods_per_cycle;

#endif
