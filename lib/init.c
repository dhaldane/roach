#include "init.h"
#include "i2c.h"
#include "timer.h"
#include "uart.h"
#include "pwm.h"
#include "ports.h"
extern unsigned long t1_ticks;


void SetupADC(void)
{

    AD1CON1bits.ADON = 0;       //disable
    AD1CON1bits.ADSIDL = 0;     //continue in idle mode
    AD1CON1bits.AD12B = 0;      //10 bit mode
    AD1CON1bits.FORM = 0b00;    //integer (0000 00dd dddd dddd) format output
    AD1CON1bits.SSRC = 0b011;   //Sample clock source based on PWM
    AD1CON1bits.SIMSAM = 1;     //Sample channels simultaneously
    AD1CON1bits.ASAM = 0;       //Auto sampling off

    AD1CON2bits.VCFG = 0b000;   //Vdd is pos. ref and Vss is neg. ref.
    AD1CON2bits.CSCNA = 0;      //Do not scan inputs
    AD1CON2bits.CHPS = 0b01;    //Convert channels 0 and 1
    AD1CON2bits.SMPI = 0b0001;  //Interrupt after 2 conversions (depends on CHPS and SIMSAM)
    AD1CON2bits.BUFM = 1;       //Always fill conversion buffer from first element
    AD1CON2bits.ALTS = 0;       //Do not alternate MUXes for analog input selection

    AD1CON3bits.ADRC = 0;       //Derive conversion clock from system clock
//    AD1CON3bits.SAMC = 0b00001; //Auto sampling clock period is one Tad
    AD1CON3bits.ADCS = 0b00000010; // Each TAD is 3 Tcy

    AD1PCFGL = 0xFFF0;          //Enable AN0 - AN3 as analog inputs

    AD1CHS0bits.CH0SA = 0b00001;      //Select AN1 for CH0 +ve input
    AD1CHS0bits.CH0NA = 0b00000;      //Select Vref- for CH0 -ve input

    AD1CHS123bits.CH123SA = 0b1;  //Select AN3 for CH1 +ve input
    AD1CHS123bits.CH123NA = 0b00;  //Select Vref- for CH1 -ve input

    AD1CON1bits.ADON = 1;       //enable

//    unsigned int AD1CON1value, AD1CON2value, AD1CON3value, AD1CON4value, AD1PCFGHvalue, AD1PCFGLvalue, AD1CSSHvalue, AD1CSSLvalue, AD1CHS0value, AD1CHS123value;
//
//    AD1CON1value = ADC_MODULE_ON & ADC_IDLE_CONTINUE &
//                   ADC_AD12B_10BIT & ADC_FORMAT_INTG & ADC_CLK_MPWM &
//                   ADC_MULTIPLE & ADC_AUTO_SAMPLING_ON & ADC_SAMP_ON;
//    //AD1CON2value = ADC_VREF_AVDD_AVSS & ADC_SCAN_OFF & ADC_SELECT_CHAN_01 &
//    AD1CON2value = ADC_VREF_AVDD_AVSS & ADC_SCAN_OFF & ADC_SELECT_CHAN_0 &
//                   ADC_DMA_ADD_INC_1 & ADC_ALT_BUF_OFF & ADC_ALT_INPUT_OFF;
//    AD1CON3value = ADC_CONV_CLK_SYSTEM & ADC_CONV_CLK_3Tcy & ADC_SAMPLE_TIME_1;
//    AD1CON4value = ADC_DMA_BUF_LOC_1;
//    AD1PCFGHvalue = ENABLE_ALL_DIG_16_31;
//    AD1PCFGLvalue = ENABLE_AN0_ANA & ENABLE_AN1_ANA & ENABLE_AN2_ANA & ENABLE_AN3_ANA;
//    AD1CSSHvalue = SCAN_NONE_16_31;
//    AD1CSSLvalue = 0x000F; // Enabling AN0-3
//    //DisableIntADC1;
//    AD1CHS0value = ADC_CH0_NEG_SAMPLEA_VREFN & ADC_CH0_POS_SAMPLEA_AN1;
////    AD1CHS123value = ADC_CH123_NEG_SAMPLEA_VREFN & ADC_CH123_POS_SAMPLEA_3_4_5;
////    SetChanADC1(AD1CHS123value, AD1CHS0value);
//    SetChanADC1(0x0000, AD1CHS0value);
//    OpenADC1(AD1CON1value, AD1CON2value, AD1CON3value, AD1CON4value, AD1PCFGLvalue, AD1PCFGHvalue, AD1CSSHvalue, AD1CSSLvalue);
}

/*
void SetupCamera(void)
{
    OVCAM_PWDN = 0; // PDWN: CAM On

    delay_us(5000);

    SCCB_SetupOV7660();
}
*/

void SetupI2C(void)
{
    unsigned int I2C1CONvalue, I2C1BRGvalue;
    I2C1CONvalue = I2C1_ON & I2C1_IDLE_CON & I2C1_CLK_HLD &
                   I2C1_IPMI_DIS & I2C1_7BIT_ADD & I2C1_SLW_DIS &
                   I2C1_SM_DIS & I2C1_GCALL_DIS & I2C1_STR_DIS &
                   I2C1_NACK & I2C1_ACK_DIS & I2C1_RCV_DIS &
                   I2C1_STOP_DIS & I2C1_RESTART_DIS & I2C1_START_DIS;
    I2C1BRGvalue = 363; // Fcy(1/Fscl - 1/1111111)-1
    OpenI2C1(I2C1CONvalue, I2C1BRGvalue);
    IdleI2C1();
}

void SetupInterrupts(void)
{
    #if defined(__IMAGEPROC2)

        ConfigINT2(RISING_EDGE_INT & EXT_INT_ENABLE & EXT_INT_PRI_7); // Battery Supervisor

    #elif defined(__IMAGEPROC1)

        ConfigINT0(RISING_EDGE_INT & EXT_INT_ENABLE & EXT_INT_PRI_7); // Battery Supervisor

    #endif
    ConfigIntTimer1(T1_INT_PRIOR_4 & T1_INT_OFF);
    ConfigIntTimer2(T2_INT_PRIOR_4 & T2_INT_OFF);
}

// timer 1 is used for main pid motor control loop
void SetupTimer1(void)
{
    unsigned int T1CON1value, T1PERvalue;
    T1CON1value = T1_ON & T1_SOURCE_INT & T1_PS_1_8 & T1_GATE_OFF &
                  T1_SYNC_EXT_OFF & T1_INT_PRIOR_2;
    T1PERvalue = 0x03E8; //clock period = 0.0002s = ((T1PERvalue * prescaler)/FCY) (5000Hz)
  	t1_ticks = 0;
    OpenTimer1(T1CON1value, T1PERvalue);
}

void SetupTimer2(void)
{
    unsigned int T2CON1value, T2PERvalue;
    T2CON1value = T2_ON & T2_SOURCE_INT & T2_PS_1_8 & T2_GATE_OFF;
    T2PERvalue = 0xC350; // clock period = 0.01s = ((T1PERvalue * prescaler)/FCY) (100Hz)
    OpenTimer2(T2CON1value, T2PERvalue);
}

void SetupUART2(void)
{
    /// UART2 for RS-232 w/PC @ 230400, 8bit, No parity, 1 stop bit
    unsigned int U2MODEvalue, U2STAvalue, U2BRGvalue;
    U2MODEvalue = UART_EN & UART_IDLE_CON & UART_IrDA_DISABLE &
                  UART_MODE_FLOW & UART_UEN_10 & UART_DIS_WAKE &
                  UART_DIS_LOOPBACK & UART_DIS_ABAUD & UART_UXRX_IDLE_ONE &
                  UART_BRGH_FOUR & UART_NO_PAR_8BIT & UART_1STOPBIT;
    U2STAvalue  = UART_INT_TX & UART_INT_RX_CHAR &UART_SYNC_BREAK_DISABLED &
                  UART_TX_ENABLE & UART_ADR_DETECT_DIS &
                  UART_IrDA_POL_INV_ZERO; // If not, whole output inverted.
    U2BRGvalue  = 43; // (Fcy / ({16|4} * baudrate)) - 1
    OpenUART2(U2MODEvalue, U2STAvalue, U2BRGvalue);
}

unsigned int PTPERvalue=4999;

void SetupPWM(void){

   LATB  = 0x0000;
   TRISB = 0b0000011011111011;

   PORTBbits.RB8 = 0;
   PORTBbits.RB11 = 0;

   unsigned int SEVTCMPvalue, PTCONvalue, PWMCON1value, PWMCON2value;
   SEVTCMPvalue = 160; //Special event compare register triggers special event
   //SEVTCMPvalue = 4839; //Special event compare register triggers special event
                        //at this point in the PWM period. Useful for sampling
                        //back EMF when motor is turned off, for example.
                        //Values larger than 4839 for a time base of 4999 give
                        //unreliable results. Consequently, running the motor
                        //at greater than 96.7% duty cycle means you cannot
                        //sense back EMF.

   PTCONvalue = PWM_EN & PWM_IDLE_CON & PWM_OP_SCALE1 &
//                PWM_IPCLK_SCALE4 & PWM_MOD_FREE;
                PWM_IPCLK_SCALE16 & PWM_MOD_FREE;
   PWMCON1value = PWM_MOD1_IND & PWM_PEN1L & PWM_PEN1H &
                  PWM_MOD2_IND & PWM_PEN2L & PWM_PEN2H &
                  PWM_MOD3_IND & PWM_PEN3L & PWM_PEN3H;
   PWMCON2value = PWM_SEVOPS1 & PWM_OSYNC_TCY & PWM_UEN;
   ConfigIntMCPWM(PWM_INT_DIS & PWM_FLTA_DIS_INT & PWM_FLTB_DIS_INT);

   OpenMCPWM(PTPERvalue, SEVTCMPvalue, PTCONvalue, PWMCON1value, PWMCON2value);
   P1OVDCONbits.POVD1L = 1;
   P1OVDCONbits.POVD1H = 1;
   P1OVDCONbits.POVD2L = 1;
   P1OVDCONbits.POVD2H = 1;
   P1OVDCONbits.POVD3L = 1;
   P1OVDCONbits.POVD3H = 1;
   SetDCMCPWM(1, 0, 0);
   SetDCMCPWM(2, 0, 0);
   SetDCMCPWM(3, 0, 0);
}
