/*
 * Copyright (c) 2012-2013, Regents of the University of California
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * - Neither the name of the University of California, Berkeley nor the names
 *   of its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *
 * Austria Microsystems AS5047P/AMC Driver
 *
 * by Duncan Haldane
 *
 * v.alpha
 *
 * Revisions:

 *
 */

#include "as5047.h"
#include "ports.h"
#include "spi.h"
#include "tih.h"
#include "spi_controller.h"
#include "ams-enc.h"
#include "utils.h"
#include <string.h>

// Read/Write Access
#define READ (32768)
#define WRITE (0)
#define ANGLE_REG 0x3FFE

#define MOT_OFFSET 2383
EncObj motPos;

int motMin, motMax;

/*-----------------------------------------------------------------------------
 * Declaration of static functions
 -----------------------------------------------------------------------------*/
static void writeReg(unsigned short regaddr, unsigned short data );
static unsigned short readReg(unsigned short regaddr);
static unsigned short parity(unsigned short regaddr);
static inline void setupSPI();


/*-----------------------------------------------------------------------------
 * Public functions
 -----------------------------------------------------------------------------*/

// Note to self: FIFO State change requires power cycle!

void asSetup(void) {

  _LATB1 = 1;
  // setup SPI port
  setupSPI();  // Setup SPI for register configuration

  // Write settings
  writeReg(0x0019, 0x001D);
  writeReg(0x0018, 0x001C);

  // Write zero position
  setMotZero(1503);
  // setMotZero(2383); ???? This was determined value. Check on this
  motMin = -4000;
  motMax = 4000;
  tiHChangeMode(1, TIH_MODE_BRAKE);
  motPos.offset = 4613;
}

void setMotZero(unsigned short offs){
     // Write zero position
  unsigned short zhold;
  zhold = offs >> 6; //8MSB of zero position
  writeReg(0x0016, zhold);
  zhold = offs & 0x003F; //6 LSB of zero position
  writeReg(0x0017, zhold);
}

void setMotMin(int min){
  motMin = min;
}

int getMotMin(){
  return motMin;
}

void setMotMax(int Max){
  motMax = Max;
}

int getMotMax(){
  return motMax;
}


void as5047EncoderUpdatePos(void) {
    unsigned int new_pos;
    new_pos = readReg(ANGLE_REG) & 0x3FFF;
    unsigned int prev_pos = motPos.pos;
    if (new_pos > prev_pos) {
        if( (new_pos-prev_pos) > MAX_HALL/2 ) {//Subtract one Rev count if diff > 180
            motPos.oticks--;
        }
    } else {
        if( (prev_pos-new_pos) > MAX_HALL/2 ) {//Add one Rev count if -diff > 180
            motPos.oticks++;
        }
    }
    motPos.pos = new_pos;
}


/*-----------------------------------------------------------------------------
* ----------------------------------------------------------------------------
* The functions below are intended for internal use, i.e., private methods.
* Users are recommended to use functions defined above.
* ----------------------------------------------------------------------------
-----------------------------------------------------------------------------*/

/*****************************************************************************
* Function Name : writeReg
* Description : Write a data to a register
* Parameters : regaddr - address of register
* data - value to be written to the register
* Return Value : None
*****************************************************************************/
static void writeReg(unsigned short regaddr, unsigned short data )
{   
  regaddr = regaddr | parity(regaddr);
  data = data | parity(data);
  spic2BeginTransaction(AMS_CS);
  spic2Transmit16(regaddr);
  spic2EndTransaction();
  spic2BeginTransaction(AMS_CS);
  spic2Transmit16(data);
  spic2EndTransaction();
}

static void asFinishUpdate(unsigned int cause) {
  //TODO(rqou): don't ignore cause


}

/*****************************************************************************
* Function Name : readReg
* Description : Read a register
* Parameters : regaddr - address of register
* Return Value : register contents
*****************************************************************************/
static unsigned short readReg(unsigned short regaddr) {
  unsigned short c;
  regaddr = regaddr | 1<<14;
  regaddr = regaddr | parity(regaddr);
  spic2BeginTransaction(AMS_CS);
  spic2Transmit16(regaddr);
  _LATB1 = 1;
  delay_us(1);
  _LATB1 = 0;
  c = spic2Receive16();
  spic2EndTransaction();
  return c;
}

static unsigned short parity(unsigned short value)
{
    unsigned char cnt = 0;
    unsigned char i;
 
    for (i = 0; i < 16; i++)
    {
        if (value & 0x1)
        {
            cnt++;
        }
        value >>= 1;
    }
    cnt = cnt & 0x1;
    return cnt << 15;
}

/*****************************************************************************
* Function Name : setupSPI
* Description : Setup SPI for mpuscope
* Parameters : None
* Return Value : None
*****************************************************************************/
static inline void setupSPI()
{
  spicSetupChannel2(AMS_CS,
                    ENABLE_SCK_PIN &
                    ENABLE_SDO_PIN &
                    SPI_MODE16_ON &
                    SPI_SMP_OFF &
                    SPI_CKE_OFF &
                    SLAVE_ENABLE_OFF &
                    CLK_POL_ACTIVE_HIGH &
                    MASTER_ENABLE_ON &
                    PRI_PRESCAL_1_1 &
                    SEC_PRESCAL_6_1);

  spic2SetCallback(AMS_CS, &asFinishUpdate);
}
