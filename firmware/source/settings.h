/*
 * Settings
 *
 * created on 2012-5-14 by fgb (derived from or_const.h by apullin)
 */

#ifndef __SETTINGS_H
#define __SETTINGS_H

/////// Radio settings ///////
#define RADIO_CHANNEL		0x19
#define RADIO_SRC_ADDR 		0x2052
#define RADIO_PAN_ID  	0x2050
//Hard-coded destination address, must match basestation or XBee addr
#define RADIO_DST_ADDR		0x2051


#define RADIO_TXPQ_MAX_SIZE   10
#define RADIO_RXPQ_MAX_SIZE   10

//Telemetry type setup
#define TELEM_TYPE vrTelemStruct_t
#define TELEM_INCLUDE "vr_telem.h"
#define TELEMPACKFUNC(x) vrTelemGetData(x)

#endif //_SETTINGS_H