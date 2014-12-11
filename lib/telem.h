#ifndef __TELEM_H
#define __TELEM_H

#include "settings.h" //Required to set telemetry type

#ifndef TELEM_INCLUDE
#error "A telemtry structure is not defined."
#endif

#include TELEM_INCLUDE

// Prototypes
void telemSetup(); //To be called in main
void telemSetDivisor();
void telemReadbackSamples(unsigned long);
void telemSaveData(telemStruct_t *data);
void telemGetPID();
void telemSetSamplesToSave(unsigned long n);
void telemErase(unsigned long);
void telemSetSkip(unsigned int skipnum);
void telemSetStartTime(void);
void telemGetSample(unsigned long sampNum, unsigned int sampLen, unsigned char *data);

#endif  // __TELEM_H
