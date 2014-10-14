// Contents of this file are copyright Andrew Pullin, 2013

#ifndef __TELEM_H
#define __TELEM_H

#include "settings.h" //Required to set telemetry type
#include TELEM_INCLUDE

#ifndef TELEM_TYPE
#error "A telemtry type is not defined."
#endif

#include <stdint.h>

//Telemetry packet structure
typedef struct {
    uint32_t sampleIndex;
    uint32_t timestamp;
    TELEM_TYPE telemData;
} telemStruct_t;

#define TELEM_STREAM_OFF  0
#define TELEM_STREAM_ON   1

// Prototypes
void telemSetup(); //To be called in main
void telemReadbackSamples(unsigned long);
void telemSendDataDelay(telemStruct_t* sample, int delaytime_ms);
void telemSaveData(telemStruct_t *data);
void telemSetSamplesToSave(unsigned long n);
void telemErase(unsigned long);
void telemSetSkip(unsigned int skipnum);
void telemSetStartTime(void);
void telemGetSample(unsigned long sampNum, unsigned int sampLen, unsigned char *data);

// Added for compatibility with 'roach'
void telemSaveNow();

#endif  // __TELEM_H
