// Contents of this file are copyright Andrew Pullin, 2013

#include "utils.h"
#include "settings.h"
#include "dfmem.h"
#include "telem.h"
#include "radio.h"
#include "at86rf231_driver.h"
#include "led.h"
#include "sclock.h"
#include "timer.h"
#include "cmd.h" //for CMD codes
#include <string.h> //for memcpy

telemStruct_t telemBuffer;
unsigned int telemDataSize;
unsigned int telemPacketSize;

#if defined(__RADIO_HIGH_DATA_RATE)
#define READBACK_DELAY_TIME_MS 3
#else
#define READBACK_DELAY_TIME_MS 10
#endif

#define TELEM_HEADER_SIZE   sizeof(telemBuffer.sampleIndex) + sizeof(telemBuffer.timestamp)

////////   Private variables   ////////////////

unsigned long samplesToSave;
unsigned long sampIdx = 0;


//Offset for time value when recording samples
unsigned long telemStartTime;

static DfmemGeometryStruct mem_geo;

///////////// Private functions //////////////

void telemSetup() {

    dfmemGetGeometryParams(&mem_geo); // Read memory chip sizing

    //Telemetry packet size is set at startupt time.
    telemDataSize = sizeof (TELEM_TYPE); //OctoRoACH specific
    telemPacketSize = sizeof (telemStruct_t);
}

void telemSetSamplesToSave(unsigned long n) {
    samplesToSave = n;
    sampIdx = 0;
}

void telemSendDataDelay(telemStruct_t* sample, int delaytime_ms) {

    radioSendData(RADIO_DEST_ADDR, 0, CMD_FLASH_READBACK, telemPacketSize,
           (unsigned char*) sample, 0 );
    
    delay_ms(delaytime_ms); // allow radio transmission time
    LED_2 = ~LED_2;
}

void telemReadbackSamples(unsigned long numSamples) {
    int delaytime_ms = READBACK_DELAY_TIME_MS;
    unsigned long i = 0; //will actually be the same as the sampleIndex

    LED_GREEN = 1;
    //Disable motion interrupts for readback
    //_T1IE = 0; _T5IE=0; //TODO: what is a cleaner way to do this?

    telemStruct_t sampleData;
    DisableIntT1;
    for (i = 0; i < numSamples; i++) {
        //Retireve data from flash
        telemGetSample(i, sizeof (sampleData), (unsigned char*) (&sampleData));
        //Reliable send, with linear backoff
        do {
            //debugpins1_set();
            telemSendDataDelay(&sampleData, delaytime_ms);
            //Linear backoff
            delaytime_ms += 0;
            //debugpins1_clr();
        } while (trxGetLastACKd() == 0);
        delaytime_ms = READBACK_DELAY_TIME_MS;
    }
    EnableIntT1;
    LED_GREEN = 0;

}

//Saves telemetry data structure into flash memory, in order

void telemSaveData(telemStruct_t * telemPkt) {

    //Write the packet header info to the DFMEM
    dfmemSave((unsigned char*) telemPkt, sizeof(telemStruct_t));
    samplesToSave--;

    //This is done here instead of the ISR because telemSaveData() will only be
    //executed if samplesToSave > 0 upon entry.
    if (samplesToSave == 0) {
        //Done sampling, commit last buffer
        dfmemSync();
    }
}

void telemErase(unsigned long numSamples) {
    //dfmemEraseSectorsForSamples(numSamples, sizeof (telemU));
    // TODO (apullin) : Add an explicit check to see if the number of saved
    //                  samples will fit into memory!
    LED_2 = 1;
    unsigned int firstPageOfSector, i;

    //avoid trivial case
    if (numSamples == 0) {
        return;
    }

    //Saves to dfmem will NOT overlap page boundaries, so we need to do this level by level:
    unsigned int samplesPerPage = mem_geo.bytes_per_page / telemPacketSize; //round DOWN int division
    unsigned int numPages = (numSamples + samplesPerPage - 1) / samplesPerPage; //round UP int division
    unsigned int numSectors = (numPages + mem_geo.pages_per_sector - 1) / mem_geo.pages_per_sector;

    //At this point, it is impossible for numSectors == 0
    //Sector 0a and 0b will be erased together always, for simplicity
    //Note that numSectors will be the actual number of sectors to erase,
    //   even though the sectors themselves are numbered starting at '0'
    DisableIntT1;
    dfmemEraseSector(0); //Erase Sector 0a
    dfmemEraseSector(8); //Erase Sector 0b

    //Start erasing the rest from Sector 1:
    for (i = 1; i <= numSectors; i++) {
        firstPageOfSector = mem_geo.pages_per_sector * i;
        //hold off until dfmem is ready for secort erase command
        //while (!dfmemIsReady());
        //LED should blink indicating progress
        LED_2 = ~LED_2;
        //Send actual erase command
        dfmemEraseSector(firstPageOfSector);
    }

    EnableIntT1;
    //Leadout flash, should blink faster than above, indicating the last sector
    //while (!dfmemIsReady()) {
    //    LED_2 = ~LED_2;
    //    delay_ms(75);
    //}
    LED_2 = 0; //Green LED off

    //Since we've erased, reset our place keeper vars
    dfmemZeroIndex();
}


void telemGetSample(unsigned long sampNum, unsigned int sampLen, unsigned char *data)
{
    unsigned int samplesPerPage = mem_geo.bytes_per_page / sampLen; //round DOWN int division
    unsigned int pagenum = sampNum / samplesPerPage;
    unsigned int byteOffset = (sampNum - pagenum*samplesPerPage)*sampLen;

    dfmemRead(pagenum, byteOffset, sampLen, data);
}

//This function is a setter for the telemStartTime variable,
//which is used to offset the recorded times for telemetry, such that
//they start at approx. 0, instead of reflecting the total number of
//sclock ticks.

void telemSetStartTime(void) {
    telemStartTime = sclockGetTime();
}
