//vr_telem.h , VelociRoACH specific telemetry packet format header

#include <stdint.h>

// Data structure type

typedef struct {
    int32_t posL; // Hall angle position
    int32_t posR;
    int32_t composL; // Commanded Hall angle position
    int32_t composR;
    int16_t dcL; // PWM duty cycle
    int16_t dcR;
    int16_t gyroX;
    int16_t gyroY;
    int16_t gyroZ;
    int16_t accelX;
    int16_t accelY;
    int16_t accelZ;
    int16_t bemfL;
    int16_t bemfR;
    int16_t Vbatt; // battery voltage
} vrTelemStruct_t;

//void vrTelemGetData(unsigned char* ptr);
void vrTelemGetData(vrTelemStruct_t* ptr);

unsigned int vrTelemGetSize();