//vr_telem.h , VelociRoACH specific telemetry packet format header

#include <stdint.h>

// Data structure type

typedef struct {
    int32_t posTail;
    int32_t posFemur;
    int32_t posMotor; // Hall angle position
    int32_t bodyAngle;
    int32_t composBody; // Commanded Hall angle position
    int32_t composMotor;
    int16_t dcTail; // PWM duty cycle
    int16_t dcBLDC;
    int16_t gyroX;
    int16_t gyroY;
    int16_t gyroZ;
    int16_t accelX;
    int16_t accelY;
    int16_t accelZ;
} vrTelemStruct_t;

//void vrTelemGetData(unsigned char* ptr);
void vrTelemGetData(vrTelemStruct_t* ptr);

unsigned int vrTelemGetSize();