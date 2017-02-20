//vr_telem.h , VelociRoACH specific telemetry packet format header

#include <stdint.h>

// Data structure type

typedef struct {
    int32_t posTail;
    int32_t posFemur;
    int32_t posMotor; // Hall angle position of BLDC

    int32_t pitch; // estimated angles
    int32_t roll;
    int32_t yaw;
    int32_t pitchSet; // Commanded Hall angle position 
 
    int16_t dcBLDC;  // Current draw of BLDC

    int16_t dcTail; // PWM duty cycle
    int16_t dcProp1; // PWM duty cycle
    int16_t dcProp2; // PWM duty cycle

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
