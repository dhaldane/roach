#include "experiment.h"
#include "as5047.h"
#include "ams-enc.h"
#include "tail_ctrl.h"

#define ABS(my_val) ((my_val) < 0) ? -(my_val) : (my_val)
#define LSB2ENCRAD = 0.000095873799
#define ENCRAD2LSB = 10430


extern EncObj motPos;
extern EncObj encPos[NUM_ENC];



bool footContact(void) {
    int eps = 300;
    if (ABS(calibPos(motPos) - crankFromFemur()) > eps)
    {
        LED_2 = 1;
        return 1;
    } else {.
        LED_2 = 0;
        return 0;
    }
}

long calibPos(struct EncObj enc){
    long temp;
    temp = (long)(enc.pos << 2);       // pos 14 bits 0x0 -> 0x3fff
    return temp + (enc.oticks << 16);
}

long crankFromFemur(void) { // TODO: Avoid float conversion
    float femur = calibPos(encPos[1])*LSB2ENCRAD;
    float crank = 2.003*femur^5 + 7.434*femur^4 + 9.429*femur^3 + 2.691*femur^2 + 0.3893*femur + 0.001175
    return (long) crank * ENCRAD2LSB;
}
