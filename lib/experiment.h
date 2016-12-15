#include "protocol.h"

#define EXP_WALL_JUMP       0
#define EXP_SINGLE_JUMP     1

typedef struct {
    int16_t duration;
    int32_t leg_extension; // Units: 15.16 radian
} experiment_params_sj_t;

typedef struct {
    int32_t launch_angle;       // Units: 15.16 radian
    int32_t launch_threshold;   // Units: 15.16 radian
    int32_t landing_angle;      // Units: 15.16 radian
    int32_t leg_extension;      // Units: 15.16 radian
    int32_t leg_retraction;     // Units: 15.16 radian
} experiment_params_wj_t;

void expStart(uint8_t mode);
void expFlow();
char footContact(void);
void exp_set_params_sj(int16_t duration, int32_t leg_extension);
void exp_set_params_wj(int32_t launch_angle, int32_t launch_threshold, int32_t landing_angle, int32_t leg_extension, int32_t leg_retraction);
    
long calibPos(char idx);
unsigned int crankFromFemur(void);
void send_command_packet(packet_union_t *uart_tx_packet, int32_t position, uint32_t current, uint8_t flags);