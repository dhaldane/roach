#include "protocol.h"

#define EXP_WALL_JUMP       0
#define EXP_SINGLE_JUMP     1
#define EXP_VICON           2

#define MJ_IDLE         0
#define MJ_START        1
#define MJ_STOP         2
#define MJ_AIR          3
#define MJ_GND          4

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
void expStop(uint8_t stopSignal);
void expFlow();
void multiJumpFlow();
char footContact(void);
char footTakeoff(void);
void exp_set_params_sj(int16_t duration, int32_t leg_extension);
void exp_set_params_wj(int32_t launch_angle, int32_t launch_threshold, int32_t landing_angle, int32_t leg_extension, int32_t leg_retraction);

void setLegSetpoint(long length);
void setPushoffCmd(long cmd);
    
long calibPos(char idx);
unsigned int crankFromFemur(void);
void send_command_packet(packet_union_t *uart_tx_packet, int32_t position, uint32_t current, uint8_t flags);
