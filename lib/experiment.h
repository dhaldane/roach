#include "protocol.h"

#define EXP_WALL_JUMP       0
#define EXP_SINGLE_JUMP     1

void expStart(uint8_t mode);
void expFlow();
char footContact(void);
long calibPos(char idx);
unsigned int crankFromFemur(void);
void send_command_packet(packet_union_t *uart_tx_packet, int32_t position, uint32_t current, uint8_t flags);