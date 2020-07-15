#ifndef __CAN_HAL_h
#define __CAN_HAL_h

#include "stdio.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

void CAN_SetTxPacket(uint8_t data_send[],uint8_t data_len);
void CAN_Init(void);

#endif
