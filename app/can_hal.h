#ifndef __CAN_HAL_H
#define __CAN_HAL_H

#include "can.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

extern uint16_t can_send_set_value;

void USER_CAN_init(void);
void USER_CAN_TxMessage(uint8_t aTxData[], uint8_t DLC);

#endif
