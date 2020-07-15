#ifndef BNO085_APP_H
#define BNO085_APP_H

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "sh2.h"
#include "sh2_util.h"
#include "sh2_err.h"
#include "sh2_SensorValue.h"
#include "sh2_hal.h"
#include "i2c_hal.h"
#include "Quaternion.h"
#include "can_hal.h"

extern uint8_t IMU_data[8];

void BNO085_init(void);
void BNO085_service(void);
    
#endif
