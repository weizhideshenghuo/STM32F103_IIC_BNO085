#ifndef I2C_HAL_H
#define I2C_HAL_H

#include "sh2_hal.h"
#include "sh2_err.h"
#include "i2c.h"
#include "tim.h"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

// Initialize the SHTP HAL and return a reference to it.
sh2_Hal_t *sh2_hal_init(void);

void enableInts(void);
void disableInts(void);

#endif
