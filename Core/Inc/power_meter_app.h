#ifndef POWER_METER_APP_H
#define POWER_METER_APP_H

#include "stm32g0xx_hal.h"

void PowerMeter_Init(I2C_HandleTypeDef *hi2c);
void PowerMeter_Update(void);

#endif
