#ifndef CORDIC_SQRT_H
#define CORDIC_SQRT_H

#include "stm32g4xx_hal.h"
#include <stdbool.h>

bool CordicSqrt_Init(CORDIC_HandleTypeDef *hcordic);
float CordicSqrt(float x);

#endif
