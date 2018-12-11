#ifndef _BATTERY_H_
#define _BATTERY_H_

#include "stm32f0xx_hal.h"
#include "stdio.h"
#include "stdbool.h"
#include "CJSON.h"
#include "usart.h"
#include "adc.h"
#include "stdarg.h"
#include "dma.h"
#include "stdlib.h"
#include "math.h"
#include "string.h"

void battery_update(void);

#endif
