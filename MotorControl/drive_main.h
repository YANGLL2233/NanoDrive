#ifndef DRIVE_MAIN_H
#define DRIVE_MAIN_H

/* System header file */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/* ARM math lib */
#include "arm_math.h"

/* STM32 header file */
#include "stm32g4xx_hal.h"
#include "stm32g4xx.h"
#include "main.h"

/* user define header file */
#include "error.h"

// Period in [s]
#define CURRENT_MEAS_PERIOD ((float)2 * TIM_1_PERIOD_CLOCKS * (TIM_1_RCR + 1) / (float)TIM_1_CLOCK_HZ)
static const float current_meas_period = CURRENT_MEAS_PERIOD;

// Frequency in [Hz]
#define CURRENT_MEAS_HZ ((float)(TIM_1_CLOCK_HZ) / (float)(2 * TIM_1_PERIOD_CLOCKS * (TIM_1_RCR + 1)))
static const int current_meas_hz = CURRENT_MEAS_HZ;

#define VBUS_S_DIVIDER_RATIO   (11.0f)

#define CURRENT_SENSE_MIN_VOLT 0.3f
#define CURRENT_SENSE_MAX_VOLT 3.0f

typedef struct {
    float phA;
    float phB;
    float phC;
} Iph_ABC_t;

void drive_init(void);

#endif
