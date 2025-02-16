#ifndef DRV8323_H
#define DRV8323_H

#include "main.h"

#define drv8323_enable()                                               \
    {                                                                  \
        HAL_GPIO_WritePin(ENABLE_GPIO_Port, ENABLE_Pin, GPIO_PIN_SET); \
    }

#define drv8323_disable()                                                \
    {                                                                    \
        HAL_GPIO_WritePin(ENABLE_GPIO_Port, ENABLE_Pin, GPIO_PIN_RESET); \
    }

#define drv8323_cal_enable()                                     \
    {                                                            \
        HAL_GPIO_WritePin(CAL_GPIO_Port, CAL_Pin, GPIO_PIN_SET); \
    }

#define drv8323_cal_disable()                                      \
    {                                                              \
        HAL_GPIO_WritePin(CAL_GPIO_Port, CAL_Pin, GPIO_PIN_RESET); \
    }

#define drv_delay(x)  \
    {                 \
        HAL_Delay(x); \
    }

void drv8323_init(void);

#endif
