#include "drv8323.h"

void drv8323_init(void)
{
    drv8323_enable();
    drv_delay(100);
    drv8323_cal_enable();
    drv_delay(500);
    drv8323_cal_disable();
}
