#ifndef FOC_H
#define FOC_H

#include "drive_main.h"

bool FOC_voltage(float v_d, float v_q, float pwm_phase);
bool FOC_current(float Id_des, float Iq_des, float I_phase, float pwm_phase);

#endif
