#ifndef MOTOR_H
#define MOTOR_H

#include "drive_main.h"

#define CURRENT_ADC_LOWER_BOUND ((uint32_t)((float)(1 << 12) * CURRENT_SENSE_MIN_VOLT / 3.3f))
#define CURRENT_ADC_UPPER_BOUND ((uint32_t)((float)(1 << 12) * CURRENT_SENSE_MAX_VOLT / 3.3f))

typedef struct {
    bool pre_calibrated; // can be set to true to indicate that all values here are valid
    int32_t pole_pairs;
    float calibration_current;          // [A]
    float resistance_calib_max_voltage; // [V] - You may need to increase this if this voltage isn't sufficient to drive
                                        // calibration_current through the motor.
    float phase_inductance;             // to be set by measure_phase_inductance
    float phase_resistance;             // to be set by measure_phase_resistance
    float torque_constant; // [Nm/A] for PM motors, [Nm/A^2] for induction motors. Equal to 8.27/Kv of the motor

    // Read out max_allowed_current to see max supported value for current_lim.
    float current_lim;        //[A]
    float current_lim_margin; // Maximum violation of current_lim
    float torque_lim;         //[Nm].
    // Value used to compute shunt amplifier gains
    float requested_current_range;   // [A]
    float current_control_bandwidth; // [rad/s]
    float inverter_temp_limit_lower;
    float inverter_temp_limit_upper;

    float acim_gain_min_flux;   // [A]
    float acim_autoflux_min_Id; // [A]
    bool acim_autoflux_enable;
    float acim_autoflux_attack_gain;
    float acim_autoflux_decay_gainf;

    bool R_wL_FF_enable; // Enable feedforwards for R*I and w*L*I terms
    bool bEMF_FF_enable; // Enable feedforward for bEMF

    float I_bus_hard_min;
    float I_bus_hard_max;
    float I_leak_max;

    float dc_calib_tau;

} MotorConfig_t;

bool enqueue_modulation_timings(float mod_alpha, float mod_beta);
bool enqueue_voltage_timings(float v_alpha, float v_beta);

void disarm_with_error(uint32_t error);

void vbus_sense_adc_cb(uint32_t adc_value);
float phase_current_from_adcval(uint32_t ADCValue);
void current_meas_cb(Iph_ABC_t *current);
void dc_calib_cb(Iph_ABC_t *current);

#endif
