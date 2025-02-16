#include "foc.h"

#include "utils.h"
#include "motor.h"

Iph_ABC_t current_meas_ = {0.0f, 0.0f, 0.0f};

float p_gain;                       // [V/A]
float i_gain;                       // [V/As]
float v_current_control_integral_d; // [V]
float v_current_control_integral_q; // [V]
float Ibus;                         // DC bus current [A]
// Voltage applied at end of cycle:
float final_v_alpha; // [V]
float final_v_beta;  // [V]
float Id_setpoint;   // [A]
float Iq_setpoint;   // [A]
float Iq_measured;   // [A]
float Id_measured;   // [A]
float I_measured_report_filter_k;

// float max_allowed_current;    // [A]
// float overcurrent_trip_level; // [A]
// float acim_rotor_flux;        // [A]
// float async_phase_vel;        // [rad/s electrical]
// float async_phase_offset;     // [rad electrical]

extern float vbus_voltage;

// We should probably make FOC Current call FOC Voltage to avoid duplication.
bool FOC_voltage(float v_d, float v_q, float pwm_phase)
{
    float c       = our_arm_cos_f32(pwm_phase);
    float s       = our_arm_sin_f32(pwm_phase);
    float v_alpha = c * v_d - s * v_q;
    float v_beta  = c * v_q + s * v_d;
    return enqueue_voltage_timings(v_alpha, v_beta);
}

bool FOC_current(float Id_des, float Iq_des, float I_phase, float pwm_phase)
{
    // Clarke transform
    float Ialpha = current_meas_.phA;
    float Ibeta  = one_by_sqrt3 * (current_meas_.phB - current_meas_.phC);

    // Park transform
    float c_I = our_arm_cos_f32(I_phase);
    float s_I = our_arm_sin_f32(I_phase);
    float Id  = c_I * Ialpha + s_I * Ibeta;
    float Iq  = c_I * Ibeta - s_I * Ialpha;
    Iq_measured += I_measured_report_filter_k * (Iq - Iq_measured);
    Id_measured += I_measured_report_filter_k * (Id - Id_measured);

    // Current error
    float Ierr_d = Id_des - Id;
    float Ierr_q = Iq_des - Iq;

    // TODO look into feed forward terms (esp omega, since PI pole maps to RL tau)
    // Apply PI control
    float Vd = v_current_control_integral_d + Ierr_d * p_gain;
    float Vq = v_current_control_integral_q + Ierr_q * p_gain;

    float mod_to_V = (2.0f / 3.0f) * vbus_voltage;
    float V_to_mod = 1.0f / mod_to_V;
    float mod_d    = V_to_mod * Vd;
    float mod_q    = V_to_mod * Vq;

    // Vector modulation saturation, lock integrator if saturated
    // TODO make maximum modulation configurable
    float mod_scalefactor = 0.80f * sqrt3_by_2 * 1.0f / sqrtf(mod_d * mod_d + mod_q * mod_q);
    if (mod_scalefactor < 1.0f) {
        mod_d *= mod_scalefactor;
        mod_q *= mod_scalefactor;
        // TODO make decayfactor configurable
        v_current_control_integral_d *= 0.99f;
        v_current_control_integral_q *= 0.99f;
    } else {
        v_current_control_integral_d += Ierr_d * (i_gain * current_meas_period);
        v_current_control_integral_q += Ierr_q * (i_gain * current_meas_period);
    }

    // Compute estimated bus current
    Ibus = mod_d * Id + mod_q * Iq;

    // Inverse park transform
    float c_p       = our_arm_cos_f32(pwm_phase);
    float s_p       = our_arm_sin_f32(pwm_phase);
    float mod_alpha = c_p * mod_d - s_p * mod_q;
    float mod_beta  = c_p * mod_q + s_p * mod_d;

    // Report final applied voltage in stationary frame (for sensorles estimator)
    final_v_alpha = mod_to_V * mod_alpha;
    final_v_beta  = mod_to_V * mod_beta;

    // Apply SVM
    if (!enqueue_modulation_timings(mod_alpha, mod_beta)) {
        return false; // error set inside enqueue_modulation_timings
    }

    return true;
}
