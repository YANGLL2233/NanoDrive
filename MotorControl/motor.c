#include "motor.h"

#include "utils.h"
#include "drv8323.h"

const float adc_ref_voltage         = 3.3f;
const float adc_full_scale          = 4096.0f;
const float shunt_resistance_       = 0.0015f;
const float shunt_conductance_      = 1.0f / shunt_resistance_;
const float phase_current_rev_gain_ = 1.0f / 40.0f;

extern Iph_ABC_t current_meas_;

uint16_t next_timings_[3] = {TIM_1_PERIOD_CLOCKS / 2, TIM_1_PERIOD_CLOCKS / 2, TIM_1_PERIOD_CLOCKS / 2};

MotorConfig_t motor_config_ = {.pole_pairs = 7, .dc_calib_tau = 0.2f, .current_lim_margin = 10.0f};

Iph_ABC_t DC_calib_ = {0.0f, 0.0f, 0.0f};

float dc_calib_running_since_ = 3.0f; // current sensor calibration needs some time to settle
float I_bus_                  = 0.0f; // this motors contribution to the bus current

float effective_current_lim_ = 2.0f;  // [A]
float max_allowed_current_   = 25.0f; // [A] set in setup()
float max_dc_calib_          = 25.0f; // [A] set in setup()

float torque_setpoint_src_; // Usually points to the Controller object's output
float phase_vel_src_;       // Usually points to the Encoder object's output

float direction_ = 1.0f; // if -1 then positive torque is converted to negative Iq

bool next_timings_valid_ = false;

float vbus_voltage = 0.0f;

void disarm_with_error(uint32_t error)
{
    set_error(error);
    drv8323_disable();
}

static void apply_pwm_timings(uint16_t *timings, bool tentative)
{
    TIM1->CCR1 = timings[0];
    TIM1->CCR2 = timings[1];
    TIM1->CCR3 = timings[2];

    if (!tentative) {
        // Set the Automatic Output Enable so that the Master Output Enable
        // bit will be automatically enabled on the next update event.
        TIM1->BDTR |= TIM_BDTR_AOE;
    }
}

bool enqueue_modulation_timings(float mod_alpha, float mod_beta)
{
    float tA, tB, tC;
    if (SVM(mod_alpha, mod_beta, &tA, &tB, &tC) != 0) {
        set_error(ERROR_MODULATION_MAGNITUDE);
        return false;
    }

    next_timings_[0]    = (uint16_t)(tA * (float)TIM_1_PERIOD_CLOCKS);
    next_timings_[1]    = (uint16_t)(tB * (float)TIM_1_PERIOD_CLOCKS);
    next_timings_[2]    = (uint16_t)(tC * (float)TIM_1_PERIOD_CLOCKS);
    next_timings_valid_ = true;

    apply_pwm_timings(next_timings_, false);

    return true;
}

bool enqueue_voltage_timings(float v_alpha, float v_beta)
{
    float vfactor   = 1.0f / ((2.0f / 3.0f) * vbus_voltage);
    float mod_alpha = vfactor * v_alpha;
    float mod_beta  = vfactor * v_beta;
    if (!enqueue_modulation_timings(mod_alpha, mod_beta)) return false;
    return true;
}

void vbus_sense_adc_cb(uint32_t adc_value)
{
    float voltage_scale = adc_ref_voltage * VBUS_S_DIVIDER_RATIO / adc_full_scale;
    vbus_voltage        = (float)adc_value * voltage_scale;
}

float phase_current_from_adcval(uint32_t ADCValue)
{
    // Make sure the measurements don't come too close to the current sensor's hardware limitations
    if (ADCValue < CURRENT_ADC_LOWER_BOUND || ADCValue > CURRENT_ADC_UPPER_BOUND) {
        set_error(ERROR_CURRENT_SENSE_SATURATION);
        return 0.0f;
    }

    int adcval_bal     = (int)ADCValue - (1 << 11);
    float amp_out_volt = (3.3f / (float)(1 << 12)) * (float)adcval_bal;
    float shunt_volt   = amp_out_volt * phase_current_rev_gain_;
    float current      = shunt_volt * shunt_conductance_;
    return current;
}

void current_meas_cb(Iph_ABC_t *current)
{
    bool dc_calib_valid = (dc_calib_running_since_ >= motor_config_.dc_calib_tau * 7.5f) &&
                          (fabsf(DC_calib_.phA) < max_dc_calib_) && (fabsf(DC_calib_.phB) < max_dc_calib_) &&
                          (fabsf(DC_calib_.phC) < max_dc_calib_);

    if (dc_calib_valid) {
        current_meas_.phA = current->phA - DC_calib_.phA;

        current_meas_.phB = current->phB - DC_calib_.phB;

        current_meas_.phC = current->phC - DC_calib_.phC;

        float Itrip    = effective_current_lim_ + motor_config_.current_lim_margin;
        float Inorm_sq = 2.0f / 3.0f * (SQ(current_meas_.phA) + SQ(current_meas_.phB) + SQ(current_meas_.phC));

        if (Inorm_sq > SQ(Itrip)) { disarm_with_error(ERROR_CURRENT_LIMIT_VIOLATION); }

    } else {
        current_meas_.phA = current_meas_.phB = current_meas_.phC = 0.0f;
        disarm_with_error(ERROR_UNKNOWN_CURRENT_MEASUREMENT);
    }
}

void dc_calib_cb(Iph_ABC_t *current)
{
    const float dc_calib_period = (float)(2 * TIM_1_PERIOD_CLOCKS * (TIM_1_RCR + 1)) / TIM_1_CLOCK_HZ;

    const float calib_filter_k = fminf(dc_calib_period / motor_config_.dc_calib_tau, 1.0f);
    DC_calib_.phA += (current->phA - DC_calib_.phA) * calib_filter_k;
    DC_calib_.phB += (current->phB - DC_calib_.phB) * calib_filter_k;
    DC_calib_.phC += (current->phC - DC_calib_.phC) * calib_filter_k;
    dc_calib_running_since_ += dc_calib_period;
}
