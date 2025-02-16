//
// Created by YANGLLFF on 2025/2/11.
//

#include "encoder.h"

#include "as5047p.h"
#include "interface_uart.h"
#include "motor.h"

uint32_t MODE_FLAG_ABS = 0x100;

encoder_config_typedef config_ = {.mode                       = MODE_SPI_ABS_AS5047P,
                                  .calib_range                = 0.02f,
                                  .calib_scan_distance        = 16.0f * M_PI,
                                  .calib_scan_omega           = 4.0f * M_PI,
                                  .bandwidth                  = 1000.0f,
                                  .phase_offset               = 0,
                                  .phase_offset_float         = 0.0f,
                                  .cpr                        = (1024 * 16),
                                  .index_offset               = 0.0f,
                                  .use_index                  = false,
                                  .pre_calibrated             = true,
                                  .direction                  = 1,
                                  .use_index_offset           = true,
                                  .enable_phase_interpolation = true,
                                  .find_idx_on_lockin_only    = false,
                                  .ignore_illegal_hall_state  = false,
                                  .hall_polarity              = 0,
                                  .hall_polarity_calibrated   = false};

bool index_found_           = false;
bool is_ready_              = false;
int32_t shadow_count_       = 0;
int32_t count_in_cpr_       = 0;
float interpolation_        = 0.0f;
float phase_                = 0.0f; // [rad]
float phase_vel_            = 0.0f; // [rad/s]
float pos_estimate_counts_  = 0.0f; // [count]
float pos_cpr_counts_       = 0.0f; // [count]
float delta_pos_cpr_counts_ = 0.0f; // [count] phase detector result for debug
float vel_estimate_counts_  = 0.0f; // [count/s]
float pll_kp_               = 0.0f; // [count/s / count]
float pll_ki_               = 0.0f; // [(count/s^2) / count]
float calib_scan_response_  = 0.0f; // debug report from offset calib
int32_t pos_abs_            = 0;
float spi_error_rate_       = 0.0f;

float pos_estimate_ = 0.0f; // [turn]
float vel_estimate_ = 0.0f; // [turn/s]
float pos_circular_ = 0.0f; // [turn]

bool pos_estimate_valid_ = false;
bool vel_estimate_valid_ = false;

int16_t tim_cnt_sample_ = 0; //

bool abs_spi_pos_updated_ = false;
Mode mode_                = MODE_SPI_ABS_AS5047P;

TIM_HandleTypeDef *timer_;

extern MotorConfig_t motor_config_;

static void encoder_set_error(uint32_t error)
{
    vel_estimate_valid_ = false;
    pos_estimate_valid_ = false;
    set_error(error);
}

void encoder_setup(void)
{
    mode_ = config_.mode;
    encoder_update_pll_gains();
}

void encoder_update_pll_gains(void)
{
    pll_kp_ = 2.0f * config_.bandwidth;    // basic conversion to discrete time
    pll_ki_ = 0.25f * (pll_kp_ * pll_kp_); // Critically damped

    // Check that we don't get problems with discrete time approximation
    if (current_meas_period * pll_kp_ >= 1.0f) { encoder_set_error(ERROR_UNSTABLE_GAIN); }
}

void encoder_sample_now(void)
{
    uint16_t pos;
    switch (mode_) {
        case MODE_INCREMENTAL: {
            tim_cnt_sample_ = (int16_t)timer_->Instance->CNT;
        } break;

        case MODE_SPI_ABS_AS5047P: {
            // SPI Read AS5047P
            pos = read_as5047p_checked(ANGLECOM_ADDR);
        } break;

        default: {
            encoder_set_error(ERROR_UNSUPPORTED_ENCODER_MODE);
        } break;
    }
    pos_abs_             = pos;
    abs_spi_pos_updated_ = true;
    if (config_.pre_calibrated) { is_ready_ = true; }
}

bool encoder_update(void)
{
    // update internal encoder state.
    int32_t delta_enc       = 0;
    int32_t pos_abs_latched = pos_abs_; // LATCH

    switch (mode_) {
        case MODE_INCREMENTAL: {
            // TODO: use count_in_cpr_ instead as shadow_count_ can overflow
            // or use 64 bit
            int16_t delta_enc_16 = (int16_t)(tim_cnt_sample_ - (int16_t)shadow_count_);
            delta_enc            = (int32_t)delta_enc_16; // sign extend
        } break;

        case MODE_SPI_ABS_AS5047P: {

            if (abs_spi_pos_updated_ == false) {
                // Low pass filter the error
                spi_error_rate_ += current_meas_period * (1.0f - spi_error_rate_);
                if (spi_error_rate_ > 0.05f) {
                    encoder_set_error(ERROR_ABS_SPI_COM_FAIL);
                    return false;
                }
            } else {
                // Low pass filter the error
                spi_error_rate_ += current_meas_period * (0.0f - spi_error_rate_);
            }
            delta_enc = pos_abs_latched - count_in_cpr_; // LATCH
            delta_enc = mod(delta_enc, config_.cpr);
            if (delta_enc > config_.cpr / 2) { delta_enc -= config_.cpr; }
        } break;
        default: {
            encoder_set_error(ERROR_UNSUPPORTED_ENCODER_MODE);
            return false;
        } break;
    }

    shadow_count_ += delta_enc;
    count_in_cpr_ += delta_enc;
    count_in_cpr_ = mod(count_in_cpr_, config_.cpr);

    if (mode_ & MODE_FLAG_ABS) { count_in_cpr_ = pos_abs_latched; }

    // Memory for pos_circular
    float pos_cpr_counts_last = pos_cpr_counts_;

    //// run pll (for now pll is in units of encoder counts)
    // Predict current pos
    pos_estimate_counts_ += current_meas_period * vel_estimate_counts_;
    pos_cpr_counts_ += current_meas_period * vel_estimate_counts_;

    // discrete phase detector
    // if Encoder model is not in HALL mode, use under code
    float delta_pos_counts     = (float)(shadow_count_ - (int32_t)floorf(pos_estimate_counts_));
    float delta_pos_cpr_counts = (float)(count_in_cpr_ - (int32_t)floorf(pos_cpr_counts_));

    delta_pos_cpr_counts = wrap_pm(delta_pos_cpr_counts, (float)(config_.cpr));
    delta_pos_cpr_counts_ += 0.1f * (delta_pos_cpr_counts - delta_pos_cpr_counts_); // for debug
    // pll feedback
    pos_estimate_counts_ += current_meas_period * pll_kp_ * delta_pos_counts;
    pos_cpr_counts_ += current_meas_period * pll_kp_ * delta_pos_cpr_counts;
    pos_cpr_counts_ = fmodf_pos(pos_cpr_counts_, (float)(config_.cpr));
    vel_estimate_counts_ += current_meas_period * pll_ki_ * delta_pos_cpr_counts;
    bool snap_to_zero_vel = false;
    if (fabsf(vel_estimate_counts_) < 0.5f * current_meas_period * pll_ki_) {
        vel_estimate_counts_ = 0.0f; // align delta-sigma on zero to prevent jitter
        snap_to_zero_vel     = true;
    }

    // Outputs from Encoder for Controller
    pos_estimate_ = pos_estimate_counts_ / (float)config_.cpr;
    vel_estimate_ = vel_estimate_counts_ / (float)config_.cpr;

    // TODO: we should strictly require that this value is from the previous iteration
    // to avoid spinout scenarios. However that requires a proper way to reset
    // the encoder from error states.
    float pos_circular = pos_circular_;
    pos_circular += wrap_pm((pos_cpr_counts_ - pos_cpr_counts_last) / (float)config_.cpr, 1.0f);
    pos_circular  = fmodf_pos(pos_circular, 1.0f);
    pos_circular_ = pos_circular;

    //// run encoder count interpolation
    int32_t corrected_enc = count_in_cpr_ - config_.phase_offset;
    // if we are stopped, make sure we don't randomly drift
    if (snap_to_zero_vel || !config_.enable_phase_interpolation) {
        interpolation_ = 0.5f;
        // reset interpolation if encoder edge comes
        // TODO: This isn't correct. At high velocities the first phase in this count may very well not be at the edge.
    } else if (delta_enc > 0) {
        interpolation_ = 0.0f;
    } else if (delta_enc < 0) {
        interpolation_ = 1.0f;
    } else {
        // Interpolate (predict) between encoder counts using vel_estimate,
        interpolation_ += current_meas_period * vel_estimate_counts_;
        // don't allow interpolation indicated position outside of [enc, enc+1)
        if (interpolation_ > 1.0f) interpolation_ = 1.0f;
        if (interpolation_ < 0.0f) interpolation_ = 0.0f;
    }
    float interpolated_enc = corrected_enc + interpolation_;

    //// compute electrical phase
    // TODO avoid recomputing elec_rad_per_enc every time
    float elec_rad_per_enc = motor_config_.pole_pairs * 2 * M_PI * (1.0f / (float)(config_.cpr));
    float ph               = elec_rad_per_enc * (interpolated_enc - config_.phase_offset_float);

    // is ready
    if (is_ready_) {
        phase_ = wrap_pm_pi(ph) * config_.direction;

        phase_vel_ = (2 * M_PI) * vel_estimate_ * motor_config_.pole_pairs * config_.direction;
    }

    return true;
}
