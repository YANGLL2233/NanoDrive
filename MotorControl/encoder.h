//
// Created by YANGLLFF on 2025/2/11.
//

#ifndef NANODRIVE_MOTORCONTROL_ENCODER_H
#define NANODRIVE_MOTORCONTROL_ENCODER_H

#include "utils.h"
#include "board.h"

typedef enum { MODE_INCREMENTAL = 0, MODE_HALL, MODE_SIN_COS, MODE_SPI_ABS_AS5047P } Mode;

typedef struct {
    Mode mode;
    float calib_range;         // Accuracy required to pass encoder cpr check
    float calib_scan_distance; // rad electrical
    float calib_scan_omega;    // rad/s electrical
    float bandwidth;
    int32_t phase_offset;     // Offset between encoder count and rotor electrical phase
    float phase_offset_float; // Sub-count phase alignment offset
    int32_t cpr;              // encoder cpr
    float index_offset;
    bool use_index;
    bool pre_calibrated; // If true, this means the offset stored in
                         // configuration is valid and does not need
                         // be determined by run_offset_calibration.
                         // In this case the encoder will enter ready
                         // state as soon as the index is found.
    int32_t direction;   // direction with respect to motor
    bool use_index_offset;
    bool enable_phase_interpolation; // Use velocity to interpolate inside the
                                     // count state
    bool find_idx_on_lockin_only;    // Only be sensitive during locking scan constant
                                     // vel state
    bool ignore_illegal_hall_state;  // do not error on bad states like 000 or 111
    uint8_t hall_polarity;
    bool hall_polarity_calibrated;

} encoder_config_typedef;

/**
 *
 */
void encoder_setup(void);
void encoder_update_pll_gains(void);
void encoder_sample_now(void);
bool encoder_update(void);

#endif // NANODRIVE_MOTORCONTROL_ENCODER_H
