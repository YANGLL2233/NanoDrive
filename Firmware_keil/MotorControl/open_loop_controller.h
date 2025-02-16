#ifndef OPEN_LOOP_CONTROLLER_H
#define OPEN_LOOP_CONTROLLER_H

#include "utils.h"

typedef struct {
    // Config
    float max_current_ramp_;   // [A/s]
    float max_voltage_ramp_;   // [V/s]
    float max_phase_vel_ramp_; // [rad/s^2]

    // Inputs
    float target_vel_; // phase[rad/s]
    float target_current_;
    float target_voltage_;
    float initial_phase_;

    // State/Outputs
    uint32_t timestamp_;
    float Idq_setpoint_[2];
    float Vdq_setpoint_[2];
    float phase_;
    float phase_vel_;
    float total_distance_;
} OpenLoopController_t;

void open_loop_controller_update(OpenLoopController_t *open_loop_controller);

#endif
