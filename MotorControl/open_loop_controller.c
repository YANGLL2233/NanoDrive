#include "open_loop_controller.h"

#define clamp(value, min, max) ((value) < (min) ? (min) : ((value) > (max) ? (max) : (value)))

void open_loop_controller_update(OpenLoopController_t *open_loop_controller)
{
    float prev_Id = open_loop_controller->Idq_setpoint_[0];
    // float pre_Iq =  open_loop_controller->Idq_setpoint_[1];
    float prev_Vd = open_loop_controller->Vdq_setpoint_[0];
    // float pre_Vq =  open_loop_controller->Vdq_setpoint_[1];

    float phase     = open_loop_controller->phase_;
    float phase_vel = open_loop_controller->phase_vel_;

    float dt = 0.000125f;

    open_loop_controller->Idq_setpoint_[0] = clamp(open_loop_controller->target_current_,
                                                   prev_Id - open_loop_controller->max_current_ramp_ * dt,
                                                   prev_Id + open_loop_controller->max_current_ramp_ * dt);
    open_loop_controller->Vdq_setpoint_[0] = clamp(open_loop_controller->target_voltage_,
                                                   prev_Vd - open_loop_controller->max_voltage_ramp_ * dt,
                                                   prev_Vd + open_loop_controller->max_voltage_ramp_ * dt);

    phase_vel = clamp(open_loop_controller->target_vel_,
                      phase_vel - open_loop_controller->max_phase_vel_ramp_ * dt,
                      phase_vel + open_loop_controller->max_phase_vel_ramp_ * dt);

    open_loop_controller->phase_vel_      = phase_vel;
    open_loop_controller->phase_          = wrap_pm_pi(phase + phase_vel * dt);
    open_loop_controller->total_distance_ = open_loop_controller->total_distance_ + phase_vel * dt;
}
