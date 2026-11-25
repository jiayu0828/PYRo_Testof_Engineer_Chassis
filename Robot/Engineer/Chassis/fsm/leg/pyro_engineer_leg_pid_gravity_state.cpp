#include "pyro_engineer_chassis.h"

namespace pyro
{

// 左拨杆 MID：右摇杆手动目标 + 位置/速度双环 PID，叠加重力/软墙前馈。
// 当前档位不叠加 Pitch/Roll 姿态虚拟力，便于先验证重力前馈符号和大小。
void engineer_chassis_t::leg_state_pid_gravity_t::enter(owner *chassis)
{
    for (int i = 0; i < 2; ++i)
    {
        chassis->_leg_enter_active_mode(
            i, chassis->_ctx.data.current_leg_theta[i]);
    }
}

void engineer_chassis_t::leg_state_pid_gravity_t::execute(owner *chassis)
{
    for (int i = 0; i < 2; ++i)
    {
        const float q     = chassis->_ctx.data.current_leg_theta[i];
        const float qdot  = chassis->_ctx.data.current_leg_radps[i];
        const float pitch = chassis->_ctx.data.imu_pitch;

        chassis->_leg_update_manual_target(
            i, chassis->_ctx.cmd->leg.target_theta);

        const float pid_tau = chassis->_leg_run_dual_pid(i);

        chassis->_leg_update_vmc_feedforward(
            i, q, qdot, pitch, false, 0.0f, 0.0f);

        chassis->_ctx.telemetry.leg_tau_manual_pid[i] = pid_tau;
        chassis->_ctx.telemetry.leg_control_mode[i]   = 3.0f;

        const float raw_torque =
            pid_tau +
            chassis->_ctx.data.out_leg_gravity_torque[i] +
            chassis->_ctx.data.leg_tau_wall[i];

        chassis->_leg_apply_common_output(i, raw_torque, qdot);
    }
}

void engineer_chassis_t::leg_state_pid_gravity_t::exit(owner *chassis)
{
    (void)chassis;
}

} // namespace pyro
