#include "pyro_engineer_chassis.h"

namespace pyro
{

// 左拨杆 MID：右摇杆直接力矩 + 重力补偿，不经过位置/速度 PID。
// 当前档位不叠加 Pitch/Roll 姿态虚拟力；阻尼、限幅和斜率限制仍作为安全边界。
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

        const float manual_torque = chassis->_ctx.cmd->leg.manual_torque;

        const auto ff = chassis->_leg_calc_feedforward(
            i, q, qdot, pitch, false, 0.0f, 0.0f);

        chassis->_ctx.data.target_leg_theta[i]   = q;
        chassis->_ctx.data.target_leg_radps[i]   = 0.0f;
        chassis->_ctx.data.leg_tau_attitude[i]   = ff.attitude_tau;
        chassis->_ctx.data.out_leg_gravity_torque[i] = ff.gravity_tau;
        chassis->_ctx.telemetry.leg_alpha[i]         = ff.theta_xt;
        chassis->_ctx.telemetry.leg_j_dynamic[i]     = ff.j_dynamic;
        chassis->_ctx.telemetry.leg_ff_scale[i]      = 1.0f;
        chassis->_ctx.telemetry.leg_gravity_valid[i] = ff.gravity_valid;
        chassis->_ctx.telemetry.leg_tau_manual[i]    = manual_torque;
        chassis->_ctx.telemetry.leg_control_mode[i]  = 3.0f;

        const float raw_torque = manual_torque + ff.gravity_tau;

        chassis->_leg_apply_common_output(i, raw_torque, qdot);
    }
}

void engineer_chassis_t::leg_state_pid_gravity_t::exit(owner *chassis)
{
    (void)chassis;
}

} // namespace pyro
