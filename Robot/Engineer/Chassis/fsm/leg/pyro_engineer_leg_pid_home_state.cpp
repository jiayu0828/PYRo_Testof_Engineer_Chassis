#include "pyro_engineer_chassis.h"

namespace pyro
{

// 左拨杆 DOWN：限速插值返回校准零点，只跑纯双环 PID + 全模式关节阻尼。
void engineer_chassis_t::leg_state_pid_home_t::enter(owner *chassis)
{
    for (int i = 0; i < 2; ++i)
    {
        chassis->_leg_enter_active_mode(
            i, chassis->_ctx.data.current_leg_theta[i]);
    }
}

void engineer_chassis_t::leg_state_pid_home_t::execute(owner *chassis)
{
    for (int i = 0; i < 2; ++i)
    {
        const float q    = chassis->_ctx.data.current_leg_theta[i];
        const float qdot = chassis->_ctx.data.current_leg_radps[i];

        chassis->_leg_update_home_target(i, q);
        const float pid_tau = chassis->_leg_run_dual_pid(i);

        chassis->_leg_clear_feedforward(i);
        chassis->_ctx.telemetry.leg_tau_manual[i] = pid_tau;
        chassis->_ctx.telemetry.leg_control_mode[i]   = 1.0f;

        chassis->_leg_apply_common_output(i, pid_tau, qdot);
    }
}

void engineer_chassis_t::leg_state_pid_home_t::exit(owner *chassis)
{
    (void)chassis;
}

} // namespace pyro
