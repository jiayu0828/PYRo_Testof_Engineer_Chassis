#include "pyro_engineer_chassis.h"

namespace pyro
{

// 左拨杆 UP：VMC 重力/软墙/Pitch-Roll 姿态力状态。
// 进入瞬间参考角从当前角起步并清双环 PID，保证与其他腿控状态无扰切换。
void engineer_chassis_t::leg_state_vmc_t::enter(owner *chassis)
{
    for (int i = 0; i < 2; ++i)
    {
        chassis->_leg_enter_active_mode(
            i, chassis->_ctx.data.current_leg_theta[i]);
    }
}

void engineer_chassis_t::leg_state_vmc_t::execute(owner *chassis)
{
    const float pitch = chassis->_ctx.data.imu_pitch;
    const float roll  = chassis->_ctx.data.imu_roll;

    // Pitch/Roll PID 输出的是整机任务空间虚拟力，两腿循环外只计算一次。
    const bool attitude_active =
        LEG_ATTITUDE_PID_EN &&
        chassis->_ctx.pid.pitch_pid != nullptr &&
        chassis->_ctx.pid.roll_pid != nullptr;

    float f_pitch = 0.0f;
    float f_roll  = 0.0f;

    if (attitude_active)
    {
        f_pitch = chassis->_ctx.pid.pitch_pid->calculate(
            LEG_TARGET_PITCH_RAD, pitch);
        f_roll = chassis->_ctx.pid.roll_pid->calculate(
            LEG_TARGET_ROLL_RAD, roll);
    }
    else
    {
        if (chassis->_ctx.pid.pitch_pid) chassis->_ctx.pid.pitch_pid->clear();
        if (chassis->_ctx.pid.roll_pid)  chassis->_ctx.pid.roll_pid->clear();
    }

    chassis->_ctx.data.att_force_pitch = f_pitch;
    chassis->_ctx.data.att_force_roll  = f_roll;

    for (int i = 0; i < 2; ++i)
    {
        const float q    = chassis->_ctx.data.current_leg_theta[i];
        const float qdot = chassis->_ctx.data.current_leg_radps[i];

        // VMC 不追摇杆位置目标；平滑目标跟随当前 q，切到 PID 档首帧无跳变。
        chassis->_leg_zero_interp_init[i]         = false;
        chassis->_leg_target_smooth[i]            = q;
        chassis->_ctx.data.target_leg_theta[i]   = q;
        chassis->_ctx.data.target_leg_radps[i]   = 0.0f;
        chassis->_ctx.data.leg_tau_manual_pid[i] = 0.0f;
        chassis->_ctx.data.leg_control_mode[i]   = 2.0f;

        chassis->_leg_update_vmc_feedforward(
            i, q, qdot, pitch, attitude_active, f_pitch, f_roll);

        const float raw_torque =
            chassis->_ctx.data.out_leg_gravity_torque[i] +
            chassis->_ctx.data.leg_tau_wall[i] +
            chassis->_ctx.data.leg_tau_attitude[i];

        chassis->_leg_apply_common_output(i, raw_torque, qdot);
    }
}

void engineer_chassis_t::leg_state_vmc_t::exit(owner *chassis)
{
    (void)chassis;
}

} // namespace pyro
