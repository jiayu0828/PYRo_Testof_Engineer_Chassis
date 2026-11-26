#include "pyro_engineer_chassis.h"

// =========================================================
// FSM：被动状态
// =========================================================
namespace pyro
{

void engineer_chassis_t::state_passive_t::enter(owner *owner)
{
    // 进入被动状态：禁用电机、清零控制边界、清空 PID
    for (auto *motor : owner->_ctx.motor.mecanum)
    {
        if (motor) motor->disable();
    }
    for (int i = 0; i < 4; i++)
    {
        owner->_ctx.data.target_wheel_rpm[i] = 0.0f;
        owner->_ctx.data.out_wheel_torque[i] = 0.0f;
        owner->_mec_en_state[i]              = false;

        if (owner->_ctx.mecanum_power_node[i] != nullptr)
        {
            auto *power_node = owner->_ctx.mecanum_power_node[i];
            power_node->target_cmd          = 0.0f;
            power_node->safe_cmd            = 0.0f;
            power_node->last_controlled_cmd = 0.0f;
        }
    }
    for (auto *pid : owner->_ctx.pid.mecanum_pid)
    {
        if (pid) pid->clear();
    }

    // 履带去使能、数据清零、清 PID
    for (auto *motor : owner->_ctx.motor.track)
    {
        if (motor) motor->disable();
    }
    for (int i = 0; i < 2; i++)
    {
        owner->_ctx.data.target_track_rpm[i] = 0.0f;
        owner->_ctx.data.out_track_torque[i] = 0.0f;
        owner->_track_en_state[i]            = false;
    }
    for (auto *pid : owner->_ctx.pid.track_pid)
    {
        if (pid) pid->clear();
    }

    // 腿去使能、控制数据清零、清 PID
    for (auto *motor : owner->_ctx.motor.leg)
    {
        if (motor) motor->disable();
    }
    for (int i = 0; i < 2; i++)
    {
        owner->_ctx.data.target_leg_theta[i]       = 0.0f;
        owner->_ctx.data.target_leg_radps[i]       = 0.0f;
        owner->_ctx.data.out_leg_torque[i]         = 0.0f;
        owner->_ctx.data.out_leg_gravity_torque[i] = 0.0f;
        owner->_ctx.data.leg_tau_attitude[i]       = 0.0f;

        owner->_leg_target_smooth[i]     = 0.0f;
        owner->_leg_zero_interp_init[i]  = false;
        owner->_leg_last_logic_torque[i] = 0.0f;
        owner->_leg_en_state[i]          = false;
    }

    // 纯观察量统一清零，避免 Live Watch 残留主动状态数值。
    owner->_ctx.telemetry = {};

    for (auto *pid : owner->_ctx.pid.leg_pos_pid)
    {
        if (pid) pid->clear();
    }
    for (auto *pid : owner->_ctx.pid.leg_vel_pid)
    {
        if (pid) pid->clear();
    }
    if (owner->_ctx.pid.pitch_pid) owner->_ctx.pid.pitch_pid->clear();
    if (owner->_ctx.pid.roll_pid)  owner->_ctx.pid.roll_pid->clear();
}

void engineer_chassis_t::state_passive_t::execute(owner *owner)
{
    // 被动状态每周期执行：输出力矩数组保持 0，再由统一发送边界下发。
    // 电机 disable 已在 enter() 中完成，这里不重复操作 CAN 使能帧。
    for (int i = 0; i < 4; i++)
    {
        owner->_ctx.data.out_wheel_torque[i] = 0.0f;
    }
    for (int i = 0; i < 2; i++)
    {
        owner->_ctx.data.out_track_torque[i] = 0.0f;
        owner->_ctx.data.out_leg_torque[i]   = 0.0f;
    }
    owner->_supercap_control();
    owner->_send_motor_command();
}

void engineer_chassis_t::state_passive_t::exit(owner *owner)
{
}

} // namespace pyro
