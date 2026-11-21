#include "pyro_engineer_chassis.h"

// =========================================================
// FSM：被动状态
// =========================================================
namespace pyro
{

void engineer_chassis_t::state_passive_t::enter(owner *owner)
{
    // 进入被动状态：禁用电机、清零数据、清空 PID
    for (auto *motor : owner->_ctx.motor.mecanum)
    {
        if (motor) motor->disable();
    }
    for (int i = 0; i < 4; i++)
    {
        owner->_ctx.data.target_wheel_rpm[i] = 0.0f;
        owner->_ctx.data.out_wheel_torque[i] = 0.0f;
        owner->_ctx.data.wheel_cmd_before_power_ctrl[i] = 0.0f;
        owner->_ctx.data.wheel_cmd_after_power_ctrl[i]  = 0.0f;
        owner->_ctx.data.wheel_power_scale[i]           = 1.0f;
        owner->_mec_en_state[i] = false;

        if (owner->_ctx.mecanum_power_node[i] != nullptr)
        {
            auto *power_node = owner->_ctx.mecanum_power_node[i];
            power_node->target_cmd          = 0.0f;
            power_node->safe_cmd            = 0.0f;
            power_node->last_controlled_cmd = 0.0f;
        }
    }
    owner->_ctx.data.wheel_power_total_w        = 0.0f;
    owner->_ctx.data.wheel_power_control_active = false;
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
        owner->_track_en_state[i] = false;
    }
    for (auto *pid : owner->_ctx.pid.track_pid)
    {
        if (pid) pid->clear();
    }

    // 腿去使能、数据清零、清 PID
    for (auto *motor : owner->_ctx.motor.leg)
    {
        if (motor) motor->disable();
    }
    for (int i = 0; i < 2; i++)
    {
        owner->_ctx.data.target_leg_theta[i] = 0.0f;
        owner->_ctx.data.target_leg_radps[i] = 0.0f;
        owner->_ctx.data.out_leg_torque[i]         = 0.0f;
        owner->_ctx.data.out_leg_gravity_torque[i] = 0.0f;
        owner->_ctx.data.leg_alpha[i]              = 0.0f;
        owner->_ctx.data.leg_j_dynamic[i]          = 0.0f;
        owner->_ctx.data.leg_gravity_valid[i]      = false;
        owner->_ctx.data.leg_tau_wall[i]           = 0.0f;
        owner->_ctx.data.leg_tau_damping[i]        = 0.0f;
        owner->_ctx.data.leg_tau_attitude[i]       = 0.0f;
        owner->_ctx.data.leg_ff_scale[i]           = 0.0f;
        owner->_ctx.data.leg_tau_manual_pid[i]     = 0.0f;
        owner->_ctx.data.leg_control_mode[i]       = 0.0f;
        owner->_leg_target_smooth[i]               = 0.0f;
        owner->_leg_zero_interp_init[i]            = false;
        owner->_leg_manual_map_active[i]           = false;
        owner->_leg_manual_cmd_offset[i]           = 0.0f;
        owner->_leg_last_logic_torque[i]           = 0.0f;
        owner->_leg_en_state[i]                    = false;
    }
    owner->_ctx.data.att_force_pitch = 0.0f;
    owner->_ctx.data.att_force_roll  = 0.0f;
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
