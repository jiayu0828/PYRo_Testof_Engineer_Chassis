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
        motor->disable();
    }
    for (int i = 0; i < 4; i++)
    {
        owner->_ctx.data.target_wheel_rpm[i] = 0.0f;
        owner->_ctx.data.out_wheel_torque[i] = 0.0f;
    }
    for (auto *pid : owner->_ctx.pid.mecanum_pid)
    {
        pid->clear();
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
    }
    for (auto *pid : owner->_ctx.pid.track_pid)
    {
        if (pid) pid->clear();
    }
}

void engineer_chassis_t::state_passive_t::execute(owner *owner)
{
    // 被动状态每周期执行：发零扭矩保证安全
    for (int i = 0; i < 4; i++)
    {
        owner->_ctx.data.out_wheel_torque[i] = 0.0f;
    }
    for (int i = 0; i < 2; i++)
    {
        owner->_ctx.data.out_track_torque[i] = 0.0f;
    }
    owner->_send_motor_command();
}

void engineer_chassis_t::state_passive_t::exit(owner *owner)
{
}

} // namespace pyro
