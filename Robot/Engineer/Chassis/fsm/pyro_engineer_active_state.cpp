#include "pyro_engineer_chassis.h"

// =========================================================
// FSM：主动状态
// =========================================================
namespace pyro
{

void engineer_chassis_t::state_active_t::enter(owner *owner)
{
    // 进入主动状态：使能电机、清空 PID，避免切换瞬间积分突变导致抽搐
    for (auto *motor : owner->_ctx.motor.mecanum)
    {
        // motor->enable();
    }
    for (auto *pid : owner->_ctx.pid.mecanum_pid)
    {
        pid->clear();
    }

    // 履带电机使能 + 清 PID（空指针保护）
    for (auto *motor : owner->_ctx.motor.track)
    {
        if (motor) motor->enable();
    }
    for (auto *pid : owner->_ctx.pid.track_pid)
    {
        if (pid) pid->clear();
    }
}

void engineer_chassis_t::state_active_t::execute(owner *owner)
{
    // 主动状态控制流水线
    owner->_kinematics_solve();   // 1. 运动学逆解
    owner->_mecanum_control();   // 2. 麦轮速度环
    owner->_track_control();     // 3. 履带速度环
    owner->_send_motor_command(); // 4. 发送电机指令
}

void engineer_chassis_t::state_active_t::exit(owner *owner)
{
}

} // namespace pyro
