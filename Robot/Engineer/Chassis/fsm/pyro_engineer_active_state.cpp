#include "pyro_engineer_chassis.h"

// =========================================================
// FSM：主动状态
// =========================================================
namespace pyro
{

void engineer_chassis_t::state_active_t::enter(owner *owner)
{
    // 电机使能由 execute 中的 _apply_motor_enable 按右拨杆统一管理；
    // 这里只清空 PID，避免切换瞬间积分突变导致抽搐
    for (auto *pid : owner->_ctx.pid.mecanum_pid)
    {
        if (pid) pid->clear();
    }
    for (auto *pid : owner->_ctx.pid.track_pid)
    {
        if (pid) pid->clear();
    }
    for (auto *pid : owner->_ctx.pid.leg_pos_pid)
    {
        if (pid) pid->clear();
    }
    for (auto *pid : owner->_ctx.pid.leg_vel_pid)
    {
        if (pid) pid->clear();
    }
}

void engineer_chassis_t::state_active_t::execute(owner *owner)
{
    // 主动状态控制流水线
    owner->_apply_motor_enable();   // 1. 按右拨杆使能/失能各电机
    owner->_kinematics_solve();     // 2. 麦轮运动学逆解
    owner->_mecanum_control();      // 3. 麦轮速度环
    owner->_track_control();        // 4. 履带恒速 + 速度环
    owner->_leg_control();          // 5. 腿双环 PID
    owner->_send_motor_command();   // 6. 发送电机指令
}

void engineer_chassis_t::state_active_t::exit(owner *owner)
{
}

} // namespace pyro
