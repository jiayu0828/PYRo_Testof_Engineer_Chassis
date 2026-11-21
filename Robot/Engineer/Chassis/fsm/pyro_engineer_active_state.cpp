#include "pyro_engineer_chassis.h"

// =========================================================
// FSM：主动状态
// =========================================================
namespace pyro
{

void engineer_chassis_t::state_active_t::enter(owner *owner)
{
    // 电机使能由 execute 中的 _apply_motor_enable 按右拨杆统一管理；
    // 这里清空 PID 和麦轮功控滤波状态，避免切换瞬间积分/旧缩放值导致抽搐。
    for (int i = 0; i < 4; ++i)
    {
        if (owner->_ctx.pid.mecanum_pid[i])
        {
            owner->_ctx.pid.mecanum_pid[i]->clear();
        }
        if (owner->_ctx.mecanum_power_node[i] != nullptr)
        {
            auto *power_node = owner->_ctx.mecanum_power_node[i];
            power_node->target_cmd          = 0.0f;
            power_node->safe_cmd            = 0.0f;
            power_node->last_controlled_cmd = 0.0f;
        }
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
    if (owner->_ctx.pid.pitch_pid) owner->_ctx.pid.pitch_pid->clear();
    if (owner->_ctx.pid.roll_pid)  owner->_ctx.pid.roll_pid->clear();

    // 重新进入 ACTIVE 后，让腿部子 HFSM 在下一周期重新 enter 当前目标状态。
    owner->_leg_fsm.reset();
}

void engineer_chassis_t::state_active_t::execute(owner *owner)
{
    // 主动状态控制流水线
    owner->_apply_motor_enable();   // 1. 按右拨杆使能/失能各电机
    owner->_kinematics_solve();     // 2. 麦轮运动学逆解
    owner->_mecanum_control();      // 3. 麦轮速度环 + 四轮麦轮功控
    owner->_supercap_control();     // 4. 超级电容板使能/保活命令
    owner->_track_control();        // 5. 履带恒速/停止速度环
    owner->_leg_control();          // 6. 腿控子 HFSM：UP VMC / MID PID+重力 / DOWN 纯PID回零
    owner->_send_motor_command();   // 7. 发送电机指令
}

void engineer_chassis_t::state_active_t::exit(owner *owner)
{
    // 顶层 ACTIVE 退出时先退出腿部子 HFSM，调用当前腿控状态的 exit 回调。
    owner->_leg_fsm.exit(owner);
}

} // namespace pyro
