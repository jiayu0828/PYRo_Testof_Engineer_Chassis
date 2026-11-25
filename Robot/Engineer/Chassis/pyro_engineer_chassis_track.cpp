/**
 * @file pyro_engineer_chassis_track.cpp
 * @brief 两条履带的恒速/停止速度环 PID（成员函数实现分片）。
 *
 * 方案 A 文件拆分：函数定义从 pyro_engineer_chassis.cpp 原样剪切；
 * 类声明仍位于 pyro_engineer_chassis.h，控制逻辑不在纯搬运提交中改变。
 */
#include "pyro_engineer_chassis.h"

namespace pyro
{

// =========================================================
void engineer_chassis_t::_track_control()
{
    const bool track_run_active =
        _ctx.cmd->track_en &&
        _ctx.cmd->left_switch == left_switch_cmd_t::UP;

    for (int i = 0; i < 2; i++)
    {
        // 未使能时不生成目标/力矩，并清掉速度环积分，避免重新使能瞬间冲一下
        if (!_ctx.cmd->track_en)
        {
            _ctx.data.target_track_rpm[i] = 0.0f;
            _ctx.data.out_track_torque[i] = 0.0f;
            if (_ctx.pid.track_pid[i]) _ctx.pid.track_pid[i]->clear();
            continue;
        }

        _ctx.data.target_track_rpm[i] =
            track_run_active ? TRACK_CONSTANT_RPM : 0.0f;

        if (_ctx.pid.track_pid[i])
        {
            _ctx.data.out_track_torque[i] = _ctx.pid.track_pid[i]->calculate(
                _ctx.data.target_track_rpm[i],
                _ctx.data.current_track_rpm[i]);
        }
        else
        {
            _ctx.data.out_track_torque[i] = 0.0f;
        }
    }
}




// =========================================================
// _send_motor_command()：发送所有电机指令

} // namespace pyro
