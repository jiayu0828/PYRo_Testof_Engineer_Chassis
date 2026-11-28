/**
 * @file pyro_engineer_chassis_mecanum.cpp
 * @brief 麦轮逆运动学与四轮速度环 PID（成员函数实现分片）。
 *
 * 方案 A 文件拆分：函数定义从 pyro_engineer_chassis.cpp 原样剪切；
 * 类声明仍位于 pyro_engineer_chassis.h，控制逻辑不在纯搬运提交中改变。
 */
#include "pyro_engineer_chassis.h"

namespace pyro
{

// =========================================================
void engineer_chassis_t::_kinematics_solve()
{
    float vx_cmd = _ctx.cmd->chassis.vx;

    // 上台阶模式只限制麦轮纵向目标速度，不改变遥控原始命令和履带固定 RPM。
    if (_ctx.cmd->left_switch == left_switch_cmd_t::UP)
    {
        if (vx_cmd > CLIMB_MAX_VX)
        {
            vx_cmd = CLIMB_MAX_VX;
        }
        else if (vx_cmd < -CLIMB_MAX_VX)
        {
            vx_cmd = -CLIMB_MAX_VX;
        }
    }

    // 当前履带不参与底盘 vx/vy/wz 解算，这里只解算 4 个麦轮
    const auto wheel_speeds = _kinematics->solve(
        vx_cmd,
        _ctx.cmd->chassis.vy,
        _ctx.cmd->chassis.wz,
        false);

    // 线速度 m/s -> 轮轴 RPM（逻辑正方向，不在此取反）
    _ctx.data.target_wheel_rpm[0] = mps_to_rpm(wheel_speeds.mec_fl, WHEEL_RADIUS);
    _ctx.data.target_wheel_rpm[1] = mps_to_rpm(wheel_speeds.mec_fr, WHEEL_RADIUS);
    _ctx.data.target_wheel_rpm[2] = mps_to_rpm(wheel_speeds.mec_bl, WHEEL_RADIUS);
    _ctx.data.target_wheel_rpm[3] = mps_to_rpm(wheel_speeds.mec_br, WHEEL_RADIUS);
}

// =========================================================
// _mecanum_control()：麦轮速度环 PID
// =========================================================
void engineer_chassis_t::_mecanum_control()
{
    for (int i = 0; i < 4; i++)
    {
        if (_ctx.pid.mecanum_pid[i])
        {
            _ctx.data.out_wheel_torque[i] = _ctx.pid.mecanum_pid[i]->calculate(
                _ctx.data.target_wheel_rpm[i],
                _ctx.data.current_wheel_rpm[i]);
        }
        else
        {
            _ctx.data.out_wheel_torque[i] = 0.0f;
        }
    }

    // PID 原始输出之后、统一 CAN 发送之前进行麦轮专用功率限制。
    _mecanum_power_control();
}

// =========================================================
// _mecanum_power_control_init()：四个麦轮注册同一组标定参数

} // namespace pyro
