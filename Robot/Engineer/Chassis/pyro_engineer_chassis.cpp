#include "pyro_engineer_chassis.h"

#include "pyro_algo_common.h"
#include "pyro_dji_motor_drv.h"

namespace pyro
{

// =========================================================
// 构造函数
// =========================================================
engineer_chassis_t::engineer_chassis_t()
    : module_base_t("engineer_chassis", 512, 1024,
                    task_base_t::priority_t::HIGH)
{
    _ctx = {};
}

// =========================================================
// _init()：初始化回调（只调用一次）
// =========================================================
status_t engineer_chassis_t::_init()
{
    // 1. 把外部依赖拷贝到上下文
    _ctx.motor = _module_deps.motor_deps;
    _ctx.pid   = _module_deps.pid_deps;

    // 2. 新建混合底盘运动学求解器（专家模式：4 个独立 K，支持非对称）
    //因为腿部的原因，导致我的底盘本身是一个梯形，具体参数粗测了一遍，如果不行，再找图纸测一下把
    _kinematics = new hybrid_kin_t(TRACK_SPACING, K_FL, K_FR, K_BL, K_BR);

    return PYRO_OK;
}

// =========================================================
// _update_feedback()：反馈更新（每周期先执行）
// =========================================================
void engineer_chassis_t::_update_feedback()
{
    for (int i = 0; i < 4; i++)
    {
        _ctx.motor.mecanum[i]->update_feedback();

        // get_current_rotate() 为电机轴 rad/s，乘 1/减速比 得轮轴 rad/s，
        // 再经 radps_to_rpm 转成轮轴 RPM，与目标侧量纲保持一致
        _ctx.data.current_wheel_rpm[i] = radps_to_rpm(
            _ctx.motor.mecanum[i]->get_current_rotate() *
            dji_m3508_motor_drv_t::reciprocal_reduction_ratio);

        _ctx.data.wheel_online[i] = _ctx.motor.mecanum[i]->is_online();
    }

    // 右侧两轮反馈取反：统一到逻辑正方向，使 PID 两侧方向一致
    _ctx.data.current_wheel_rpm[1] = -_ctx.data.current_wheel_rpm[1];
    _ctx.data.current_wheel_rpm[3] = -_ctx.data.current_wheel_rpm[3];
}

// =========================================================
// _kinematics_solve()：麦轮逆运动学
// =========================================================
void engineer_chassis_t::_kinematics_solve()
{
    // 巡航模式 track_en=false：履带输出 0，仅求解 4 个麦轮。
    // 未来越障启用履带时，把该开关改为由命令/状态驱动即可，求解器无需更换。
    const auto wheel_speeds = _kinematics->solve(
        _ctx.cmd->chassis.vx, _ctx.cmd->chassis.vy, _ctx.cmd->chassis.wz,
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
        _ctx.data.out_wheel_torque[i] = _ctx.pid.mecanum_pid[i]->calculate(
            _ctx.data.target_wheel_rpm[i], _ctx.data.current_wheel_rpm[i]);
    }
}

// =========================================================
// _send_motor_command()：发送所有电机指令
// =========================================================
void engineer_chassis_t::_send_motor_command() const
{
    // 右侧两轮发送时取反，把逻辑方向转回电调物理方向
    _ctx.motor.mecanum[0]->send_torque(_ctx.data.out_wheel_torque[0]);
    _ctx.motor.mecanum[1]->send_torque(-_ctx.data.out_wheel_torque[1]);
    _ctx.motor.mecanum[2]->send_torque(_ctx.data.out_wheel_torque[2]);
    _ctx.motor.mecanum[3]->send_torque(-_ctx.data.out_wheel_torque[3]);
}

// =========================================================
// _fsm_execute()：状态机执行
// =========================================================
void engineer_chassis_t::_fsm_execute()
{
    // 1. 把当前命令指针存到上下文
    _ctx.cmd = &_current_cmd;

    // 2. 根据 mode 切换顶层状态
    if (cmd_base_t::mode_t::ACTIVE == _current_cmd.mode)
    {
        _main_fsm.change_state(&_state_active);
    }
    else
    {
        _main_fsm.change_state(&_state_passive);
    }

    // 3. 执行当前状态
    _main_fsm.execute(this);
}

} // namespace pyro
