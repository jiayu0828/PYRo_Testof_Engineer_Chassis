#include "pyro_engineer_chassis.h"

#include <cmath>

namespace pyro
{



// 腿部 Pitch/Roll 姿态环使能与目标角已下沉到 engineer_config.h，
// 供 Chassis/fsm/leg/ 下的 VMC 状态文件直接使用。

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
// 获取上下文（非 const 版本，基类只有 const 版本）
// =========================================================
engineer_context_t &engineer_chassis_t::get_ctx()
{
    return _ctx;
}

// =========================================================
// _init()：初始化回调（只调用一次）
// =========================================================
status_t engineer_chassis_t::_init()
{
    // 1. 把外部依赖拷贝到上下文
    _ctx.motor      = _module_deps.motor_deps;
    _ctx.pid        = _module_deps.pid_deps;
    _ctx.powermeter = _module_deps.powermeter;

    // 2. 混合底盘运动学求解器（4 个独立 K，支持前后轮距非对称）
    _kinematics = new hybrid_kin_t(TRACK_SPACING, K_FL, K_FR, K_BL, K_BR);

    // 3. 四个麦轮注册模型功控节点（共用 FL 标定参数；不作用于履带和腿）
    _mecanum_power_control_init();

    return PYRO_OK;
}

// =========================================================
// 初始化报警任务
// =========================================================
void chassis_warning_init(){
     // warning_drv_t 是全局类，不在 pyro 命名空间内
    warning_drv_t &warning = warning_drv_t::instance();

    // 初始化 WS2812 + 蜂鸣器，并创建 20ms 循环任务
    warning.start();
}

// =========================================================
// _update_feedback()：反馈更新（每周期先执行）
// =========================================================
void engineer_chassis_t::_update_feedback()
{
    // ---- 麦轮反馈：硬件方向只由 MECANUM_MOTOR_SIGN 统一修正 ----
    for (int i = 0; i < 4; i++)
    {
        _ctx.motor.mecanum[i]->update_feedback();

        // get_current_rotate() 为电机轴 rad/s，乘 1/减速比得轮轴 rad/s，
        // 再经 radps_to_rpm 转成轮轴 RPM，与目标侧量纲保持一致。
        const float wheel_motor_rpm = radps_to_rpm(
            _ctx.motor.mecanum[i]->get_current_rotate() *
            dji_m3508_motor_drv_t::reciprocal_reduction_ratio);

        _ctx.data.current_wheel_rpm[i] =
            static_cast<float>(MECANUM_MOTOR_SIGN[i]) * wheel_motor_rpm;
        _ctx.data.current_wheel_temp[i] =
            static_cast<float>(_ctx.motor.mecanum[i]->get_temperature());
        _ctx.data.wheel_online[i] = _ctx.motor.mecanum[i]->is_online();
    }

    // ---- 履带电机反馈（达妙 4310，输出轴 rad/s，不除减速比）----
    for (int i = 0; i < 2; i++)
    {
        if (!_ctx.motor.track[i]) continue;

        _ctx.motor.track[i]->update_feedback();
        const float track_motor_rpm =
            radps_to_rpm(_ctx.motor.track[i]->get_current_rotate());
        _ctx.data.current_track_rpm[i] =
            static_cast<float>(TRACK_MOTOR_SIGN[i]) * track_motor_rpm;
        _ctx.data.track_online[i] = _ctx.motor.track[i]->is_online();
    }

    // ---- 腿部反馈、校准与 IMU ----
    _leg_feedback();

    // ---- 0x212 CAN 功率计：只刷新测量值，不参与当前控制闭环 ----
    if (_ctx.powermeter != nullptr)
    {
        powermeter_data meter_frame{};
        if (_ctx.powermeter->get_data(meter_frame))
        {
            _ctx.powermeter_feedback      = meter_frame;
            _ctx.data.power_meter_current = meter_frame.current;
            _ctx.data.power_meter_voltage = meter_frame.voltage;
            _ctx.data.power_meter_w       = meter_frame.power;
            ++_ctx.data.power_meter_rx_count;
        }
    }

#ifdef SUPERCAP_UART
    // ---- 超级电容板：驱动任务完成 CRC/解包，控制任务只拷贝最新帧 ----
    supercap_drv_t *supercap = supercap_drv_t::get_instance();
    if (supercap != nullptr)
    {
        _ctx.supercap_rx_feedback = supercap->get_feedback();
        const auto &cap_fb        = _ctx.supercap_rx_feedback;

        _ctx.data.supercap_online            = supercap->check_online();
        _ctx.data.supercap_cap_power_raw     = static_cast<float>(cap_fb.cap_power_cap)/100.0f - 250.0f;
        _ctx.data.supercap_chassis_power_raw = static_cast<float>(cap_fb.chassis_power_cap)/100.0f;
        _ctx.data.supercap_voltage_cv        = static_cast<float>(cap_fb.vot_cap);
        _ctx.data.supercap_error_flag        = cap_fb.error_flag;
        _ctx.data.supercap_low_flag          = cap_fb.cap_low_flag;
        _ctx.data.supercap_over_current_flag = cap_fb.over_normal_c_l;
    }
#else
    _ctx.data.supercap_online = false;
#endif
}

// =========================================================
// _apply_motor_enable()：按右拨杆命令边沿式使能/失能各电机
// =========================================================
void engineer_chassis_t::_apply_motor_enable()
{
    // 麦轮：右拨杆 MID/UP 使能
    for (int i = 0; i < 4; i++)
    {
        if (!_ctx.motor.mecanum[i]) continue;

        if (_ctx.cmd->mecanum_en && !_mec_en_state[i])
        {
            _ctx.motor.mecanum[i]->enable();
            _mec_en_state[i] = true;
        }
        else if (!_ctx.cmd->mecanum_en && _mec_en_state[i])
        {
            _ctx.motor.mecanum[i]->disable();
            _mec_en_state[i] = false;
        }
    }

    // 履带：右拨杆 UP 使能
    for (int i = 0; i < 2; i++)
    {
        if (!_ctx.motor.track[i]) continue;

        if (_ctx.cmd->track_en && !_track_en_state[i])
        {
            _ctx.motor.track[i]->enable();
            _track_en_state[i] = true;
        }
        else if (!_ctx.cmd->track_en && _track_en_state[i])
        {
            _ctx.motor.track[i]->disable();
            _track_en_state[i] = false;
        }
    }

    // 腿：右拨杆 UP 使能
    for (int i = 0; i < 2; i++)
    {
        if (!_ctx.motor.leg[i]) continue;

        if (_ctx.cmd->leg.en && !_leg_en_state[i])
        {
            _ctx.motor.leg[i]->enable();
            _leg_en_state[i] = true;
        }
        else if (!_ctx.cmd->leg.en && _leg_en_state[i])
        {
            _ctx.motor.leg[i]->disable();
            _leg_en_state[i] = false;
        }
    }
}

// =========================================================
// _kinematics_solve()：麦轮逆运动学

// =========================================================
void engineer_chassis_t::_send_motor_command() const
{
    // 所有逻辑力矩在电机发送边界统一乘安装方向表；
    // 控制层内部不要再对某一侧单独写负号，避免双重反向。
    for (int i = 0; i < 4; i++)
    {
        if (_ctx.motor.mecanum[i])
        {
            const float send_torque =
                static_cast<float>(MECANUM_MOTOR_SIGN[i]) *
                _ctx.data.out_wheel_torque[i];
            _ctx.motor.mecanum[i]->send_torque(send_torque);
        }
    }

    for (int i = 0; i < 2; i++)
    {
        if (_ctx.motor.track[i])
        {
            const float send_torque =
                static_cast<float>(TRACK_MOTOR_SIGN[i]) *
                _ctx.data.out_track_torque[i];
            _ctx.motor.track[i]->send_torque(send_torque);
        }
    }

    for (int i = 0; i < 2; i++)
    {
        if (_ctx.motor.leg[i])
        {
            const float send_torque =
                static_cast<float>(LEG_MOTOR_SIGN[i]) *
                _ctx.data.out_leg_torque[i];
            _ctx.motor.leg[i]->send_torque(send_torque);
        }
    }
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
