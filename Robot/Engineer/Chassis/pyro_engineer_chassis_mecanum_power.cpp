/**
 * @file pyro_engineer_chassis_mecanum_power.cpp
 * @brief 四个麦轮的模型功率控制与功率节点初始化（成员函数实现分片）。
 *
 * 方案 A 文件拆分：函数定义从 pyro_engineer_chassis.cpp 原样剪切；
 * 类声明仍位于 pyro_engineer_chassis.h，控制逻辑不在纯搬运提交中改变。
 */
#include "pyro_engineer_chassis.h"
#include <cmath>

namespace pyro
{

// =========================================================
void engineer_chassis_t::_mecanum_power_control_init()
{
    // 四个麦轮共用这一组 FL.csv 拟合参数，作为当前四轮平均模型。
    constexpr float common_k1 = 0.03622596f; // 机械功率项系数，对应 cmd * rpm
    constexpr float common_k2 = 0.08411378f; // 铜损/热损耗系数，对应 cmd^2
    constexpr float common_k3 = 0.00005988f; // 高频摩擦损耗，对应 rpm^2
    constexpr float common_k4 = 0.01115514f; // 库仑摩擦损耗，对应 |rpm|
    constexpr float common_k5 = 0.75f;       // 静态基础功耗 W
    constexpr float common_alpha = 0.00393f; // 铜阻温度修正系数

    // 裁判系统未启用时，缓冲环目标能量和 PID 也直接放在这里初始化。
    constexpr float buffer_target_energy_j = 60.0f;
    constexpr float buffer_pid_kp = 0.1f;
    constexpr float buffer_pid_ki = 0.01f;
    constexpr float buffer_pid_kd = 0.0f;

    const power_fit_params_t mecanum_common_params{
        common_k1,
        common_k2,
        common_k3,
        common_k4,
        common_k5,
        common_alpha,
    };

    auto &power_controller = power_controller_t::get_instance();
    power_controller.config_buffer_loop(
        buffer_target_energy_j,
        buffer_pid_kp,
        buffer_pid_ki,
        buffer_pid_kd);

    for (int i = 0; i < 4; ++i)
    {
        _ctx.mecanum_power_node[i] =
            power_controller.register_motor(mecanum_common_params);
    }
}

// =========================================================
// _mecanum_power_control()：仅对四个麦轮做模型功率限制
// =========================================================
void engineer_chassis_t::_mecanum_power_control()
{
    // 麦轮功控运行参数：只作用于 4 个麦轮，不影响履带和腿部。
    constexpr bool  power_control_en          = true;  // false=旁路，保留麦轮 PID 原始输出
    constexpr float default_power_limit_w     = 50.0f; // 无裁判系统时的四轮总预测功率上限 W
    constexpr float default_buffer_energy_j   = 60.0f; // 无裁判系统时的虚拟缓冲能量 J
    constexpr float scale_valid_cmd_threshold = 0.001f; // 判断实际缩放/介入的最小指令差
    constexpr float default_cap_extra_power_w     = 150.0f;  // 超级电容额外功率口径 W（裁判系统未启用时）
    _ctx.data.wheel_power_control_active  = false;
    _ctx.data.wheel_power_total_w         = 0.0f;
    _ctx.data.supercap_extra_power_allowed = false;

    if (!power_control_en)
    {
        return;
    }

    float power_limit_w   = default_power_limit_w;
    float buffer_energy_j = default_buffer_energy_j;

#ifdef REFEREE_UART
    // 启用裁判系统串口后，在线帧有效时自动切换为裁判系统下发值。
    referee_drv_t *referee = referee_drv_t::get_instance();
    if (referee != nullptr && referee->is_online())
    {
        const referee_data_t &referee_data = referee->get_data();

        if (referee_data.robot_status.chassis_power_limit > 0U)
        {
            power_limit_w =
                static_cast<float>(referee_data.robot_status.chassis_power_limit);
        }
        if (referee_data.power_heat.buffer_energy > 0U)
        {
            buffer_energy_j =
                static_cast<float>(referee_data.power_heat.buffer_energy);
        }
    }
#endif

    _ctx.data.wheel_power_limit_w         = power_limit_w;
    _ctx.data.wheel_power_buffer_energy_j = buffer_energy_j;

    // 1. 填入四个麦轮节点；uncontrolled_cmd=0 表示麦轮 PID 输出全部受控。
    for (int i = 0; i < 4; ++i)
    {
        _ctx.data.wheel_cmd_before_power_ctrl[i] = _ctx.data.out_wheel_torque[i];
        _ctx.data.wheel_cmd_after_power_ctrl[i]  = _ctx.data.out_wheel_torque[i];
        _ctx.data.wheel_power_scale[i]            = 1.0f;

        power_node_t *node = _ctx.mecanum_power_node[i];

        node->target_cmd       = _ctx.data.out_wheel_torque[i];
        node->uncontrolled_cmd = 0.0f;
        node->rpm              = _ctx.data.current_wheel_rpm[i];
        node->temp             = _ctx.data.current_wheel_temp[i];
    }

    // 2. 超级电容额外功率口径：
    //    电容板在线、状态机已周期发送 use_cap=1，且电容电压高于使能门限后，
    //    才给四轮麦轮功控放宽一个固定功率；电压低于回差门限立即收回。
    constexpr bool  supercap_extra_power_en       = true;//先关掉超级电容
    constexpr float supercap_enable_voltage_cv    = 1900.0f; // 19.00 V
    constexpr float supercap_disable_voltage_cv   = 1750.0f; // 17.50 V，滞回防抖
    static bool     supercap_voltage_ready        = false;

    if (!supercap_extra_power_en || !_ctx.data.supercap_online || !_ctx.data.supercap_output_enabled)
    {
        supercap_voltage_ready = false;
    }
    else if (_ctx.data.supercap_voltage_cv >= supercap_enable_voltage_cv)
    {
        supercap_voltage_ready = true;
    }
    else if (_ctx.data.supercap_voltage_cv <= supercap_disable_voltage_cv)
    {
        supercap_voltage_ready = false; 
    }
    if(_ctx.data.supercap_error_flag == 1U){
        auto &warning = warning_drv_t::instance();
        warning.set_buzzer_warning_level(
        warning_drv_t::buzzer_warning_level_t::WARNING_LEVEL_3);
    }//如果有error_flag 直接报警
    else{
        auto &warning = warning_drv_t::instance();
        warning.set_buzzer_warning_level(warning_drv_t::buzzer_warning_level_t::NONE); 
    }//如果清除了error_flg 就先不管它了
    if(_ctx.data.supercap_chassis_power_raw > 100.0f){
        auto &warning = warning_drv_t::instance();
        warning.set_buzzer_warning_level(
        warning_drv_t::buzzer_warning_level_t::WARNING_LEVEL_2);
    }//但如果无论有无error_flag 功率没控住就直接报警

    const float cap_extra_power_w =
        (supercap_extra_power_en && _ctx.data.supercap_online &&
         _ctx.data.supercap_output_enabled && supercap_voltage_ready)
            ? default_cap_extra_power_w + _ctx.data.supercap_cap_power_raw
            : 0.0f;

    _ctx.data.supercap_extra_power_allowed = cap_extra_power_w > 0.0f;
    _ctx.data.supercap_extra_power_w       = cap_extra_power_w;

    // 3. 四节点共用一个总功率求解器；第三参为电容端允许补充的总功率 W。
    auto &power_controller = power_controller_t::get_instance();
    power_controller.solve(power_limit_w, buffer_energy_j, cap_extra_power_w);

    // 4. 用安全指令覆盖麦轮 PID 输出；履带和腿部数组完全不在这里处理。
    for (int i = 0; i < 4; ++i)
    {
        power_node_t *node = _ctx.mecanum_power_node[i];

        _ctx.data.out_wheel_torque[i]                = node->safe_cmd;
        _ctx.data.wheel_cmd_after_power_ctrl[i]      = node->safe_cmd;
        _ctx.data.wheel_power_predict_w[i]           = node->power_predict;
        _ctx.data.wheel_power_total_w               += node->power_predict;

        const float raw_cmd = _ctx.data.wheel_cmd_before_power_ctrl[i];
        if (fabsf(raw_cmd) > scale_valid_cmd_threshold)
        {
            _ctx.data.wheel_power_scale[i] = node->safe_cmd / raw_cmd;
        }
        else
        {
            _ctx.data.wheel_power_scale[i] = 1.0f;
        }

        if (fabsf(node->safe_cmd - raw_cmd) > scale_valid_cmd_threshold)
        {
            _ctx.data.wheel_power_control_active = true;
        }
    }
}

// =========================================================
// _supercap_control()：超级电容板命令组装、使能状态机和周期发送
// 主动/被动 FSM 都调用：主动状态满足条件后使能电容，被动状态延时发 use_cap=0。

} // namespace pyro
