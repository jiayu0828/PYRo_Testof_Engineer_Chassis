#include "pyro_engineer_chassis.h"

#include <cmath>

namespace pyro
{

// =========================================================
// 小坦巡航 VMC 适配函数
// 输入 theta/theta_dot 为小坦站立角坐标系；输出 tau_gravity
// 仍为小坦 theta 帧力矩，由调用层做本车 q 帧符号适配。
// 现在已经明白怎么推导的了 一会修改一下相关的参数的问题
// =========================================================
namespace
{



} // namespace



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
void engineer_chassis_t::_kinematics_solve()
{
    // 当前履带不参与底盘 vx/vy/wz 解算，这里只解算 4 个麦轮
    const auto wheel_speeds = _kinematics->solve(
        _ctx.cmd->chassis.vx,
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
//现在先关掉功率控制，只看我的原始的PID下的功率
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
// =========================================================
void engineer_chassis_t::_supercap_control()
{
#ifdef SUPERCAP_UART
    // 状态机参数内聚在函数内，避免再往 config.h 增加零散宏。
    constexpr bool     supercap_tx_en           = true;
    constexpr uint32_t enable_delay_ticks       = 1000U; // 1 kHz 控制任务：约 1 s 后使能
    constexpr uint32_t refresh_ticks            = 10U;   // 10 ms 周期保活发送
    constexpr uint32_t disable_debounce_ticks    = 30U;   // 失能去抖 30 ms
    constexpr uint8_t  tx_buffer_limit_energy_j = 60U;   // 给电容板的缓冲能量上限 byte
    constexpr float    tx_byte_max_f             = 255.0f;

    static bool     cap_enabled   = false;
    static uint32_t enable_timer  = 0;
    static uint32_t refresh_timer = 0;
    static uint32_t disable_timer = 0;

    supercap_drv_t *supercap = supercap_drv_t::get_instance();
    const bool cap_online = supercap->check_online();

    // 当前未启用裁判串口：本地以主动 FSM + 麦轮使能作为允许电容参与的安全条件。
    const bool chassis_armed =
        (_ctx.cmd != nullptr) &&
        (_ctx.cmd->mode == cmd_base_t::mode_t::ACTIVE) &&
        _ctx.cmd->mecanum_en;

    // power_referee 是裁判系统实测底盘功率；当前工程未接裁判串口，固定填 0。
    // 电容板自身还有 chassis_power_cap ADC 反馈可用于观察。
    _ctx.supercap_tx_cmd.power_referee              = 0U;
    _ctx.supercap_tx_cmd.power_buffer_limit_referee = tx_buffer_limit_energy_j;
    _ctx.supercap_tx_cmd.kill_chassis_user          = 0U;
    _ctx.supercap_tx_cmd.speed_up_user_now          = 0U;

    if (chassis_armed)
    {
        const float tx_power_limit_w = fminf(
            fmaxf(_ctx.data.wheel_power_limit_w, 0.0f), tx_byte_max_f);
        const float tx_buffer_energy_j = fminf(
            fmaxf(_ctx.data.wheel_power_buffer_energy_j, 0.0f), tx_byte_max_f);

        _ctx.supercap_tx_cmd.power_limit_referee =
            static_cast<uint8_t>(lroundf(tx_power_limit_w));
        _ctx.supercap_tx_cmd.power_buffer_referee =
            static_cast<uint8_t>(lroundf(tx_buffer_energy_j));
    }
    else
    {
        _ctx.supercap_tx_cmd.power_limit_referee  = 0U;
        _ctx.supercap_tx_cmd.power_buffer_referee = 0U;
    }

    const bool enable_request = supercap_tx_en && cap_online && chassis_armed;

    if (enable_request)
    {
        disable_timer = 0U;

        if (!cap_enabled)
        {
            refresh_timer = 0U;
            if (++enable_timer >= enable_delay_ticks)
            {
                enable_timer = 0U;
                cap_enabled  = true;
                _ctx.supercap_tx_cmd.use_cap = 1U;

                if (supercap->send_cmd(_ctx.supercap_tx_cmd) == PYRO_OK)
                {
                    ++_ctx.data.supercap_tx_count;
                }
            }
        }
        else if (++refresh_timer >= refresh_ticks)
        {
            refresh_timer = 0U;
            _ctx.supercap_tx_cmd.use_cap = 1U;

            if (supercap->send_cmd(_ctx.supercap_tx_cmd) == PYRO_OK)
            {
                ++_ctx.data.supercap_tx_count;
            }
        }
    }
    else
    {
        enable_timer  = 0U;
        refresh_timer = 0U;

        if (cap_enabled)
        {
            if (++disable_timer >= disable_debounce_ticks)
            {
                disable_timer = 0U;
                cap_enabled   = false;
                _ctx.supercap_tx_cmd.use_cap = 0U;

                if (supercap != nullptr &&
                    supercap->send_cmd(_ctx.supercap_tx_cmd) == PYRO_OK)
                {
                    ++_ctx.data.supercap_tx_count;
                }
            }
        }
        else
        {
            disable_timer = 0U;
            _ctx.supercap_tx_cmd.use_cap = 0U;
        }
    }

    _ctx.data.supercap_output_enabled = cap_enabled;
    _ctx.data.supercap_cmd_use_cap    = _ctx.supercap_tx_cmd.use_cap;
#else
    _ctx.data.supercap_online               = false;
    _ctx.data.supercap_output_enabled       = false;
    _ctx.data.supercap_cmd_use_cap          = 0U;
    _ctx.data.supercap_extra_power_allowed = false;
    _ctx.data.supercap_extra_power_w       = 0.0f;
#endif
}
// =========================================================
// _track_control()：履带速度环 PID
// 右拨杆 UP 使能电机；左拨杆 UP 且电机使能时按固定 RPM 恒速运行。
// =========================================================
void engineer_chassis_t::_track_control()
{
    const bool track_run_active = _ctx.cmd->track_en && _ctx.cmd->track_run;

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
