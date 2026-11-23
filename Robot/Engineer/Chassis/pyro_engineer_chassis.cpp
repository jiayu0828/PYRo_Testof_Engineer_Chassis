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
struct leg_xt_vmc_result_t
{
    float j_dynamic{};   // 姿态投影后的动态雅可比
    float tau_gravity{}; // 单腿重力补偿力矩（小坦 theta 帧）
    bool  valid{false};   // 力平衡分母是否有效
};

leg_xt_vmc_result_t calc_leg_xiaotan_vmc(float theta_raw, float pitch)
{
    leg_xt_vmc_result_t result{};

    // 多项式只在小坦实测拟合域内求值
    const float theta =
        fminf(fmaxf(theta_raw, LEG_XT_MIN_POS), LEG_XT_MAX_POS);

    const float cp = cosf(pitch);
    const float sp = sinf(pitch);

    const float j_x = evaluate_polynomial(
        theta, LEG_XT_JX_COEF, LEG_XT_JX_DEGREE);
    const float j_y = evaluate_polynomial(
        theta, LEG_XT_JY_COEF, LEG_XT_JY_DEGREE);
    const float x_b = evaluate_polynomial(
        theta, LEG_XT_XB_COEF, LEG_XT_XB_DEGREE);
    const float y_b = evaluate_polynomial(
        theta, LEG_XT_YB_COEF, LEG_XT_YB_DEGREE);

    // 小坦巡航 _leg_vmc() 前轴 O 力平衡
    const float y_wheel    = y_b - LEG_XT_H_HIP_OFFSET;
    const float numerator  = LEG_XT_DIST_FRONT * cp - LEG_XT_H_COG * sp;
    const float denominator =
        (LEG_XT_DIST_FRONT + LEG_XT_DIST_HIP + x_b) * cp + y_wheel * sp;

    float f_gravity_ff = 0.0f;
    if (fabsf(denominator) > 1e-4f)
    {
        f_gravity_ff =
            LEG_POLY_VEH_MASS * LEG_GRAVITY * numerator / denominator;
        result.valid = true;
    }

    result.j_dynamic   = j_y * cp + j_x * sp;
    result.tau_gravity = result.j_dynamic * (0.5f * f_gravity_ff);
    return result;
}
} // namespace

// IMU 一阶低通系数：每帧保留 15% 新采样、85% 历史值
static constexpr float LEG_IMU_LPF_ALPHA = 0.15f;

// module_base 控制任务使用 vTaskDelayUntil 固定 1 ms 周期；插值按标称 dt 计算。
static constexpr float LEG_CONTROL_DT_S = 0.001f;

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
// _calibrate_leg()：腿软件置零
// =========================================================
void engineer_chassis_t::_calibrate_leg()
{

    _ctx.data.leg_theta_cal[0] = LEG_CALI_LEFT;
    _ctx.data.leg_theta_cal[1] = LEG_CALI_RIGHT;

    for (int i = 0; i < 2; i++)
    {
        if (!_ctx.motor.leg[i]) continue;

        // 把当前多圈角记为校准基准，相对角归零
        // _ctx.data.leg_theta_cal[i]     = _ctx.data.leg_theta_unwrap[i];
        _ctx.data.current_leg_theta[i] = 0.0f;
        _ctx.data.target_leg_theta[i]  = 0.0f;
        _ctx.data.target_leg_radps[i]  = 0.0f;
        _leg_target_smooth[i]          = 0.0f;
        _leg_zero_interp_init[i]       = false;
        _leg_manual_map_active[i]       = false;
        _leg_manual_cmd_offset[i]      = 0.0f;
        _ctx.data.leg_calibrated[i]    = true;

        // 清 PID，避免切换瞬间积分突变
        if (_ctx.pid.leg_pos_pid[i]) _ctx.pid.leg_pos_pid[i]->clear();
        if (_ctx.pid.leg_vel_pid[i]) _ctx.pid.leg_vel_pid[i]->clear();
    }

    // 重新校准时同步清空姿态环，避免旧积分带入校准后的第一次 VMC
    if (_ctx.pid.pitch_pid) _ctx.pid.pitch_pid->clear();
    if (_ctx.pid.roll_pid)  _ctx.pid.roll_pid->clear();
}

// =========================================================
// _leg_feedback()：腿反馈（unwrap、校准、IMU）
// =========================================================
void engineer_chassis_t::_leg_feedback()
{
    // 校准脉冲（右拨杆进入 UP 的边沿触发一次）
    if (_ctx.cmd->leg.calibrate)
    {
        _calibrate_leg();
    }

    // IMU 姿态：直接读取弧度，并减去固定安装角
    float yaw_rad   = 0.0f;
    float pitch_rad = 0.0f;
    float roll_rad  = 0.0f;
    ins_drv_t::get_instance()->get_rads_n(&yaw_rad, &pitch_rad, &roll_rad);

    const float raw_pitch = pitch_rad - PITCH_OFFSET_RAD;
    const float raw_roll  = roll_rad - ROLL_OFFSET_RAD;

    // 一阶低通：首帧直接采用原始值，避免启动时从 0 缓慢爬升
    if (!_imu_lpf_init)
    {
        _imu_pitch_lpf = raw_pitch;
        _imu_roll_lpf  = raw_roll;
        _imu_lpf_init  = true;
    }
    else
    {
        _imu_pitch_lpf =
            LEG_IMU_LPF_ALPHA * raw_pitch +
            (1.0f - LEG_IMU_LPF_ALPHA) * _imu_pitch_lpf;
        _imu_roll_lpf =
            LEG_IMU_LPF_ALPHA * raw_roll +
            (1.0f - LEG_IMU_LPF_ALPHA) * _imu_roll_lpf;
    }

    _ctx.data.imu_pitch_raw = raw_pitch;
    _ctx.data.imu_pitch     = _imu_pitch_lpf;
    _ctx.data.imu_roll_raw  = raw_roll;
    _ctx.data.imu_roll      = _imu_roll_lpf;

    for (int i = 0; i < 2; i++)
    {
        _ctx.motor.leg[i]->update_feedback();
        _ctx.data.leg_online[i] = _ctx.motor.leg[i]->is_online();

        // 多圈 unwrap：首帧直接初始化，之后按带符号最短角增量累积
        const float raw = _ctx.motor.leg[i]->get_current_position();
        _ctx.data.leg_raw_position[i] = raw;

        if (!_leg_fb_init[i])
        {
            _last_leg_raw[i] = raw;
            _leg_fb_init[i]  = true;
        }

        float delta = raw - _last_leg_raw[i];
        if (delta > PI)
        {
            delta -= 2.0f * PI;
        }
        else if (delta < -PI)
        {
            delta += 2.0f * PI;
        }

        _ctx.data.leg_theta_unwrap[i] += delta;
        _last_leg_raw[i] = raw;

        // 相对角（校准点为 0）：在反馈边界乘该腿唯一安装方向符号
        if (_ctx.data.leg_calibrated[i])
        {
            _ctx.data.current_leg_theta[i] =
                LEG_MOTOR_SIGN[i] *
                (_ctx.data.leg_theta_unwrap[i] - _ctx.data.leg_theta_cal[i]);
        }

        // 电机输出轴角速度：在反馈边界乘同一个安装方向符号
        _ctx.data.current_leg_radps[i] =
            LEG_MOTOR_SIGN[i] * _ctx.motor.leg[i]->get_current_rotate();
    }
}

// =========================================================
// 腿部机构算法辅助函数
// =========================================================

// 腿失能：输出和积分全部清零，重新使能时不会携带旧力矩或积分冲击。
void engineer_chassis_t::_leg_handle_disabled(int i)
{
    _ctx.data.target_leg_theta[i]       = 0.0f;
    _ctx.data.target_leg_radps[i]       = 0.0f;
    _ctx.data.out_leg_torque[i]         = 0.0f;
    _ctx.data.out_leg_gravity_torque[i] = 0.0f;
    _ctx.data.leg_alpha[i]              = 0.0f;
    _ctx.data.leg_gravity_valid[i]      = false;
    _ctx.data.leg_j_dynamic[i]          = 0.0f;
    _ctx.data.leg_tau_wall[i]           = 0.0f;
    _ctx.data.leg_tau_damping[i]        = 0.0f;
    _ctx.data.leg_ff_scale[i]           = 0.0f;
    _ctx.data.leg_tau_manual_pid[i]     = 0.0f;
    _ctx.data.leg_tau_attitude[i]       = 0.0f;
    _ctx.data.leg_control_mode[i]       = 0.0f;

    _leg_target_smooth[i]         = 0.0f;
    _leg_zero_interp_init[i]      = false;
    _leg_manual_map_active[i]     = false;
    _leg_manual_cmd_offset[i]     = 0.0f;
    _leg_last_logic_torque[i]     = 0.0f;

    if (_ctx.pid.leg_pos_pid[i]) _ctx.pid.leg_pos_pid[i]->clear();
    if (_ctx.pid.leg_vel_pid[i]) _ctx.pid.leg_vel_pid[i]->clear();
}

// 已使能但未校准：不输出力矩，等待右拨杆进入 UP 的边沿执行软件置零。
void engineer_chassis_t::_leg_handle_uncalibrated(int i)
{
    _ctx.data.out_leg_torque[i]         = 0.0f;
    _ctx.data.out_leg_gravity_torque[i] = 0.0f;
    _ctx.data.leg_alpha[i]              = 0.0f;
    _ctx.data.leg_gravity_valid[i]      = false;
    _ctx.data.leg_j_dynamic[i]          = 0.0f;
    _ctx.data.leg_tau_wall[i]           = 0.0f;
    _ctx.data.leg_tau_damping[i]        = 0.0f;
    _ctx.data.leg_ff_scale[i]           = 0.0f;
    _ctx.data.leg_tau_manual_pid[i]     = 0.0f;
    _ctx.data.leg_tau_attitude[i]       = 0.0f;
    _ctx.data.leg_control_mode[i]       = 0.0f;
    _leg_last_logic_torque[i]           = 0.0f;
}

// 任一有效腿控子状态进入瞬间：参考角从当前角起步并清双环积分，保证无扰切换。
void engineer_chassis_t::_leg_enter_active_mode(int i, float current_q)
{
    _leg_manual_map_active[i] = false;
    _leg_manual_cmd_offset[i] = 0.0f;
    _leg_zero_interp_init[i]  = false;
    _leg_target_smooth[i]     = current_q;

    if (_ctx.pid.leg_pos_pid[i]) _ctx.pid.leg_pos_pid[i]->clear();
    if (_ctx.pid.leg_vel_pid[i]) _ctx.pid.leg_vel_pid[i]->clear();
}

// 小坦巡航 VMC 重力链 + q 空间软墙；include_attitude=true 时再叠加 Pitch/Roll 映射力矩。
void engineer_chassis_t::_leg_update_vmc_feedforward(
    int i, float q, float qdot, float pitch,
    bool include_attitude, float f_pitch, float f_roll)
{
    float vmc_wall_tau = 0.0f;

    // 多项式有效窗外共用的下端制动墙；起步 q≈0 死区不做主动推腿墙。
    if (q < LEG_POLY_Q_VALID_MIN)
    {
        vmc_wall_tau =
            LEG_POLY_WALL_K * (LEG_POLY_Q_VALID_MIN - q) -
            LEG_POLY_WALL_D * qdot;
        vmc_wall_tau = fp32_constrain(
            vmc_wall_tau, 0.0f, LEG_POLY_WALL_MAX);
    }

    const float theta_raw  = LEG_Q_XT_SIGN * q + LEG_POLY_THETA_OFFSET;
    const float theta_eval = fp32_constrain(
        theta_raw, LEG_POLY_THETA_MIN, LEG_POLY_THETA_MAX);

    const leg_xt_vmc_result_t vmc = calc_leg_xiaotan_vmc(theta_eval, pitch);

    const bool q_window_ok =
        (q >= LEG_POLY_Q_VALID_MIN && q <= LEG_POLY_Q_VALID_MAX &&
         theta_raw >= LEG_POLY_THETA_MIN &&
         theta_raw <= LEG_POLY_THETA_MAX);

    // theta 帧重力矩按线性坐标变换映射回 q 帧；
    // 左右腿物理镜像统一留给发送边界的 LEG_MOTOR_SIGN。
    const float poly_gravity_q =
        LEG_Q_XT_SIGN * LEG_POLY_GRAVITY_SCALE * vmc.tau_gravity;

    const bool gravity_enable =
        LEG_VMC_GRAVITY_EN && vmc.valid && q_window_ok &&
        (fabsf(poly_gravity_q) > 1e-5f);

    const float gravity_tau = gravity_enable
        ? fp32_constrain(poly_gravity_q,
                         -LEG_POLY_GRAVITY_TAU_MAX,
                         LEG_POLY_GRAVITY_TAU_MAX)
        : 0.0f;

    // Pitch/Roll 虚拟力 -> 小坦 theta 帧力矩 -> 本车 q 逻辑帧力矩。
    // Roll 左右腿反号，与小坦 _leg_vmc() 保持一致；q->电机物理方向在发送层处理。
    float attitude_tau_q = 0.0f;
    if (include_attitude)
    {
        const float roll_force_i = (i == 0) ? f_roll : -f_roll;
        const float tau_attitude_xt =
            vmc.j_dynamic * (f_pitch + roll_force_i);
        attitude_tau_q = LEG_Q_XT_SIGN * tau_attitude_xt;
    }

    _ctx.data.leg_alpha[i]              = theta_eval;
    _ctx.data.leg_j_dynamic[i]          = vmc.j_dynamic;
    _ctx.data.leg_tau_wall[i]           = vmc_wall_tau;
    _ctx.data.leg_tau_attitude[i]       = attitude_tau_q;
    _ctx.data.leg_ff_scale[i]           = 1.0f;
    _ctx.data.out_leg_gravity_torque[i] = gravity_tau;
    _ctx.data.leg_gravity_valid[i]      = gravity_enable;
}

// 纯双环 PID 档明确撤掉所有 VMC/前馈观测量，避免 Live Watch 残留上一档数值。
void engineer_chassis_t::_leg_clear_feedforward(int i)
{
    _ctx.data.leg_alpha[i]              = 0.0f;
    _ctx.data.leg_j_dynamic[i]          = 0.0f;
    _ctx.data.out_leg_gravity_torque[i] = 0.0f;
    _ctx.data.leg_gravity_valid[i]      = false;
    _ctx.data.leg_tau_wall[i]           = 0.0f;
    _ctx.data.leg_tau_attitude[i]       = 0.0f;
    _ctx.data.leg_ff_scale[i]           = 0.0f;
}

// 左 MID：右摇杆下推增量映射；摇杆回中后冻结当前平滑目标，不自动跳回零点。
void engineer_chassis_t::_leg_update_manual_target(int i, float stick_cmd)
{
    _leg_zero_interp_init[i] = false;

    if (stick_cmd < -LEG_MANUAL_STICK_EPS)
    {
        // 第一次推摇杆时把当前平滑目标作为基准，避免中位命令 0 造成目标跳变。
        if (!_leg_manual_map_active[i])
        {
            _leg_manual_map_active[i] = true;
            _leg_manual_cmd_offset[i] =
                _leg_target_smooth[i] - stick_cmd;
        }

        float desired = stick_cmd + _leg_manual_cmd_offset[i];
        desired = fp32_constrain(desired, LEG_MANUAL_MIN, LEG_MANUAL_MAX);

        _leg_target_smooth[i] +=
            (desired - _leg_target_smooth[i]) * LEG_TARGET_LERP;
        _leg_target_smooth[i] = fp32_constrain(
            _leg_target_smooth[i], LEG_MANUAL_MIN, LEG_MANUAL_MAX);
    }
    else
    {
        _leg_manual_map_active[i] = false;
    }
}

// 左 DOWN：最终目标为校准零点，但位置环参考按最大速度逐步接近，避免目标阶跃。
void engineer_chassis_t::_leg_update_home_target(int i, float current_q)
{
    if (!_leg_zero_interp_init[i])
    {
        _leg_target_smooth[i]    = current_q;
        _leg_zero_interp_init[i] = true;
    }

    constexpr float home_goal = 0.0f;
    const float goal_error    = home_goal - _leg_target_smooth[i];
    const float max_step      = LEG_ZERO_INTERP_VMAX * LEG_CONTROL_DT_S;
    const float interp_step   =
        fp32_constrain(goal_error, -max_step, max_step);

    _leg_target_smooth[i] += interp_step;

    // 参考角进入小死区后直接吸附到最终目标；位置环继续负责保持。
    if (fabsf(home_goal - _leg_target_smooth[i]) <= LEG_ZERO_INTERP_EPS)
    {
        _leg_target_smooth[i] = home_goal;
    }

    _leg_manual_map_active[i] = false;
    _leg_manual_cmd_offset[i] = 0.0f;
}

// 位置环 -> 速度环；返回速度环 PID 输出的逻辑力矩。
float engineer_chassis_t::_leg_run_dual_pid(int i)
{
    _ctx.data.target_leg_theta[i] = _leg_target_smooth[i];
    _ctx.data.target_leg_radps[i] = 0.0f;

    if (_ctx.pid.leg_pos_pid[i])
    {
        _ctx.data.target_leg_radps[i] =
            _ctx.pid.leg_pos_pid[i]->calculate(
                _ctx.data.target_leg_theta[i],
                _ctx.data.current_leg_theta[i]);
    }

    float pid_tau = 0.0f;
    if (_ctx.pid.leg_vel_pid[i])
    {
        pid_tau = _ctx.pid.leg_vel_pid[i]->calculate(
            _ctx.data.target_leg_radps[i],
            _ctx.data.current_leg_radps[i]);
    }

    return pid_tau;
}

// 所有有效腿控模式共用：关节阻尼、总力矩限幅和 1ms 输出斜率限制。
void engineer_chassis_t::_leg_apply_common_output(
    int i, float raw_torque, float qdot)
{
    float tau_damping = -LEG_F_JOINT_DAMP_KD * qdot;
    tau_damping = fp32_constrain(
        tau_damping, -LEG_F_JOINT_DAMP_MAX, LEG_F_JOINT_DAMP_MAX);

    _ctx.data.leg_tau_damping[i] = tau_damping;
    raw_torque += tau_damping;
    raw_torque = fp32_constrain(raw_torque, -LEG_MAX_TORQUE, LEG_MAX_TORQUE);

    // 抑制模式切换、有效域跳变和保护造成的力矩阶跃。
    float delta = raw_torque - _leg_last_logic_torque[i];
    delta = fp32_constrain(
        delta, -LEG_F_TORQUE_SLEW_STEP, LEG_F_TORQUE_SLEW_STEP);

    const float output_torque = fp32_constrain(
        _leg_last_logic_torque[i] + delta,
        -LEG_MAX_TORQUE,
        LEG_MAX_TORQUE);

    _leg_last_logic_torque[i]     = output_torque;
    _ctx.data.out_leg_torque[i]   = output_torque;
}

// =========================================================
// _leg_control()：腿部 active 子 HFSM 协调入口
// ---------------------------------------------------------
// 各状态的 enter/execute/exit 拆分在 Chassis/fsm/leg/*.cpp；
// 这里只负责把使能、校准和左拨杆请求映射为目标子状态，并驱动 PYRo FSM。
// 顶层 active/passive FSM 与 Chassis/fsm 下两个顶层状态文件保持不变。
// =========================================================
void engineer_chassis_t::_leg_control()
{
    const bool vmc_request        = _ctx.cmd->track_run;
    const bool pid_gravity_request = _ctx.cmd->leg.home_to_zero;

    state_t<owner> *target_state = &_leg_state_pid_home;

    if (!_ctx.cmd->leg.en)
    {
        target_state = &_leg_state_disabled;
    }
    else if (!_ctx.data.leg_calibrated[0] || !_ctx.data.leg_calibrated[1])
    {
        // 两腿共用同一左拨杆模式；任一条腿未完成校准时先进入安全等待态。
        target_state = &_leg_state_uncalibrated;
    }
    else if (vmc_request)
    {
        target_state = &_leg_state_vmc;
    }
    else if (pid_gravity_request)
    {
        target_state = &_leg_state_pid_gravity;
    }
    else
    {
        target_state = &_leg_state_pid_home;
    }

    _leg_fsm.change_state(target_state);
    _leg_fsm.execute(this);
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
