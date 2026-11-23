/**
 * @file pyro_engineer_chassis_mecanum_supercap.cpp
 * @brief 超级电容板运行时反馈外的命令组装、使能状态机与周期发送；当前额外功率仅服务麦轮功控。
 *
 * 方案 A 文件拆分：函数定义从 pyro_engineer_chassis.cpp 原样剪切；
 * 类声明仍位于 pyro_engineer_chassis.h，控制逻辑不在纯搬运提交中改变。
 */
#include "pyro_engineer_chassis.h"
#include <cmath>

namespace pyro
{

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

} // namespace pyro
