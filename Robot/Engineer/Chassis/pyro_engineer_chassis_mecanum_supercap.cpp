/**
 * @file pyro_engineer_chassis_mecanum_supercap.cpp
 * @brief 超级电容板命令组装、使能状态机与周期发送；当前额外功率仅服务麦轮功控。
 *
 * 协议帧、CRC、DMA 和反馈解包由 supercap_drv_t 负责；本文件只保留应用层状态。
 */
#include "pyro_engineer_chassis.h"
#include <cmath>

namespace pyro
{

// =========================================================
void engineer_chassis_t::_supercap_control()
{
#ifdef SUPERCAP_UART
    constexpr uint32_t enable_delay_ticks     = 1000U; // 1 kHz 控制任务：约 1 s 后使能
    constexpr uint32_t refresh_ticks          = 10U;   // 10 ms 周期保活发送
    constexpr uint32_t disable_debounce_ticks = 30U;   // 失能去抖 30 ms
    constexpr uint8_t  tx_buffer_limit_energy_j = 60U; // 给电容板的缓冲能量上限 byte
    constexpr float    tx_byte_max_f          = 255.0f;

    auto &cap = _ctx.supercap;

    supercap_drv_t *supercap = supercap_drv_t::get_instance();
    const bool cap_online    = supercap->check_online();

    const bool chassis_armed =
        (_ctx.cmd != nullptr) &&
        (_ctx.cmd->mode == cmd_base_t::mode_t::ACTIVE);

    supercap_drv_t::chassis_cmd_t tx_cmd{};

    // power_referee 是裁判系统实测底盘功率；当前工程未接裁判串口，固定填 0。
    tx_cmd.power_referee              = 0U;
    tx_cmd.power_buffer_limit_referee = tx_buffer_limit_energy_j;
    tx_cmd.kill_chassis_user          = 0U;
    tx_cmd.speed_up_user_now          = 0U;

    if (chassis_armed)
    {
        const float tx_power_limit_w = fminf(
            fmaxf(_ctx.data.wheel_power_limit_w, 0.0f), tx_byte_max_f);
        const float tx_buffer_energy_j = fminf(
            fmaxf(_ctx.data.wheel_power_buffer_energy_j, 0.0f), tx_byte_max_f);

        tx_cmd.power_limit_referee =
            static_cast<uint8_t>(lroundf(tx_power_limit_w));
        tx_cmd.power_buffer_referee =
            static_cast<uint8_t>(lroundf(tx_buffer_energy_j));
    }

    const bool enable_request = cap_online && chassis_armed;

    if (enable_request)
    {
        cap.disable_timer = 0U;

        if (!cap.enabled)
        {
            cap.refresh_timer = 0U;
            if (++cap.enable_timer >= enable_delay_ticks)
            {
                cap.enable_timer = 0U;
                cap.enabled      = true;
                tx_cmd.use_cap   = 1U;

                (void)supercap->send_cmd(tx_cmd);
            }
        }
        else if (++cap.refresh_timer >= refresh_ticks)
        {
            cap.refresh_timer = 0U;
            tx_cmd.use_cap    = 1U;

            (void)supercap->send_cmd(tx_cmd);
        }
    }
    else
    {
        cap.enable_timer  = 0U;
        cap.refresh_timer = 0U;

        if (cap.enabled)
        {
            if (++cap.disable_timer >= disable_debounce_ticks)
            {
                cap.disable_timer = 0U;
                cap.enabled       = false;
                tx_cmd.use_cap    = 0U;

                (void)supercap->send_cmd(tx_cmd);
            }
        }
        else
        {
            cap.disable_timer = 0U;
        }
    }
#else
    _ctx.supercap = {};
#endif
}

} // namespace pyro
