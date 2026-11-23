/**
 * @file pyro_chassis_debug.cpp
 * @brief Engineer 底盘调试任务：超级电容 JustFloat 观测输出（UART1）
 *
 * 当前调试通道固定为 4 个 float（10 ms/帧，小端）：
 *   CH1  supercap_online_f   电容板在线状态，0.0f/1.0f
 *   CH2  cap_voltage_V       电容电压，按当前固件 0.01 V/LSB 口径换算为 V
 *   CH3  cap_power_raw       电容板回传 cap_power_cap 原始值（驱动未缩放）
 *   CH4  chassis_power_raw   电容板回传 chassis_power_cap 原始值（驱动未缩放）
 *
 * 超级电容反馈来自 UART7；本任务只通过 UART1 发送观测数据，不参与控制闭环。
 */

#include "pyro_engineer_chassis.h"
#include "pyro_bsp_uart.h"

#include "FreeRTOS.h"
#include "task.h"
#include <cstring>

using namespace pyro;

static constexpr uint8_t  DEBUG_VAR_COUNT  = 4;
static constexpr uint32_t DEBUG_PERIOD_MS  = 10;

static const uint8_t JCOM_FRAME_TAIL[4] = {0x00, 0x00, 0x80, 0x7F};

static uart_drv_t         *s_debug_uart = nullptr;
static engineer_chassis_t *s_chassis    = nullptr;
static uint8_t s_send_buf[DEBUG_VAR_COUNT * sizeof(float) + sizeof(JCOM_FRAME_TAIL)];

// 不占 JustFloat 通道；可在调试器中观察，用于判断 UART 写入是否失败。
static volatile uint32_t s_debug_frame_seq      = 0;
static volatile uint32_t s_debug_uart_error_cnt = 0;

static void jcom_send(const float *vars)
{
    uint8_t offset = 0;
    memcpy(s_send_buf + offset, vars, sizeof(float) * DEBUG_VAR_COUNT);
    offset += sizeof(float) * DEBUG_VAR_COUNT;
    memcpy(s_send_buf + offset, JCOM_FRAME_TAIL, sizeof(JCOM_FRAME_TAIL));
    offset += sizeof(JCOM_FRAME_TAIL);

    ++s_debug_frame_seq;
    if (s_debug_uart &&
        s_debug_uart->write(s_send_buf, offset, 100) != PYRO_OK)
    {
        ++s_debug_uart_error_cnt;
    }
}

static void debug_thread(void *argument)
{
    (void)argument;

    vTaskDelay(pdMS_TO_TICKS(100));
    s_chassis    = engineer_chassis_t::instance();
    s_debug_uart = &bsp_uart::get_uart1();

    float vars[DEBUG_VAR_COUNT]{};

    while (true)
    {
        if (s_chassis)
        {
            const auto &ctx = s_chassis->get_ctx();
            const auto &d   = ctx.data;

            vars[0] = d.power_meter_w ;
            vars[1] = d.supercap_voltage_cv * 0.01f;
            vars[2] = d.supercap_extra_power_w;
            vars[3] = d.supercap_chassis_power_raw;

            jcom_send(vars);
        }

        vTaskDelay(pdMS_TO_TICKS(DEBUG_PERIOD_MS));
    }
}

extern "C" void chassis_debug_init()
{
    xTaskCreate(debug_thread, "chassis_debug", 512, nullptr,
                configMAX_PRIORITIES - 4, nullptr);
}
