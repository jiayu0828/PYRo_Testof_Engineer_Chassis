/**
 * @file pyro_chassis_debug.cpp
 * @brief 底盘速度环调试任务（JustFloat 波形，UART1 / 921600）
 *
 * 用途：通过 JCOM（VOFA+ JustFloat 协议）向上位机实时导出 2 条履带
 *      速度环的「目标转速 / 当前转速 / 输出力矩」，用于 PID 标定与波形观察。
 *
 * 数据：每条履带 3 个 float，共 6 个通道，顺序为
 *      L(target,current,torque) R(target,current,torque)
 * 串口：UART1（DEBUG_UART，CubeMX 默认 921600 8N1），本任务只发不收。
 * 发送：阻塞轮询发送，避免 TX 未发完返回 BUSY 而丢点。
 */

#include "pyro_engineer_chassis.h"
#include "pyro_bsp_uart.h"

#include "FreeRTOS.h"
#include "task.h"
#include <cstring>

using namespace pyro;

/* ==================== 配置 ==================== */
// 2 条履带 × 3 个量（目标 RPM / 当前 RPM / 力矩 Nm）
static constexpr uint8_t  DEBUG_VAR_COUNT = 6;
static constexpr uint32_t DEBUG_PERIOD_MS = 10;

/* ==================== JustFloat 协议帧尾 ==================== */
static const uint8_t JCOM_FRAME_TAIL[4] = {0x00, 0x00, 0x80, 0x7F};

/* ==================== 内部变量 ==================== */
static uart_drv_t         *s_debug_uart = nullptr;
static engineer_chassis_t *s_chassis    = nullptr;

// 发送缓冲：6 个 float + 帧尾（阻塞发送期间保持有效，用静态缓冲）
static uint8_t s_send_buf[DEBUG_VAR_COUNT * sizeof(float) + sizeof(JCOM_FRAME_TAIL)];

/* ==================== 发送 JustFloat 帧 ==================== */
static void jcom_send(const float *vars)
{
    uint8_t offset = 0;
    memcpy(s_send_buf + offset, vars, sizeof(float) * DEBUG_VAR_COUNT);
    offset += sizeof(float) * DEBUG_VAR_COUNT;
    memcpy(s_send_buf + offset, JCOM_FRAME_TAIL, sizeof(JCOM_FRAME_TAIL));
    offset += sizeof(JCOM_FRAME_TAIL);

    // 阻塞轮询发送（100ms 超时），不丢点
    if (s_debug_uart)
    {
        s_debug_uart->write(s_send_buf, offset, 100);
    }
}

/* ==================== 主任务 ==================== */
static void debug_thread(void *argument)
{
    (void)argument;

    // 等底盘模块启动（chassis_app_init 已先于本任务调用）
    vTaskDelay(pdMS_TO_TICKS(100));
    s_chassis = engineer_chassis_t::instance();

    // 调试串口：UART1（DEBUG_UART，CubeMX 默认 921600 8N1）
    s_debug_uart = &bsp_uart::get_uart1();

    float vars[DEBUG_VAR_COUNT];

    while (true)
    {
        if (s_chassis)
        {
            const auto &ctx = s_chassis->get_ctx();

            // 每条履带依次填：目标 RPM、当前 RPM、输出力矩 Nm
            for (int i = 0; i < 2; i++)
            {
                vars[i * 3 + 0] = ctx.data.target_track_rpm[i];
                vars[i * 3 + 1] = ctx.data.current_track_rpm[i];
                vars[i * 3 + 2] = ctx.data.out_track_torque[i];
            }

            jcom_send(vars);
        }

        vTaskDelay(pdMS_TO_TICKS(DEBUG_PERIOD_MS));
    }
}

/* ==================== 对外入口（init_thread 调用） ==================== */
extern "C" void chassis_debug_init()
{
    xTaskCreate(debug_thread,
                "chassis_debug",
                512,
                nullptr,
                configMAX_PRIORITIES - 4,
                nullptr);
}
