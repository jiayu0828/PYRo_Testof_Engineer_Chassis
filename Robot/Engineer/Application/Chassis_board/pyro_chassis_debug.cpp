/**
 * @file pyro_chassis_debug.cpp
 * @brief 底盘调试任务（JustFloat 波形，UART1 / 921600）
 *
 * 用途：通过 JCOM（VOFA+ JustFloat 协议）向上位机实时导出履带、腿、IMU 数据，
 *      用于 PID 标定与波形观察。
 *
 * 14 个通道（顺序）：
 *   履带 L(目标RPM,当前RPM,力矩)  履带 R(目标RPM,当前RPM,力矩)
 *   腿 L(目标角速度,当前角速度,力矩)  腿 R(目标角速度,当前角速度,力矩)
 *   IMU(pitch, pitch_radps)
 * 串口：UART1（DEBUG_UART，CubeMX 默认 921600 8N1），本任务只发不收。
 */

#include "pyro_engineer_chassis.h"
#include "pyro_bsp_uart.h"

#include "FreeRTOS.h"
#include "task.h"
#include <cstring>

using namespace pyro;

/* ==================== 配置 ==================== */
static constexpr uint8_t  DEBUG_VAR_COUNT = 14;
static constexpr uint32_t DEBUG_PERIOD_MS = 10;

/* ==================== JustFloat 协议帧尾 ==================== */
static const uint8_t JCOM_FRAME_TAIL[4] = {0x00, 0x00, 0x80, 0x7F};

/* ==================== 内部变量 ==================== */
static uart_drv_t         *s_debug_uart = nullptr;
static engineer_chassis_t *s_chassis    = nullptr;

static uint8_t s_send_buf[DEBUG_VAR_COUNT * sizeof(float) + sizeof(JCOM_FRAME_TAIL)];

/* ==================== 发送 JustFloat 帧 ==================== */
static void jcom_send(const float *vars)
{
    uint8_t offset = 0;
    memcpy(s_send_buf + offset, vars, sizeof(float) * DEBUG_VAR_COUNT);
    offset += sizeof(float) * DEBUG_VAR_COUNT;
    memcpy(s_send_buf + offset, JCOM_FRAME_TAIL, sizeof(JCOM_FRAME_TAIL));
    offset += sizeof(JCOM_FRAME_TAIL);

    if (s_debug_uart)
    {
        s_debug_uart->write(s_send_buf, offset, 100);
    }
}

/* ==================== 主任务 ==================== */
static void debug_thread(void *argument)
{
    (void)argument;

    vTaskDelay(pdMS_TO_TICKS(100));
    s_chassis    = engineer_chassis_t::instance();
    s_debug_uart = &bsp_uart::get_uart1();

    float vars[DEBUG_VAR_COUNT];

    while (true)
    {
        if (s_chassis)
        {
            const auto &d = s_chassis->get_ctx().data;

            // 履带 L/R：目标 RPM、当前 RPM、力矩
            for (int i = 0; i < 2; i++)
            {
                vars[i * 3 + 0] = d.target_track_rpm[i];
                vars[i * 3 + 1] = d.current_track_rpm[i];
                vars[i * 3 + 2] = d.out_track_torque[i];
            }
            // 腿 L/R：目标角速度、当前角速度、力矩
            for (int i = 0; i < 2; i++)
            {
                vars[6 + i * 3 + 0] = d.target_leg_radps[i];
                vars[6 + i * 3 + 1] = d.current_leg_radps[i];
                vars[6 + i * 3 + 2] = d.out_leg_torque[i];
            }
            // IMU：pitch、pitch 角速度
            vars[12] = d.imu_pitch;
            vars[13] = d.imu_pitch_radps;

            jcom_send(vars);
        }

        vTaskDelay(pdMS_TO_TICKS(DEBUG_PERIOD_MS));
    }
}

/* ==================== 对外入口（init_thread 调用） ==================== */
extern "C" void chassis_debug_init()
{
    xTaskCreate(debug_thread, "chassis_debug", 512, nullptr,
                configMAX_PRIORITIES - 4, nullptr);
}
