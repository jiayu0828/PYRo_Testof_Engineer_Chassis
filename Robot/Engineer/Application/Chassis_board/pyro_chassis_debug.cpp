/**
 * @file pyro_chassis_debug.cpp
 * @brief Engineer 底盘调试任务：左前轮功率模型标定 JustFloat 输出（UART1）
 *
 * 当前标定通道固定为左前轮 FL（mecanum 数组索引 0，CAN3 ID1）：
 *   CH1  0x212 功率计实测功率 power（W）
 *   CH2  左前轮逻辑力矩指令 torque（N·m，PID 输出；发送边界再乘电机方向）
 *   CH3  左前轮轮轴转速 rpm（已换算到轮轴并应用 MECANUM_MOTOR_SIGN）
 *   CH4  左前轮 M3508 电调温度 temp（℃）
 *
 * 功率计当前挂在腿电机所在 CAN1，CAN ID=0x212。
 * 本任务只通过 UART1 发送标定数据，不参与功率闭环控制。
 */

#include "pyro_engineer_chassis.h"
#include "pyro_bsp_uart.h"

#include "FreeRTOS.h"
#include "task.h"
#include <cstring>

using namespace pyro;

static constexpr uint8_t  DEBUG_VAR_COUNT  = 4;
static constexpr uint32_t DEBUG_PERIOD_MS  = 10;
static constexpr uint8_t  CALIB_WHEEL_IDX  = 0; // 左前轮 FL

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

            vars[0] = d.power_meter_w;
            vars[1] = d.out_wheel_torque[CALIB_WHEEL_IDX];
            vars[2] = d.current_wheel_rpm[CALIB_WHEEL_IDX];
            vars[3] = d.current_wheel_temp[CALIB_WHEEL_IDX];

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
