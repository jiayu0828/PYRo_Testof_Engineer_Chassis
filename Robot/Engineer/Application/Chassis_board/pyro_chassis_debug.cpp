/**
 * @file pyro_chassis_debug.cpp
 * @brief Engineer 底盘调试任务：后腿 VMC/力矩与 IMU 姿态 JustFloat 输出（UART1）
 *
 * JustFloat 固定 12 通道：
 *   CH1  左腿相对角 current_leg_theta (rad)
 *   CH2  左腿 VMC 多项式角 theta_XT (rad)，负值表示重力链无效
 *   CH3  左腿动态雅可比 j_dynamic
 *   CH4  左腿重力力矩命令（电机坐标系，Nm）
 *   CH5  左腿电机实测力矩（Nm）
 *   CH6  右腿相对角 current_leg_theta (rad)
 *   CH7  右腿 VMC 多项式角 theta_XT (rad)，负值表示重力链无效
 *   CH8  右腿动态雅可比 j_dynamic
 *   CH9  右腿重力力矩命令（电机坐标系，Nm）
 *   CH10 右腿电机实测力矩（Nm）
 *   CH11 IMU 滤波 pitch（含安装偏置修正，rad）
 *   CH12 IMU 滤波 roll（含安装偏置修正，rad）
 *
 * 更多中间量保留在 engineer_data_ctx_t，供 Live Watch 查看：
 *   raw_position / theta_unwrap / target_theta / target_radps /
 *   tau_manual_pid / tau_attitude / tau_wall / tau_damping / control_mode。
 */

#include "pyro_engineer_chassis.h"
#include "pyro_bsp_uart.h"

#include "FreeRTOS.h"
#include "task.h"
#include <cstring>

using namespace pyro;

static constexpr uint8_t  DEBUG_VAR_COUNT = 12;
static constexpr uint32_t DEBUG_PERIOD_MS = 10;
static const uint8_t JCOM_FRAME_TAIL[4] = {0x00, 0x00, 0x80, 0x7F};

static uart_drv_t         *s_debug_uart = nullptr;
static engineer_chassis_t *s_chassis    = nullptr;
static uint8_t s_send_buf[DEBUG_VAR_COUNT * sizeof(float) + sizeof(JCOM_FRAME_TAIL)];

// 不占 JustFloat 通道；可在调试器中观察，用于定位静默丢帧和 UART 写入失败
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

            for (int i = 0; i < 2; i++)
            {
                const int base = i * 5;
                vars[base + 0] = d.current_leg_theta[i];
                // alpha 实际始终为正；用负号携带“当前重力链无效”标志
                vars[base + 1] = d.leg_gravity_valid[i]
                                     ? d.leg_alpha[i]
                                     : -d.leg_alpha[i];
                vars[base + 2] = d.leg_j_dynamic[i];
                // 调试通道按电机坐标系显示；发送边界同样使用 LEG_MOTOR_SIGN。
                vars[base + 3] = LEG_MOTOR_SIGN[i] *
                                 d.out_leg_gravity_torque[i];
                vars[base + 4] = ctx.motor.leg[i]
                                     ? ctx.motor.leg[i]->get_current_torque()
                                     : 0.0f;
            }
            vars[10] = d.imu_pitch;
            vars[11] = d.imu_roll;

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
