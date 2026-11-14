/**
 * @file pyro_chassis_remote.cpp
 * @brief 工程车底盘 Communication 层（DR16 遥控器）
 *
 * 独立任务周期读取 DR16，转换为 engineer_cmd_t 后通过 set_command() 下发。
 * 后续若改为板间通信/自动任务，只需替换本层，模块内部无需改动。
 */
extern "C" {
#include "FreeRTOS.h"
#include "task.h"
}

#include "pyro_engineer_chassis.h"
#include "pyro_dr16_rc_drv.h"

using namespace pyro;

/* ====================== 摇杆死区（归一化量纲） ====================== */
static constexpr float STICK_DEADZONE = 0.05f;

static engineer_chassis_t *s_chassis = nullptr;
static engineer_cmd_t      s_cmd{};

// 摇杆死区：绝对值小于阈值则置零
static float apply_deadzone(float value)
{
    if (value > STICK_DEADZONE || value < -STICK_DEADZONE)
    {
        return value;
    }
    return 0.0f;
}

// 使能拨杆读取。
//右拨杆本身，向上使能
static bool read_enable_switch(const virtual_rc_t &rc)
{
    return rc.switches.right.current_pos == sw_pos_t::UP;
}

// 遥控器数据 -> 底盘命令
static void rc_to_cmd()
{
    const auto &rc = rc_drv_t::read();

    // 符号为常规约定，需结合实物实测确认
    s_cmd.chassis.vx = apply_deadzone(rc.axes.ly) * MAX_CHASSIS_VX;
    s_cmd.chassis.vy = -apply_deadzone(rc.axes.lx) * MAX_CHASSIS_VY;
    s_cmd.chassis.wz = -apply_deadzone(rc.axes.rx) * MAX_CHASSIS_WZ;

    if (read_enable_switch(rc) && dr16_drv_t::instance().check_online())
    {
        s_cmd.mode = cmd_base_t::mode_t::ACTIVE;
    }
    else
    {
        s_cmd.mode = cmd_base_t::mode_t::PASSIVE;
    }
}

// 遥控任务：周期读取并下发命令
extern "C" void chassis_remote_thread(void *argument)
{
    (void)argument;
    while (true)
    {
        rc_to_cmd();
        s_chassis->set_command(s_cmd);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// 通信层初始化入口（创建遥控任务）
extern "C" void chassis_remote_init()
{
    s_chassis = engineer_chassis_t::instance();

    xTaskCreate(chassis_remote_thread, "chassis_rc", 128, nullptr,
                configMAX_PRIORITIES - 2, nullptr);
}
