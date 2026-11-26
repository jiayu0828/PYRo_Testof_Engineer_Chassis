/**
 * @file pyro_chassis_remote.cpp
 * @brief 工程车底盘 Communication 层（DR16 遥控器）
 *
 * 独立任务周期读取 DR16，转换为 engineer_cmd_t 后通过 set_command() 下发。
 *
 * 拨杆分工：
 *   右拨杆 DOWN：麦轮/履带/腿全部失能；MID：4 个麦轮使能；
 *             UP：麦轮、履带和腿使能，进入瞬间校准腿部零点一次。
 *   左拨杆 UP：履带按固定 RPM 恒速旋转，同时腿部进入 VMC（重力/Pitch/Roll 补偿）；
 *             MID：履带目标 0，腿部右摇杆直接给力矩，并叠加重力补偿；
 *             DOWN：履带目标 0，腿部纯双环 PID 限速返回校准零点。
 *   右拨杆 UP 仍负责履带/腿电机使能；右拨杆非 UP 时即使左拨杆 UP 也不会输出履带力矩。
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
static sw_pos_t            s_last_right_sw = sw_pos_t::UNKNOWN;

// 摇杆死区：绝对值小于阈值则置零
static float apply_deadzone(float value)
{
    if (value > STICK_DEADZONE || value < -STICK_DEADZONE)
    {
        return value;
    }
    return 0.0f;
}

// 遥控器数据 -> 底盘命令
static void rc_to_cmd()
{
    const auto &rc = rc_drv_t::read();

    const auto right_pos = rc.switches.right.current_pos;
    const auto left_pos  = rc.switches.left.current_pos;

    // 麦轮行走速度（仅在右拨杆 MID/UP 使能时实际输出）
    s_cmd.chassis.vx = apply_deadzone(rc.axes.ly) * MAX_CHASSIS_VX;
    s_cmd.chassis.vy = -apply_deadzone(rc.axes.lx) * MAX_CHASSIS_VY;
    s_cmd.chassis.wz = -apply_deadzone(rc.axes.rx) * MAX_CHASSIS_WZ;

    // 右拨杆三档：决定各电机使能范围
    const bool right_up  = (right_pos == sw_pos_t::UP);
    const bool right_mid = (right_pos == sw_pos_t::MID);
    s_cmd.mecanum_en = (right_up || right_mid);
    s_cmd.track_en   = right_up;
    s_cmd.leg.en     = right_up;

    // 进入 UP 的边沿：校准腿部零点一次（非每帧）
    s_cmd.leg.calibrate = (s_last_right_sw != sw_pos_t::UP && right_up);
    s_last_right_sw     = right_pos;

    // 左拨杆三档统一映射为一个枚举，避免历史 bool 命名和非法组合。
    if (left_pos == sw_pos_t::UP)
    {
        s_cmd.left_switch = left_switch_cmd_t::UP;
    }
    else if (left_pos == sw_pos_t::MID)
    {
        s_cmd.left_switch = left_switch_cmd_t::MID;
    }
    else
    {
        s_cmd.left_switch = left_switch_cmd_t::DOWN;
    }

    // 左拨杆 MID：右摇杆下推直接映射为 q 帧负向力矩；UP/DOWN 不使用该量。
    const float ry      = apply_deadzone(rc.axes.ry);
    const float ry_down = (ry < 0.0f) ? ry : 0.0f;
    s_cmd.leg.manual_torque = ry_down * LEG_MID_MANUAL_TORQUE_MAX;

    // 顶层模式：右拨杆 DOWN 或遥控器掉线 -> PASSIVE，否则 ACTIVE
    if (right_pos != sw_pos_t::DOWN &&
        dr16_drv_t::instance().check_online())
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
// 辅助臂，主臂，发射模块 几块板子， 
// UI 问题 
// 功率 相关问题
// 底盘自动 上单级台阶 3s   双级台阶    5S   
// 对接问题 ， 