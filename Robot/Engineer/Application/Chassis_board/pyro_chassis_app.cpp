/**
 * @file pyro_chassis_app.cpp
 * @brief 工程车底盘 Application 层
 *
 * 只负责：创建依赖 → 注入模块 → 启动模块。
 * 遥控器命令由 Communication 层的 DR16 任务通过 set_command() 下发。
 */
extern "C" {
#include "FreeRTOS.h"
#include "task.h"
}

#include "pyro_engineer_chassis.h"
#include "pyro_dji_motor_drv.h"
#include "pyro_algo_pid.h"
#include "pyro_bsp_can.h"

using namespace pyro;

/* ====================== 全局指针 ====================== */
static engineer_chassis_t *s_chassis = nullptr;
static engineer_deps_t    *s_deps    = nullptr;


/* ====================== 依赖初始化 ====================== */
static void deps_init()
{
    s_deps = new engineer_deps_t();

    /* ---------- 4 个麦轮电机（统一 CAN3，id1~4） ---------- */
    // 数组索引：[0]FL 前左, [1]FR 前右, [2]BL 后左, [3]BR 后右
    s_deps->motor_deps.mecanum[0] =
        new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_1, bsp_can::can3); // FL
    s_deps->motor_deps.mecanum[1] =
        new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_2, bsp_can::can3); // FR
    s_deps->motor_deps.mecanum[2] =
        new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_3, bsp_can::can3); // BL
    s_deps->motor_deps.mecanum[3] =
        new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_4, bsp_can::can3); // BR

    /* ---------- 麦轮速度环 PID ---------- */
    for (int i = 0; i < 4; i++)
    {
        s_deps->pid_deps.mecanum_pid[i] =
            new pid_t(0.3f, 0.0008f, 0.0002f,
                      1.0f, 10.0f, 20, 1, 10, 1, 4);
    }
}

/* ====================== 对外初始化入口 ====================== */
extern "C" void chassis_app_init()
{
    // 1. 创建所有依赖（电机、PID）
    deps_init();

    // 2. 获取底盘单例
    s_chassis = engineer_chassis_t::instance();

    // 3. 注入依赖
    s_chassis->configure(*s_deps);

    // 4. 启动模块（内部调用 _init 并创建模块控制任务）
    s_chassis->start();
}
