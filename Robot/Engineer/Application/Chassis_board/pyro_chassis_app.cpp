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
#include "pyro_dm_motor_drv.h"
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
                      1.0f, 10.0f, 20.0f, 1, 0, 0, 0);
    }

    /* ---------- 履带电机（达妙 4310，CAN2，ID 参考小坦 lift） ---------- */
    // 索引 [0]=L 左履带, [1]=R 右履带
    s_deps->motor_deps.track[0] =
        new dm_motor_drv_t(0x11, 0x21, bsp_can::can2); 
    s_deps->motor_deps.track[1] =
        new dm_motor_drv_t(0x12, 0x22, bsp_can::can2); 

    // 达妙解析范围（参考小坦），并清零内部 kp/kd 以使用纯力矩模式
    for (int i = 0; i < 2; i++)
    {
        auto *dm = static_cast<dm_motor_drv_t *>(s_deps->motor_deps.track[i]);
        dm->set_position_range(-PI, PI);
        dm->set_rotate_range(-52.0f, 52.0f);
        dm->set_torque_range(-27.0f, 27.0f);
        dm->set_runtime_kp(0.0f);
        dm->set_runtime_kd(0.0f);
    }

    /* ---------- 履带速度环 PID（参数参考小坦 lift_vel_pid，max_out=27 匹配力矩范围） ---------- */
    for (int i = 0; i < 2; i++)
    {
        s_deps->pid_deps.track_pid[i] =
            new pid_t(0.2f, 0.00f, 0.0f, 0.5f, 27.0f, pid_t::INTEGRAL_LIMIT);
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
