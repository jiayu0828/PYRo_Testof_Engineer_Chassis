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
#include "pyro_powermeter.h"

using namespace pyro;

/* ====================== 标定设备配置 ====================== */
// 当前测试接线：0x212 功率计的 CAN 通信线临时接在腿电机所在 CAN1。
// 功率输入回路接左前轮 M3508 电调；CAN 只负责读取功率计测量帧。
static constexpr uint32_t              POWER_METER_CAN_ID = 0x212U;
static constexpr bsp_can::which_can    POWER_METER_CAN    = bsp_can::can1;

/* ====================== 全局指针 ====================== */
static engineer_chassis_t *s_chassis = nullptr;
static engineer_deps_t    *s_deps    = nullptr;


/* ====================== 依赖初始化 ====================== */
static void deps_init()
{
    s_deps = new engineer_deps_t();

    /* ---------- 4 个麦轮电机（统一 CAN3，id1~4） ---------- */
    // 数组索引：[0]FL 前左, [1]FR 前右, [2]BL 后左, [3]BR 后右。
    // 安装方向唯一配置：engineer_config.h 的 MECANUM_MOTOR_SIGN，创建顺序必须一致。
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
        // 参数顺序：Kp, Ki, Kd, 积分限幅, 输出限幅,
        //          输出滤波截止/阶数, 微分滤波截止/阶数, OLS 阶数。
        s_deps->pid_deps.mecanum_pid[i] =
            new pid_t(0.3f, 0.0008f, 0.0002f,
                      1.0f, 10.0f,
                      20.0f, 1U,
                      0.0f, 0U, 0U);
    }

    /* ---------- 履带电机（达妙 4310，CAN2） ---------- */
    // 索引 [0]=L 左履带, [1]=R 右履带；方向唯一配置：TRACK_MOTOR_SIGN。
    s_deps->motor_deps.track[0] =
        new dm_motor_drv_t(0x11, 0x21, bsp_can::can2);
    s_deps->motor_deps.track[1] =
        new dm_motor_drv_t(0x12, 0x22, bsp_can::can2);

    // 达妙解析范围，并清零内部 kp/kd 以使用纯力矩模式
    for (int i = 0; i < 2; i++)
    {
        auto *dm = static_cast<dm_motor_drv_t *>(s_deps->motor_deps.track[i]);
        dm->set_position_range(-PI, PI);
        dm->set_rotate_range(-52.0f, 52.0f);
        dm->set_torque_range(-27.0f, 27.0f);
        dm->set_runtime_kp(0.0f);
        dm->set_runtime_kd(0.0f);
    }

    /* ---------- 履带速度环 PID（max_out=27 匹配力矩范围） ---------- */
    for (int i = 0; i < 2; i++)
    {
        s_deps->pid_deps.track_pid[i] =
            new pid_t(0.2f, 0.0f, 0.0f, 0.5f, 27.0f, pid_t::INTEGRAL_LIMIT);
    }

    /* ---------- 腿电机（达妙，CAN1；方向唯一配置：LEG_MOTOR_SIGN） ---------- */
    s_deps->motor_deps.leg[0] =
        new dm_motor_drv_t(0x31, 0x41, bsp_can::can1);
    s_deps->motor_deps.leg[1] =
        new dm_motor_drv_t(0x32, 0x42, bsp_can::can1);

    // 达妙解析范围，纯力矩模式（内部 kp/kd=0）
    for (int i = 0; i < 2; i++)
    {
        auto *dm = static_cast<dm_motor_drv_t *>(s_deps->motor_deps.leg[i]);
        dm->set_position_range(-PI, PI);
        dm->set_rotate_range(-LEG_MAX_SPEED, LEG_MAX_SPEED);
        dm->set_torque_range(-LEG_MAX_TORQUE, LEG_MAX_TORQUE);
        dm->set_runtime_kp(0.0f);
        dm->set_runtime_kd(0.0f);
    }

    /* ---------- 0x212 CAN 功率计（只读标定，不接入功率闭环） ---------- */
    s_deps->powermeter = new powermeter_drv_t(POWER_METER_CAN_ID, POWER_METER_CAN);
    if (s_deps->powermeter->init() != PYRO_OK)
    {
        delete s_deps->powermeter;
        s_deps->powermeter = nullptr;
    }

    /* ---------- 腿双环 PID  ---------- */
    for (int i = 0; i < 2; i++)
    {
        // 外环位置环：角度误差 -> 目标角速度 rad/s；积分限幅/输出限幅均为 5.655 rad/s
        s_deps->pid_deps.leg_pos_pid[i] =
            new pid_t(8.0f, 0.0f, 0.0f, 0.0f, 5.655f,
                      pid_t::INTEGRAL_LIMIT);//降低最大速度
        // 内环速度环：速度误差 -> 力矩 N·m；积分限幅/输出限幅均为 27 N·m
        s_deps->pid_deps.leg_vel_pid[i] =
            new pid_t(1.0f, 0.5f, 0.0f, 27.0f, 27.0f,
                      pid_t::INTEGRAL_LIMIT);
    }

    /* ---------- 腿部 VMC 姿态力 PID（仅左拨杆 UP 投入，输出任务空间虚拟力 N） ---------- */
    // 参数顺序：Kp, Ki, Kd, 积分限幅, 输出限幅,
    //          输出滤波截止/阶数, 微分滤波截止/阶数, OLS 阶数, 改善选项。
    s_deps->pid_deps.pitch_pid = new pid_t(
        360.0f, 0.0f, 5.0f,
        20.0f, 200.0f,
        200.0f, 1U,
        100.0f, 1U,
        4U,
        pid_t::INTEGRAL_LIMIT | pid_t::OUTPUT_FILTER |
            pid_t::DERIVATIVE_FILTER | pid_t::DERIVATIVE_ON_MEASUREMENT);

    s_deps->pid_deps.roll_pid = new pid_t(
        360.0f, 0.0f, 5.0f,
        20.0f, 200.0f,
        200.0f, 1U,
        100.0f, 1U,
        4U,
        pid_t::INTEGRAL_LIMIT | pid_t::OUTPUT_FILTER |
            pid_t::DERIVATIVE_FILTER | pid_t::DERIVATIVE_ON_MEASUREMENT);
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
