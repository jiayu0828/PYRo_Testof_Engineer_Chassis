#ifndef __ENGINEER_CONFIG_H__
#define __ENGINEER_CONFIG_H__

#include <cstdint>

// =========================================================
// 0. 电机安装方向表（硬件边界唯一方向定义）
// ---------------------------------------------------------
// +1：电机反馈正方向与底盘/关节逻辑正方向一致；
// -1：电机反装或镜像安装，反馈读取与力矩发送都乘同一个符号。
// 控制算法内部的坐标变换符号不属于电机安装方向，放在对应算法段。
// 数组顺序必须与 pyro_chassis_app.cpp 中电机创建顺序一致。
// =========================================================
constexpr int MECANUM_MOTOR_SIGN[4] = {1, -1, 1, -1}; // [0]FL [1]FR [2]BL [3]BR
constexpr int TRACK_MOTOR_SIGN[2]   = {1, -1};        // [0]L 左履带 [1]R 右履带
constexpr int LEG_MOTOR_SIGN[2]     = {1, -1};        // [0]L 左腿 [1]R 右腿

// =========================================================
// 1. 麦轮底盘几何与运动学参数
// =========================================================
constexpr float WHEEL_RADIUS       = 0.076f; // 麦轮半径 m
constexpr float WHEELBASE          = 0.45f;  // 轴距：前后轮中心距离 m
constexpr float TRACK_WIDTH_FRONT  = 0.30f;  // 前轮距：左右轮中心距离 m
constexpr float TRACK_WIDTH_BACK   = 0.50f;  // 后轮距：左右轮中心距离 m

// 混合底盘独立旋转系数 K_i = 轮 i 到旋转中心的 |纵向距离| + |横向距离|
constexpr float K_FL = (WHEELBASE + TRACK_WIDTH_FRONT) / 2.0f; // 左前麦轮（待实测核验）
constexpr float K_FR = (WHEELBASE + TRACK_WIDTH_FRONT) / 2.0f; // 右前麦轮（待实测核验）
constexpr float K_BL = (WHEELBASE + TRACK_WIDTH_BACK) / 2.0f;  // 左后麦轮（待实测核验）
constexpr float K_BR = (WHEELBASE + TRACK_WIDTH_BACK) / 2.0f;  // 右后麦轮（待实测核验）

constexpr float TRACK_SPACING = 0.45f; // 左右履带中心距离 m

// 左拨杆 UP 且右拨杆 UP 使能履带时，左右履带的逻辑目标转速 RPM。
// 左右电机镜像方向统一由 TRACK_MOTOR_SIGN 处理，因此逻辑目标同号。
constexpr float TRACK_CONSTANT_RPM = 100.0f;

// =========================================================
// 2. 遥控速度限制
// =========================================================
constexpr float MAX_CHASSIS_VX = 3.0f; // 最大前后速度 m/s
constexpr float MAX_CHASSIS_VY = 3.0f; // 最大左右速度 m/s
constexpr float MAX_CHASSIS_WZ = 4.5f; // 最大旋转角速度 rad/s

// 左拨杆 UP 上台阶时的麦轮纵向速度上限，不影响普通行驶和履带固定 RPM。
constexpr float CLIMB_MAX_VX = 0.4f;

// =========================================================
// 3. 后腿电机、手动模式与输出安全参数
// =========================================================
constexpr float LEG_MAX_TORQUE = 27.0f;  // 腿电机最终输出力矩上限 N·m
constexpr float LEG_MAX_SPEED  = 12.0f; // 腿电机输出轴速度上限 rad/s

// 腿部软件校准点
constexpr float LEG_CALI_LEFT  = 1.03348756f;  // 左腿校准位绝对角
constexpr float LEG_CALI_RIGHT = -1.18842185f; // 右腿校准位绝对角

// MID 档：右摇杆下推直接叠加的 q 帧最大手动力矩。
constexpr float LEG_MID_MANUAL_TORQUE_MAX = 8.0f;

// 当前固定目标 0 回零分支（磁盘现有 !home_mode 分支）的限速线性插值。
// 插值对象是送入位置环的参考角，单位为腿电机输出轴 q 帧 rad。
constexpr float LEG_ZERO_INTERP_VMAX = 12.0f;  // 回零参考角最大接近速度 rad/s
constexpr float LEG_ZERO_INTERP_EPS  = 0.005f; // 参考角到位吸附死区 rad

// 全模式 q 空间主动阻尼：tau_d = -Kd * qdot
constexpr float LEG_F_JOINT_DAMP_KD  = 4.0f; // N·m·s/rad(q)
constexpr float LEG_F_JOINT_DAMP_MAX = 6.0f; // N·m

// 输出斜率限制：每 1ms 周期最大力矩变化量 N·m，抑制模式切换/有效域跳变冲击
constexpr float LEG_F_TORQUE_SLEW_STEP = 0.20f;

// =========================================================
// 5. 小坦巡航 _leg_vmc() 适配链（仅左拨杆 UP 投入）
// ---------------------------------------------------------
// 这里的符号和多项式属于腿关节 q 帧到小坦 theta_XT 帧的算法坐标适配，
// 不是电机硬件安装方向；硬件方向统一看第 0 节 MOTOR_SIGN 表。
constexpr bool  LEG_VMC_GRAVITY_EN = true;
constexpr float LEG_GRAVITY        = 9.80665f;  // 重力加速度 m/s²
constexpr float LEG_POLY_VEH_MASS   = 20.0f;     // 当前 VMC 链使用的整车等效质量 kg

constexpr float LEG_Q_XT_SIGN = -1.0f; // q 帧 <-> 小坦 theta_XT 帧变换符号

// 本车 q 有效窗口；对应小坦 theta_XT 的有效拟合域。
constexpr float LEG_POLY_Q_VALID_MIN = -1.72f;
constexpr float LEG_POLY_Q_VALID_MAX = 0.0f;

constexpr float LEG_POLY_GRAVITY_TAU_MAX = 25.0f; // 重力力矩单独限幅；发送前还有 27N·m 总限幅

// 小坦巡航力平衡几何参数（来自小坦 hybrid_config.h）
constexpr float LEG_MIN_POS      = 0.0f;//需要读数确认
constexpr float LEG_MAX_POS      = 1.72f;//需要读数确认

constexpr float LEG_XT_DIST_FRONT   = 0.2395f;
constexpr float LEG_XT_DIST_HIP     = 0.1805f;
constexpr float LEG_XT_H_COG        = 0.18f;
constexpr float LEG_XT_H_HIP_OFFSET = 0.074f;
//先这样吧

// 小坦 JX/JY/XB/YB 多项式系数
constexpr uint32_t LEG_XT_JX_DEGREE = 7;
constexpr uint32_t LEG_XT_JY_DEGREE = 5;
constexpr uint32_t LEG_XT_XB_DEGREE = 6;
constexpr uint32_t LEG_XT_YB_DEGREE = 4;
//已经会推导了
constexpr float LEG_XT_JX_COEF[LEG_XT_JX_DEGREE + 1] = {
    -0.0742f, 0.3986f, -0.8740f, 0.9990f,
    -0.6561f, 0.2485f, -0.0279f, -0.0072f};

constexpr float LEG_XT_JY_COEF[LEG_XT_JY_DEGREE + 1] = {
    -0.0194f, 0.1161f, -0.2662f,
     0.3140f, -0.2409f, 0.2048f};

constexpr float LEG_XT_XB_COEF[LEG_XT_XB_DEGREE + 1] = {
    -0.0086f, 0.0379f, -0.0700f, 0.0618f,
    -0.0154f, -0.0062f, -0.0011f};

constexpr float LEG_XT_YB_COEF[LEG_XT_YB_DEGREE + 1] = {
    -0.0056f, 0.0311f, -0.0793f, 0.1957f, 0.0745f};

//相关姿态补偿  感觉Pitch 可以调整一下，来适配老左的履带的角度 
constexpr bool  LEG_ATTITUDE_PID_EN   = true;
constexpr float LEG_TARGET_PITCH_RAD = 0.10f; // rad
constexpr float LEG_TARGET_ROLL_RAD  = 0.0f;  // rad

// =========================================================
// 6. IMU 安装补偿
// =========================================================
constexpr float PITCH_OFFSET_RAD = -0.065f; // 车水平时 IMU 静止俯仰偏置，应用层减去
constexpr float ROLL_OFFSET_RAD  = 0.0f;    // 车水平时 IMU 静止横滚偏置，应用层减去

#endif // __ENGINEER_CONFIG_H__
