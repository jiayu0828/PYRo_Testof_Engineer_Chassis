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

// =========================================================
// 3. 后腿机械实测参数（SolidWorks/实测，单位 m、rad）
// 后续使用
// =========================================================
constexpr float LEG_B_X = -0.06495f; // 固定铰 B 的车体 x 坐标
constexpr float LEG_B_Y = -0.03750f; // 固定铰 B 的车体 y 坐标
constexpr float LEG_C_X = -0.04952f; // 固定铰 C 的车体 x 坐标
constexpr float LEG_C_Y = -0.19542f; // 固定铰 C 的车体 y 坐标

constexpr float LEG_L_BE = 0.072599f; // 四杆 BE 长度
constexpr float LEG_L_ED = 0.162674f; // 四杆 ED 长度
constexpr float LEG_L_DC = 0.038000f; // 四杆 DC 长度

constexpr float LEG_F_LOCAL_X = 0.15567f; // F 点在小腿局部系 x 坐标
constexpr float LEG_F_LOCAL_Y = 0.0f;     // F 点在小腿局部系 y 坐标
constexpr float LEG_W_LOCAL_X = 0.21524f; // 轮心 W 在小腿局部系 x 坐标
constexpr float LEG_W_LOCAL_Y = 0.04014f; // 轮心 W 在小腿局部系 y 坐标

constexpr int LEG_BRANCH = -1; // 装配分支（实测 D 对应 -1）

// 达妙反馈为电机输出轴角；外部仅计电机输出轴 24T -> 曲柄 48T 齿轮级。
// 若端到端实测电机角/曲柄角不是约 2.0，应以实测值覆盖，禁止再按旧 CAD 推断叠加同步带。
constexpr float LEG_EXTERNAL_RATIO = 2.0f;

// 校准位（相对角 0）对应的绝对曲柄角：完全收起的内切死点 225.89°
constexpr float LEG_CAL_ALPHA = 3.94250156f;

// =========================================================
// 4. 后腿电机、手动模式与输出安全参数
// =========================================================
constexpr float LEG_MAX_TORQUE = 27.0f;  // 腿电机最终输出力矩上限 N·m
constexpr float LEG_MAX_SPEED  = 5.655f; // 腿电机输出轴速度上限 rad/s

// 腿部软件校准点（电机多圈绝对角 rad，用户实车标定；_calibrate_leg 使用）
constexpr float LEG_CALI_LEFT  = 1.03348756f;  // 左腿校准位绝对角
constexpr float LEG_CALI_RIGHT = -1.18842185f; // 右腿校准位绝对角

// 手动控腿行程（电机输出轴相对角，rad）：校准收起位为 0，放下方向为负
constexpr float LEG_MANUAL_MAX = 0.0f;
constexpr float LEG_MANUAL_MIN = -2.28f;

constexpr float LEG_TARGET_LERP = 0.02f; // 手动目标角一阶平滑系数

// 当前固定目标 0 回零分支（磁盘现有 !home_mode 分支）的限速线性插值。
// 插值对象是送入位置环的参考角，单位为腿电机输出轴 q 帧 rad。
constexpr float LEG_ZERO_INTERP_VMAX = 6.0f;  // 回零参考角最大接近速度 rad/s
constexpr float LEG_ZERO_INTERP_EPS  = 0.005f; // 参考角到位吸附死区 rad

// 全模式 q 空间主动阻尼：tau_d = -Kd * qdot
constexpr float LEG_F_JOINT_DAMP_KD  = 4.0f; // N·m·s/rad(q)
constexpr float LEG_F_JOINT_DAMP_MAX = 6.0f; // N·m

// 输出斜率限制：每 1ms 周期最大力矩变化量 N·m，抑制模式切换/有效域跳变冲击
constexpr float LEG_F_TORQUE_SLEW_STEP = 0.20f;

// 左拨杆 DOWN 手动控腿时，右摇杆下推判定死区；MID 回零分支不使用摇杆。
constexpr float LEG_MANUAL_STICK_EPS = 0.05f;

// =========================================================
// 5. 小坦巡航 _leg_vmc() 适配链（仅左拨杆 UP 投入）
// ---------------------------------------------------------
// 这里的符号和多项式属于腿关节 q 帧到小坦 theta_XT 帧的算法坐标适配，
// 不是电机硬件安装方向；硬件方向统一看第 0 节 MOTOR_SIGN 表。
constexpr bool  LEG_VMC_GRAVITY_EN = true;
constexpr float LEG_GRAVITY        = 9.80665f;  // 重力加速度 m/s²
constexpr float LEG_POLY_VEH_MASS   = 20.0f;     // 当前 VMC 链使用的整车等效质量 kg

constexpr float LEG_Q_XT_SIGN       = -1.0f; // q 帧 <-> 小坦 theta_XT 帧变换符号
constexpr float LEG_POLY_THETA_OFFSET = 0.0f;

// 小坦多项式有效拟合域与本车 q 对应窗口
constexpr float LEG_POLY_THETA_MIN = 0.15f;
constexpr float LEG_POLY_THETA_MAX = 1.64f;
constexpr float LEG_POLY_Q_VALID_MIN = -1.64f; // q 有效窗下限，对应 theta=1.64
constexpr float LEG_POLY_Q_VALID_MAX = -0.15f; // q 有效窗上限，对应 theta=0.15

constexpr float LEG_POLY_GRAVITY_SCALE   = 1.0f;  // 对齐小坦 _leg_vmc()，无额外缩放
constexpr float LEG_POLY_GRAVITY_TAU_MAX = 25.0f; // 重力力矩单独限幅；发送前还有 27N·m 总限幅

// q 有效窗外的本车安全软墙：避免 q≈0 收起侧主动推腿
constexpr float LEG_POLY_WALL_K   = 80.0f;
constexpr float LEG_POLY_WALL_D   = 8.0f;
constexpr float LEG_POLY_WALL_MAX = 8.0f;

// 小坦巡航力平衡几何参数（来自小坦 hybrid_config.h）
constexpr float LEG_XT_MIN_POS      = 0.15f;
constexpr float LEG_XT_MAX_POS      = 1.64f;
constexpr float LEG_XT_DIST_FRONT   = 0.2395f;
constexpr float LEG_XT_DIST_HIP     = 0.1805f;
constexpr float LEG_XT_H_COG        = 0.18f;
constexpr float LEG_XT_H_HIP_OFFSET = 0.074f;

// 小坦 JX/JY/XB/YB 多项式系数
constexpr uint32_t LEG_XT_JX_DEGREE = 7;
constexpr uint32_t LEG_XT_JY_DEGREE = 5;
constexpr uint32_t LEG_XT_XB_DEGREE = 6;
constexpr uint32_t LEG_XT_YB_DEGREE = 4;

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

// Pitch/Roll 姿态环参数归属：
// - PID 增益、限幅、滤波：pyro_chassis_app.cpp 的 pitch_pid/roll_pid 构造函数；
// - 目标角、使能开关：腿部 VMC 状态文件与本配置头共同使用。
constexpr bool  LEG_ATTITUDE_PID_EN   = true;
constexpr float LEG_TARGET_PITCH_RAD = 0.10f; // rad
constexpr float LEG_TARGET_ROLL_RAD  = 0.0f;  // rad

// =========================================================
// 6. IMU 安装补偿
// =========================================================
constexpr float PITCH_OFFSET_RAD = -0.065f; // 车水平时 IMU 静止俯仰偏置，应用层减去
constexpr float ROLL_OFFSET_RAD  = 0.0f;    // 车水平时 IMU 静止横滚偏置，应用层减去

#endif // __ENGINEER_CONFIG_H__
