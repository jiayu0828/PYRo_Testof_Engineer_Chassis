#ifndef __ENGINEER_CONFIG_H__
#define __ENGINEER_CONFIG_H__

#include <cstdint>

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
// 左右镜像物理方向由反馈/发送层符号处理，因此这里两个逻辑目标同号。
constexpr float TRACK_CONSTANT_RPM = 100.0f;

// =========================================================
// 2. 遥控速度限制
// =========================================================
constexpr float MAX_CHASSIS_VX = 3.0f; // 最大前后速度 m/s
constexpr float MAX_CHASSIS_VY = 3.0f; // 最大左右速度 m/s
constexpr float MAX_CHASSIS_WZ = 4.5f; // 最大旋转角速度 rad/s

// =========================================================
// 3. 后腿机械实测参数（SolidWorks/实测，单位 m、rad）
//    当前运行的小坦巡航 VMC 适配链未直接引用以下四杆尺寸；
//    保留在固件中，供后续恢复本车四杆/F 点运动学链使用。
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

// 物理电机角增大对应曲柄局部角增大(+1)/减小(-1)。镜像装配导致左右相反。
constexpr int LEG_MOTOR_DIR[2] = {1, -1};

// 校准位（相对角 0）对应的绝对曲柄角：完全收起的内切死点 225.89°
constexpr float LEG_CAL_ALPHA = 3.94250156f;

// =========================================================
// 4. 后腿电机、手动模式与输出安全参数
// =========================================================
constexpr float LEG_MAX_TORQUE = 27.0f;  // 腿电机最终输出力矩上限 N·m
constexpr float LEG_MAX_SPEED  = 5.655f; // 腿电机输出轴速度上限 rad/s

// 手动控腿行程（电机输出轴相对角，rad）：校准收起位为 0，放下方向为负
constexpr float LEG_MANUAL_MAX = 0.0f;
constexpr float LEG_MANUAL_MIN = -2.28f;

constexpr float LEG_TARGET_LERP = 0.02f; // 手动目标角一阶平滑系数

// 腿逻辑方向（镜像装配）：左腿 +1，右腿 -1；发送层统一乘该符号
constexpr int LEG_LOGIC_SIGN[2] = {1, -1};

// 全模式 q 空间主动阻尼：tau_d = -Kd * qdot
constexpr float LEG_F_JOINT_DAMP_KD  = 4.0f; // N·m·s/rad(q)
constexpr float LEG_F_JOINT_DAMP_MAX = 6.0f; // N·m

// 输出斜率限制：每 1ms 周期最大力矩变化量 N·m，抑制模式切换/有效域跳变冲击
constexpr float LEG_F_TORQUE_SLEW_STEP = 0.20f;

// MID 手动保持段默认不叠加重力前馈；操作员主动推摇杆时，重力诊断/前馈临时打折
constexpr bool  LEG_MANUAL_GRAVITY_FF   = false;
constexpr float LEG_MANUAL_CMD_FF_SCALE = 0.30f;
constexpr float LEG_MANUAL_STICK_EPS    = 0.05f; // 右摇杆下推判定死区

// =========================================================
// 5. 小坦巡航 _leg_vmc() 适配链
// ---------------------------------------------------------
// 当前工程校准收起位 q=0、放下为负；小坦 theta_XT 站立角域为正，因此：
//     theta_XT = LEG_POLY_THETA_SIGN * q + LEG_POLY_THETA_OFFSET
// 小坦 helper 输出 theta 帧 tau_XT；调用层用 LEG_POLY_OUTPUT_SIGN
// 转成本车统一 q 逻辑力矩，发送层最后再乘 LEG_LOGIC_SIGN。
// 方向反了只允许调整 LEG_POLY_OUTPUT_SIGN，禁止同时改多个方向宏。
// =========================================================
constexpr bool  LEG_VMC_GRAVITY_EN = true;
constexpr float LEG_GRAVITY        = 9.80665f; // m/s²

// 注意：当前磁盘整理保留整理前数值 16.0kg；小坦 hybrid_config.h 的 MASS=25.5kg。
// 纯代码整理不改变控制参数，实车重力验收前需再次确认应使用哪一个质量。
constexpr float LEG_POLY_VEH_MASS = 16.0f;

constexpr float LEG_POLY_THETA_SIGN   = -1.0f;
constexpr float LEG_POLY_THETA_OFFSET = 0.0f;
constexpr float LEG_POLY_OUTPUT_SIGN  = -1.0f;

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

// =========================================================
// 6. Pitch/Roll 腿部姿态补偿
// ---------------------------------------------------------
// 左拨杆 UP（履带恒速 + 腿部 VMC）时投入。PID 输出是任务空间虚拟力 N，
// 再乘当前小坦链的动态雅可比 j_dynamic 得到 theta 帧力矩，最后统一做
// 本车 q 帧/发送方向适配。首测参数对照小坦 hybrid_chassis_app.cpp。
// =========================================================
constexpr bool  LEG_ATTITUDE_PID_EN   = true;
constexpr float LEG_TARGET_PITCH_RAD  = 0.10f; // 对照小坦 NORMAL_PITCH
constexpr float LEG_TARGET_ROLL_RAD   = 0.0f;

// 架空首测若某一轴补偿方向反了，只把对应符号改成 -1；
// 不要同时修改 LEG_POLY_OUTPUT_SIGN 或 LEG_LOGIC_SIGN。
constexpr float LEG_PITCH_FORCE_SIGN = 1.0f;
constexpr float LEG_ROLL_FORCE_SIGN  = 1.0f;

constexpr float LEG_PITCH_KP          = 260.0f; // N/rad
constexpr float LEG_ROLL_KP           = 360.0f; // N/rad
constexpr float LEG_ATTITUDE_KI       = 0.1f;
constexpr float LEG_ATTITUDE_KD       = 10.0f;  // N·s/rad
constexpr float LEG_ATTITUDE_I_LIMIT  = 20.0f;  // 积分项限幅
constexpr float LEG_ATTITUDE_MAX_FORCE = 200.0f; // PID 总输出力限幅 N

constexpr float   LEG_ATT_OUTPUT_CUTOFF_HZ      = 200.0f;
constexpr uint8_t LEG_ATT_OUTPUT_LPF_ORDER      = 1;
constexpr float   LEG_ATT_DERIV_CUTOFF_HZ       = 100.0f;
constexpr uint8_t LEG_ATT_DERIV_LPF_ORDER       = 1;
constexpr uint16_t LEG_ATT_OLS_ORDER            = 4;

// =========================================================
// 7. IMU 安装补偿
// =========================================================
constexpr float PITCH_OFFSET_RAD = -0.065f; // 车水平时 IMU 静止俯仰偏置，应用层减去
constexpr float ROLL_OFFSET_RAD  = 0.0f;    // 车水平时 IMU 静止横滚偏置，应用层减去

#endif // __ENGINEER_CONFIG_H__
