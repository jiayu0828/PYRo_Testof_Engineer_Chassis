#ifndef __ENGINEER_CONFIG_H__
#define __ENGINEER_CONFIG_H__

// =========================================================
// 麦轮底盘几何参数（约数，待机械图纸精确值后替换；单位 m）
// =========================================================
constexpr float WHEEL_RADIUS = 0.076f; // 麦轮半径
constexpr float WHEELBASE    = 0.45f;  // 轴距：前后轮中心距离
constexpr float TRACK_WIDTH_FRONT  = 0.30f;  // 轮距：左右轮中心距离
constexpr float TRACK_WIDTH_BACK   = 0.50f;  // 轮距：左右轮中心距离
// =========================================================
// 混合底盘几何（前履带 + 后麦轮）
// =========================================================
// 独立旋转系数 K_i = 轮 i 到旋转中心的 |纵向距离| + |横向距离|。
constexpr float K_FL = (WHEELBASE + TRACK_WIDTH_FRONT) / 2.0f; // 左前麦轮（待核验）
constexpr float K_FR = (WHEELBASE + TRACK_WIDTH_FRONT) / 2.0f; // 右前麦轮（待核验）
constexpr float K_BL = (WHEELBASE + TRACK_WIDTH_BACK) / 2.0f; // 左后麦轮（待核验）
constexpr float K_BR = (WHEELBASE + TRACK_WIDTH_BACK) / 2.0f; // 右后麦轮（待核验）
// =========================================================
// 混合底盘履带几何参数（约数，待机械图纸精确值后替换；单位 m）
// =========================================================
constexpr float TRACK_SPACING = 0.45f;//左右履带中心距离
constexpr float TRACK_WHEEL_RADIUS = 0.062f; // 履带驱动轮半径（占位，待图纸实测）

// =========================================================
// 遥控最大速度限制（保守初值，实测调整）
// =========================================================
constexpr float MAX_CHASSIS_VX = 3.0f; // 最大前后速度 m/s
constexpr float MAX_CHASSIS_VY = 3.0f; // 最大左右速度 m/s
constexpr float MAX_CHASSIS_WZ = 4.5f; // 最大旋转角速度 rad/s

#endif // __ENGINEER_CONFIG_H__
