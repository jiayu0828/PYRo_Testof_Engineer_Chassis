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
// 履带参数
// =========================================================
constexpr float TRACK_SPACING = 0.45f; // 左右履带中心距离
// 履带恒速目标（RPM）：左拨杆 UP 启动后，履带以此速度恒速旋转，不与 vx 绑定
constexpr float TRACK_CONSTANT_RPM = 150.0f;

// =========================================================
// 遥控最大速度限制（保守初值，实测调整）
// =========================================================
constexpr float MAX_CHASSIS_VX = 3.0f; // 最大前后速度 m/s
constexpr float MAX_CHASSIS_VY = 3.0f; // 最大左右速度 m/s
constexpr float MAX_CHASSIS_WZ = 4.5f; // 最大旋转角速度 rad/s

// =========================================================
// 后腿四杆机构（SolidWorks 实测，单位 m）
// =========================================================
// 固定铰世界坐标
constexpr float LEG_B_X = -0.06495f;
constexpr float LEG_B_Y = -0.03750f;
constexpr float LEG_C_X = -0.04952f;
constexpr float LEG_C_Y = -0.19542f;
// 四杆杆长
constexpr float LEG_L_BE = 0.072599f;
constexpr float LEG_L_ED = 0.162674f;
constexpr float LEG_L_DC = 0.038000f;
// 小腿局部系（C 原点，+x 沿 C->F）
constexpr float LEG_F_LOCAL_X = 0.15567f;
constexpr float LEG_F_LOCAL_Y = 0.0f;
constexpr float LEG_W_LOCAL_X = 0.21524f;
constexpr float LEG_W_LOCAL_Y = 0.04014f;
// 装配分支（实测 D 对应 -1）
constexpr int LEG_BRANCH = -1;

// 电机->曲柄 外部传动比：齿轮 24->48(=2) × 同步带(≈2.03)，总≈4.06，待实测
constexpr float LEG_EXTERNAL_RATIO = 4.06f;
// 物理电机角增大对应曲柄局部角增大(+1)/减小(-1)。镜像装配导致左右相反：
// 左腿电机角减小才放下(同方向,+1)，右腿电机角增大才放下(反方向,-1)
constexpr int   LEG_MOTOR_DIR[2] = {1, -1};
// 校准位（相对角 0）对应的绝对曲柄角：完全收起的内切死点 225.89°
constexpr float LEG_CAL_ALPHA = 3.94250156f; // 225.89°

// 腿电机（达妙）限制
constexpr float LEG_MAX_TORQUE = 27.0f;
constexpr float LEG_MAX_SPEED  = 5.655f; // 输出轴 rad/s

// 手动控腿（模式A）：以完全收起为 0，只能往放下（负方向）走，故为非对称限幅
constexpr float LEG_MANUAL_MAX = 0.0f;    // 收起端：已在死点，不能再往收
constexpr float LEG_MANUAL_MIN = -4.64f;  // 放下端：完全放下（理论 -4.6445，留余量）

// 目标角平滑（一阶插值）：每帧平滑目标向摇杆命令靠近的比例，越小越慢、越柔和
constexpr float LEG_TARGET_LERP = 0.05f;

// 腿逻辑方向（镜像装配）：左腿 +1，右腿 -1（右腿方向取反，使两腿机构动作同步）
constexpr int LEG_LOGIC_SIGN[2] = {1, -1};

// =========================================================
// 整车重力补偿（VMC 腿撑地承重，对髋关节 B 取矩的杠杆平衡）
// 仅在腿撑地、承担车重时使能；悬空时必须关闭，否则会产生错误大力矩
// =========================================================
constexpr bool LEG_VMC_GRAVITY_EN = true;   // 使能开关
constexpr float LEG_GRAVITY       = 9.80665f; // 重力加速度 m/s²
constexpr float LEG_VEH_MASS      = 20.0f;   // 整车总质量 kg（拟定，可称重修正）
// 整车质心相对髋关节 B 的车体系位置（m，拟定值，待 SolidWorks 整车质心修正）
constexpr float LEG_COG_DX = 0.25f; // 纵向（前后）距离：质心在 B 前方
constexpr float LEG_COG_DY = 0.20f; // 质心在 B 上方的高度

// =========================================================
// VMC Pitch 姿态平衡（履带启动、左拨杆 UP 后，腿自动扶正整车）
// =========================================================
constexpr float LEG_TARGET_PITCH = 0.0f;   // 目标 pitch（rad），先保持水平
constexpr float LEG_PITCH_KP     = 200.0f;  // 弹簧：pitch 每偏 1 rad 产生的虚拟力 N
constexpr float LEG_PITCH_KD     = 5.0f;   // 阻尼：每个俯仰角速度产生的虚拟力 N/(rad/s)
constexpr float LEG_PITCH_F_MAX  = 150.0f; // 虚拟扶正力限幅 N，防止异常大值
// 扶正方向开关：闭环方向正确取 +1；若越扶越歪（IMU 安装/传动方向导致 pitch 反向）取 -1
// 实车确认 SIGN=-1 时手感反了，翻转为 +1 纠正整体闭环方向
constexpr float LEG_PITCH_SIGN   = 1.0f;


// IMU 固定安装俯仰角补偿（rad）：车水平时 IMU 静止读数的恒定偏置，应用层减去
constexpr float PITCH_OFFSET_RAD = -0.065f;

#endif // __ENGINEER_CONFIG_H__
