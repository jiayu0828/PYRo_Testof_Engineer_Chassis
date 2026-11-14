#ifndef __PYRO_ENGINEER_CHASSIS_H__
#define __PYRO_ENGINEER_CHASSIS_H__

#include "pyro_module_base.h"
#include "pyro_algo_pid.h"
#include "pyro_motor_base.h"
#include "pyro_kin_hybrid.h" // 混合底盘运动学（履带+麦轮，支持非对称独立 K）
#include "pyro_kin_leg.h"    // 后腿四杆正运动学
#include "engineer_config.h"

namespace pyro
{

// =========================================================
// 前置声明
// =========================================================
struct engineer_cmd_t;
struct engineer_deps_t;
struct engineer_context_t;

// =========================================================
// 1. 命令定义
// =========================================================

// --- 底盘行走命令 ---
struct chassis_cmd_t
{
    float vx; // x 轴方向速度 m/s
    float vy; // y 轴方向速度 m/s
    float wz; // 旋转角速度 rad/s
};

// --- 腿部命令 ---
struct leg_cmd_t
{
    bool en;             // 腿使能（右拨杆 UP）
    bool calibrate;      // 校准脉冲（右拨杆进入 UP 的那一帧为 true）
    float target_theta;  // 手动目标相对电机角（右摇杆 ry，两腿同步）
};

// --- 总命令结构体 ---
// 后续履带/轮腿/越障等子命令以子结构形式在此追加（如 lift_cmd_t、magazine_cmd_t）
struct engineer_cmd_t final : public cmd_base_t
{
    chassis_cmd_t chassis; // 底盘行走
    leg_cmd_t     leg;     // 腿部

    // 电机使能（右拨杆三档）
    bool mecanum_en;  // 麦轮使能（右拨杆 MID/UP）
    bool track_en;    // 履带电机使能（右拨杆 UP）
    // 履带恒速运行（左拨杆 UP）；仅在 track_en 时有效
    bool track_run;

    engineer_cmd_t()
    {
        // 默认安全值：全部停止
        chassis.vx = 0.0f;
        chassis.vy = 0.0f;
        chassis.wz = 0.0f;
        leg.en         = false;
        leg.calibrate  = false;
        leg.target_theta = 0.0f;
        mecanum_en = false;
        track_en   = false;
        track_run  = false;
    }
};

// =========================================================
// 2. 依赖定义
// =========================================================
struct engineer_deps_t
{
    // 电机句柄
    struct motor_deps_t
    {
        motor_base_t *mecanum[4]{nullptr}; // 4 个麦轮电机 FL, FR, BL, BR（CAN3）
        motor_base_t *track[2]{nullptr};   // 2 个履带电机 L, R（达妙 4310，CAN2）
        motor_base_t *leg[2]{nullptr};     // 2 个腿电机 L, R（达妙，CAN1）
    };

    // PID 控制对象
    struct pid_deps_t
    {
        pid_t *mecanum_pid[4]{nullptr}; // 麦轮速度环 PID
        pid_t *track_pid[2]{nullptr};   // 履带速度环 PID
        pid_t *leg_pos_pid[2]{nullptr}; // 腿位置环 PID
        pid_t *leg_vel_pid[2]{nullptr}; // 腿速度环 PID
    };

    motor_deps_t motor_deps{};
    pid_deps_t   pid_deps{};
};

// =========================================================
// 3. 上下文数据定义
// =========================================================

// 运行时数据
struct engineer_data_ctx_t
{
    // ---- 4 个麦轮 ----
    bool  wheel_online[4]{};
    float current_wheel_rpm[4]{}; // 反馈：轮轴 RPM
    float target_wheel_rpm[4]{};  // 目标：轮轴 RPM
    float out_wheel_torque[4]{};  // 输出扭矩 Nm

    // ---- 2 个履带（索引 [0]=L, [1]=R）----
    bool  track_online[2]{};
    float current_track_rpm[2]{};  // 反馈：履带驱动轮 RPM
    float target_track_rpm[2]{};   // 目标：履带驱动轮 RPM
    float out_track_torque[2]{};   // 输出力矩 Nm

    // ---- 2 条腿（索引 [0]=L, [1]=R）----
    bool  leg_online[2]{};
    float leg_theta_unwrap[2]{};   // 电机输出轴多圈角（连续，rad）
    float leg_theta_cal[2]{};      // 校准时的电机多圈角基准
    float current_leg_theta[2]{};  // 相对电机角（校准点为 0，rad）
    float current_leg_radps[2]{};  // 电机输出轴角速度 rad/s
    float target_leg_theta[2]{};   // 目标相对电机角（手动模式来自右摇杆）
    float target_leg_radps[2]{};   // 位置环输出的目标角速度
    float out_leg_torque[2]{};     // 最终输出力矩 Nm
    float out_leg_gravity_torque[2]{}; // 整车重力补偿前馈力矩 Nm（当前开关关闭，为0）
    bool  leg_calibrated[2]{};     // 是否已校准
    // 正运动学结果
    float leg_alpha[2]{};          // 当前曲柄角 rad
    float leg_phi[2]{};            // 当前小腿角 rad
    float leg_fx[2]{}, leg_fy[2]{};  // 末端 F
    float leg_wx[2]{}, leg_wy[2]{};  // 轮心 W
    // IMU
    float imu_pitch{};             // 车身俯仰角 rad
    float imu_pitch_radps{};       // 俯仰角速度 rad/s
};

// 完整上下文
struct engineer_context_t
{
    engineer_deps_t::motor_deps_t motor;
    engineer_deps_t::pid_deps_t   pid;
    engineer_data_ctx_t           data;

    engineer_cmd_t *cmd{nullptr}; // 当前命令指针
};

// =========================================================
// 4. ModuleParams 聚合体（两参数版本必须）
// =========================================================
struct engineer_params_t
{
    using CmdType    = engineer_cmd_t;
    using ModuleDeps = engineer_deps_t;
    using ModuleCtx  = engineer_context_t;
};

// =========================================================
// 5. 工程车底盘类（两参数版本）
// =========================================================
class engineer_chassis_t final
    : public module_base_t<engineer_chassis_t, engineer_params_t>
{
    // 友元：让基类能调用私有虚函数
    friend class module_base_t<engineer_chassis_t, engineer_params_t>;

    using owner = engineer_chassis_t;

  public:
    // 单例：禁止拷贝
    engineer_chassis_t(const engineer_chassis_t &)            = delete;
    engineer_chassis_t &operator=(const engineer_chassis_t &) = delete;

  private:
    // 构造函数私有（单例模式，通过 instance() 访问）
    engineer_chassis_t();
    ~engineer_chassis_t() override = default;

    // =============================================
    // 三个核心虚函数（基类要求必须实现）
    // =============================================
    status_t _init() override;        // 初始化回调（启动时调用一次）
    void _update_feedback() override; // 反馈更新回调（每周期读传感器）
    void _fsm_execute() override;     // 状态机执行回调（每周期跑控制）

    // =============================================
    // 内部辅助函数
    // =============================================
    void _apply_motor_enable();       // 按右拨杆命令边沿式使能/失能各电机
    void _kinematics_solve();         // 麦轮逆运动学求解
    void _mecanum_control();          // 麦轮速度环 PID 控制
    void _track_control();            // 履带速度环 PID 控制
    void _leg_feedback();             // 腿反馈：unwrap、校准、正运动学、IMU
    void _calibrate_leg();            // 腿软件置零（小坦式校准）
    void _leg_control();              // 腿双环 PID
    void _send_motor_command() const; // 发送所有电机指令

    // 运动学求解器（混合底盘：巡航用麦轮，越障时叠加履带）
    hybrid_kin_t *_kinematics{nullptr};
    // 腿四杆正运动学求解器
    leg_kin_t *_leg_kinematics{nullptr};

    // 腿反馈 unwrap 内部状态
    float _last_leg_raw[2]{};  // 上一帧电机原始角
    bool  _leg_fb_init[2]{};   // 首帧初始化标志

    // 手动模式目标角平滑（一阶插值），避免摇杆突变/回中时腿快速冲击
    float _leg_target_smooth[2]{};

    // 各电机当前使能状态（边沿检测用，避免每周期重复发 enable/disable 帧）
    bool _mec_en_state[4]{};
    bool _track_en_state[2]{};
    bool _leg_en_state[2]{};

    // =============================================
    // FSM 状态机（后续越障等状态在此追加）
    // =============================================

    // 被动状态（无力/安全模式）
    struct state_passive_t final : public state_t<owner>
    {
        void enter(owner *owner) override;
        void execute(owner *owner) override;
        void exit(owner *owner) override;
    };

    // 主动状态（工作模式）
    struct state_active_t final : public state_t<owner>
    {
        void enter(owner *owner) override;
        void execute(owner *owner) override;
        void exit(owner *owner) override;
    };

    // 状态实例
    state_passive_t _state_passive;
    state_active_t  _state_active;
    fsm_t<owner>    _main_fsm;
};

} // namespace pyro

#endif // __PYRO_ENGINEER_CHASSIS_H__
