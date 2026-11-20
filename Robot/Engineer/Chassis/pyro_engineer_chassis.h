#ifndef __PYRO_ENGINEER_CHASSIS_H__
#define __PYRO_ENGINEER_CHASSIS_H__

#include "pyro_module_base.h"
#include "pyro_algo_pid.h"
#include "pyro_motor_base.h"
#include "pyro_kin_hybrid.h" // 混合底盘运动学（麦轮速度解算，支持非对称独立 K）
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
    bool  en;            // 腿使能（右拨杆 UP）
    bool  calibrate;     // 校准脉冲（右拨杆进入 UP 的那一帧为 true）
    float target_theta;  // 手动目标相对电机角（右摇杆 ry，两腿同步）
};

// --- 总命令结构体 ---
struct engineer_cmd_t final : public cmd_base_t
{
    chassis_cmd_t chassis; // 底盘行走
    leg_cmd_t     leg;     // 腿部

    // 电机使能（右拨杆三档）
    bool mecanum_en;   // 麦轮使能（右拨杆 MID/UP）
    bool track_en;     // 履带电机使能（右拨杆 UP）

    // 左拨杆模式：UP=腿部小坦巡航 VMC；MID/DOWN=腿部手动保持。
    // 当前履带速度环目标固定为 0，该开关不再表示履带恒速。
    bool leg_vmc_mode;

    engineer_cmd_t()
    {
        // 默认安全值：全部停止
        chassis.vx       = 0.0f;
        chassis.vy       = 0.0f;
        chassis.wz       = 0.0f;
        leg.en           = false;
        leg.calibrate    = false;
        leg.target_theta = 0.0f;
        mecanum_en       = false;
        track_en         = false;
        leg_vmc_mode     = false;
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
struct engineer_data_ctx_t
{
    // ---- 4 个麦轮（索引 [0]=FL, [1]=FR, [2]=BL, [3]=BR）----
    bool  wheel_online[4]{};
    float current_wheel_rpm[4]{}; // 反馈：轮轴 RPM（逻辑正方向）
    float target_wheel_rpm[4]{};  // 目标：轮轴 RPM
    float out_wheel_torque[4]{};  // 输出力矩

    // ---- 2 个履带（索引 [0]=L, [1]=R）----
    bool  track_online[2]{};
    float current_track_rpm[2]{}; // 反馈：履带驱动轮 RPM（逻辑正方向）
    float target_track_rpm[2]{};  // 目标：履带驱动轮 RPM
    float out_track_torque[2]{};  // 输出力矩

    // ---- 2 条腿（索引 [0]=L, [1]=R）----
    bool  leg_online[2]{};
    float leg_raw_position[2]{};      // 电机原始单圈角 rad
    float leg_theta_unwrap[2]{};      // 电机输出轴多圈角（连续，rad）
    float leg_theta_cal[2]{};         // 校准时的电机多圈角基准
    float current_leg_theta[2]{};     // 相对电机角（校准点为 0，rad）
    float current_leg_radps[2]{};     // 电机输出轴角速度 rad/s
    float target_leg_theta[2]{};      // 目标相对电机角（手动模式来自右摇杆）
    float target_leg_radps[2]{};      // 位置环输出的目标角速度
    float out_leg_torque[2]{};        // 最终输出逻辑力矩 N·m
    float out_leg_gravity_torque[2]{}; // 重力前馈力矩（逻辑坐标系，N·m）
    bool  leg_calibrated[2]{};        // 是否已校准

    // ---- 腿部 VMC/调试中间量（供 JustFloat 和 Live Watch 核查）----
    float leg_alpha[2]{};           // 当前送入小坦多项式的 theta_XT（rad）
    bool  leg_gravity_valid[2]{};   // 重力前馈是否有效
    float leg_j_dynamic[2]{};       // 当前姿态下的动态雅可比标量
    float leg_tau_wall[2]{};        // q 空间软墙力矩 N·m
    float leg_tau_damping[2]{};     // q 空间主动阻尼力矩 N·m
    float leg_ff_scale[2]{};        // 重力前馈缩放系数
    float leg_tau_manual_pid[2]{};  // 手动双环 PID 输出力矩 N·m
    float leg_control_mode[2]{};    // 0=失能，1=手动保持，2=巡航 VMC

    // ---- IMU：原始值和滤波值同时保留，控制只使用滤波值 ----
    float imu_pitch_raw{};
    float imu_pitch{};
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

    // 获取上下文（非 const 版本，基类只有 const 版本）
    engineer_context_t &get_ctx();

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
    // 内部辅助函数：初始化/使能
    // =============================================
    void _apply_motor_enable(); // 按右拨杆命令边沿式使能/失能各电机

    // =============================================
    // 内部辅助函数：反馈
    // =============================================
    void _leg_feedback();    // 腿反馈：unwrap、校准、IMU
    void _calibrate_leg();   // 腿软件置零（小坦式校准，非堵转）

    // =============================================
    // 内部辅助函数：控制流水线
    // =============================================
    void _kinematics_solve();         // 麦轮逆运动学求解
    void _mecanum_control();          // 麦轮速度环 PID
    void _track_control();            // 履带速度环 PID（当前目标固定 0）
    void _leg_control();              // 腿手动保持 / 巡航 VMC
    void _send_motor_command() const; // 发送所有电机指令

    // 麦轮混合运动学求解器
    hybrid_kin_t *_kinematics{nullptr};

    // =============================================
    // 腿部反馈与控制内部状态
    // =============================================
    float _last_leg_raw[2]{}; // 上一帧电机原始角
    bool  _leg_fb_init[2]{};  // 首帧初始化标志

    float _imu_pitch_lpf{};    // IMU pitch 一阶低通状态
    bool  _imu_lpf_init{false};

    float _leg_target_smooth[2]{};     // 手动模式平滑目标角
    bool  _leg_vmc_prev{false};        // 上一帧左拨杆 VMC 模式，用于模式边沿无扰切换
    bool  _leg_manual_map_active[2]{}; // 首次下推摇杆后的基准映射状态
    float _leg_manual_cmd_offset[2]{}; // 摇杆量到平滑目标的映射偏置
    float _leg_last_logic_torque[2]{};  // 上一帧最终逻辑力矩，用于斜率限制

    // 各电机当前使能状态（边沿检测用，避免每周期重复发 enable/disable 帧）
    bool _mec_en_state[4]{};
    bool _track_en_state[2]{};
    bool _leg_en_state[2]{};

    // =============================================
    // FSM 状态机（文件边界保持独立：active/passive 两个 cpp）
    // =============================================
    struct state_passive_t final : public state_t<owner>
    {
        void enter(owner *owner) override;
        void execute(owner *owner) override;
        void exit(owner *owner) override;
    };

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
