#ifndef __PYRO_ENGINEER_CHASSIS_H__
#define __PYRO_ENGINEER_CHASSIS_H__

#include "pyro_module_base.h"
#include "pyro_algo_pid.h"
#include "pyro_motor_base.h"
#include "pyro_kin_hybrid.h" // 混合底盘运动学（履带+麦轮，支持非对称独立 K）
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

// --- 总命令结构体 ---
// 后续履带/轮腿/越障等子命令以子结构形式在此追加（如 lift_cmd_t、magazine_cmd_t）
struct engineer_cmd_t final : public cmd_base_t
{
    chassis_cmd_t chassis; // 底盘行走

    engineer_cmd_t()
    {
        // 默认安全值：全部停止
        chassis.vx = 0.0f;
        chassis.vy = 0.0f;
        chassis.wz = 0.0f;
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
        motor_base_t *mecanum[4]{nullptr}; // 4 个麦轮电机 FL, FR, BL, BR
    };

    // PID 控制对象
    struct pid_deps_t
    {
        pid_t *mecanum_pid[4]{nullptr}; // 麦轮速度环 PID
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
    bool  wheel_online[4]{};
    float current_wheel_rpm[4]{}; // 反馈：轮轴 RPM
    float target_wheel_rpm[4]{};  // 目标：轮轴 RPM
    float out_wheel_torque[4]{};  // 输出扭矩 Nm
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
    void _kinematics_solve();         // 麦轮逆运动学求解
    void _mecanum_control();          // 麦轮速度环 PID 控制
    void _send_motor_command() const; // 发送所有电机指令

    // 运动学求解器（混合底盘：巡航用麦轮，越障时叠加履带）
    hybrid_kin_t *_kinematics{nullptr};

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
