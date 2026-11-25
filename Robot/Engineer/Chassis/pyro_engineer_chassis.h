#ifndef __PYRO_ENGINEER_CHASSIS_H__
#define __PYRO_ENGINEER_CHASSIS_H__

#include "pyro_module_base.h"
#include "pyro_algo_pid.h"
#include "pyro_motor_base.h"
#include "pyro_powermeter.h"
#include "pyro_power_control.h"
#include "pyro_supercap_drv.h"
#include "pyro_kin_hybrid.h" // 混合底盘运动学（麦轮速度解算，支持非对称独立 K）
#include "pyro_algo_common.h"
#include "pyro_dji_motor_drv.h"
#include "pyro_ins.h"
#include "pyro_referee.h"
#include "pyro_warning_drv.h"
//pyro库相关引用
#include "engineer_config.h"
//底盘单独的参数
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
    bool  home_to_zero;  // 历史字段名；当前=true 表示左拨杆 MID：手动双环 PID + 重力/软墙前馈
    float target_theta;  // 左拨杆 MID 右摇杆手动目标相对电机角（两腿同步）
};

// --- 总命令结构体 ---
struct engineer_cmd_t final : public cmd_base_t
{
    chassis_cmd_t chassis; // 底盘行走
    leg_cmd_t     leg;     // 腿部

    // 电机使能（右拨杆三档）
    bool mecanum_en;   // 麦轮使能（右拨杆 MID/UP）
    bool track_en;     // 履带电机使能（右拨杆 UP）

    // 左拨杆档位组合（底盘内部腿控 HFSM 据此分流，通信层字段名暂时保留）：
    // UP   = 履带恒速 + 腿部 VMC（重力/软墙/Pitch/Roll 姿态力）；
    // MID  = 履带停止 + 右摇杆手动双环 PID + 重力/软墙前馈；
    // DOWN = 履带停止 + 限速回零纯双环 PID。
    bool track_run;

    engineer_cmd_t()
    {
        // 默认安全值：全部停止
        chassis.vx       = 0.0f;
        chassis.vy       = 0.0f;
        chassis.wz       = 0.0f;
        leg.en           = false;
        leg.calibrate    = false;
        leg.home_to_zero = false;
        leg.target_theta = 0.0f;
        mecanum_en       = false;
        track_en         = false;
        track_run        = false;
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
        pid_t *pitch_pid{nullptr};      // 腿部 VMC 的 Pitch 姿态力 PID
        pid_t *roll_pid{nullptr};       // 腿部 VMC 的 Roll 姿态力 PID
    };

    // 外接 CAN 功率计（当前标定阶段挂在 CAN1，CAN ID=0x212；只采集不参与闭环）
    powermeter_drv_t *powermeter{nullptr};

    motor_deps_t motor_deps{};
    pid_deps_t   pid_deps{};
};

// =========================================================
// 3. 超级电容应用层运行时
// =========================================================
struct engineer_supercap_runtime_t
{
    bool enabled{false};
    bool voltage_ready{false};

    float voltage_v{0.0f};
    float cap_power_w{0.0f};
    float chassis_power_w{0.0f};
    float extra_power_w{0.0f};

    uint32_t enable_timer{0U};
    uint32_t refresh_timer{0U};
    uint32_t disable_timer{0U};
};

// =========================================================
// 4. 上下文数据定义
// =========================================================
struct engineer_data_ctx_t
{
    // ---- 4 个麦轮（索引 [0]=FL, [1]=FR, [2]=BL, [3]=BR）----
    bool  wheel_online[4]{};
    float current_wheel_rpm[4]{}; // 反馈：轮轴 RPM（逻辑正方向）
    float target_wheel_rpm[4]{};  // 目标：轮轴 RPM
    float out_wheel_torque[4]{};  // 功控后最终麦轮指令标称量（发送边界再乘电机方向）
    float current_wheel_temp[4]{}; // M3508 电调反馈温度 ℃

    // ---- 四麦轮模型功控中间量（仅作用于麦轮，不影响履带/腿）----
    float    wheel_cmd_before_power_ctrl[4]{}; // 麦轮 PID 原始输出
    float    wheel_cmd_after_power_ctrl[4]{};  // 功控缩放/滤波后的输出
    float    wheel_power_predict_w[4]{};       // 各麦轮模型预测功率 W
    float    wheel_power_scale[4]{};           // after/before，便于观察限幅比例
    float    wheel_power_total_w{};            // 四麦轮预测总功率 W
    float    wheel_power_limit_w{};            // 当前生效总功率上限 W
    float    wheel_power_buffer_energy_j{};    // 当前功控使用的缓冲能量 J
    bool     wheel_power_control_active{};     // 本周期是否有麦轮指令被明显缩放

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
    float target_leg_theta[2]{};      // 目标相对电机角：MID 来自摇杆映射，DOWN 为限速回零参考
    float target_leg_radps[2]{};      // 位置环输出的目标角速度
    float out_leg_torque[2]{};        // 最终输出逻辑力矩 N·m
    float out_leg_gravity_torque[2]{}; // UP/MID 重力前馈力矩（q 逻辑坐标系，N·m）；DOWN 纯 PID 时为 0
    bool  leg_calibrated[2]{};        // 是否已校准

    // ---- 腿部 VMC/调试中间量（供 JustFloat 和 Live Watch 核查）----
    float leg_alpha[2]{};           // 当前送入小坦多项式的 theta_XT（rad）
    bool  leg_gravity_valid[2]{};   // 重力前馈是否有效
    float leg_j_dynamic[2]{};       // 当前姿态下的动态雅可比标量
    float leg_tau_wall[2]{};        // q 空间软墙力矩 N·m
    float leg_tau_damping[2]{};     // q 空间主动阻尼力矩 N·m
    float leg_ff_scale[2]{};        // 重力/软墙前馈投入标志（1=投入，0=纯 PID）
    float leg_tau_manual_pid[2]{};  // MID/DOWN 双环 PID 输出力矩 N·m（字段名保留以便观察）
    float leg_tau_attitude[2]{};    // Pitch/Roll 姿态 PID 映射后的 q 帧力矩 N·m
    float leg_control_mode[2]{};    // 0=失能/未校准，1=DOWN 纯双环回零，2=UP VMC，3=MID 双环+重力前馈
    float att_force_pitch{};        // Pitch PID 输出的任务空间虚拟力 N
    float att_force_roll{};         // Roll PID 输出的左右差动力 N

    // ---- IMU：原始值和滤波值同时保留，控制只使用滤波值 ----
    float imu_pitch_raw{};
    float imu_pitch{};
    float imu_roll_raw{};
    float imu_roll{};

    // ---- 0x212 CAN 功率计（标定采集；当前不做功率限制输出）----
    float    power_meter_w{};       // 功率计解析功率 W
    float    power_meter_current{}; // 功率计电流 A
    float    power_meter_voltage{}; // 功率计电压 V
    uint32_t power_meter_rx_count{}; // 成功解析帧数，用于判断 CAN 接收是否存活

};

// 完整上下文
struct engineer_context_t
{
    engineer_deps_t::motor_deps_t motor;
    engineer_deps_t::pid_deps_t   pid;
    powermeter_drv_t             *powermeter{nullptr};
    powermeter_data               powermeter_feedback{};
    engineer_supercap_runtime_t   supercap{};
    power_node_t                 *mecanum_power_node[4]{nullptr}; // 四麦轮共用同一组拟合参数
    engineer_data_ctx_t           data;

    engineer_cmd_t *cmd{nullptr}; // 当前命令指针
};

// =========================================================
// 5. ModuleParams 聚合体（两参数版本必须）
// =========================================================
struct engineer_params_t
{
    using CmdType    = engineer_cmd_t;
    using ModuleDeps = engineer_deps_t;
    using ModuleCtx  = engineer_context_t;
};

// =========================================================
// 6. 工程车底盘类（两参数版本）
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
    void _mecanum_power_control_init(); // 注册四个麦轮功控节点（共用 FL 标定参数）
    void _mecanum_power_control();      // 麦轮模型功率限制，仅覆盖麦轮输出
    void _supercap_control();           // 电容板命令组装、使能状态机和周期发送
    void _track_control();            // 履带恒速/停止速度环 PID

    // =============================================
    // 腿部控制辅助函数：腿部 HFSM 状态类位于 Chassis/fsm/leg/*.cpp
    // ---------------------------------------------
    // 这里保留机构算法和输出处理；各状态的 enter/execute/exit
    // 按 Hero 风格放在独立状态文件中，由 _leg_fsm 统一调度。
    // =============================================
    void _leg_handle_disabled(int i);
    void _leg_handle_uncalibrated(int i);
    void _leg_enter_active_mode(int i, float current_q);
    void _leg_update_vmc_feedforward(int i, float q, float qdot, float pitch,
                                     bool include_attitude,
                                     float f_pitch, float f_roll);
    void _leg_clear_feedforward(int i);
    void _leg_update_manual_target(int i, float stick_cmd);
    void _leg_update_home_target(int i, float current_q);
    float _leg_run_dual_pid(int i);
    void _leg_apply_common_output(int i, float raw_torque, float qdot);
    void _leg_control();              // 腿部 active 子 HFSM 协调入口
    void _send_motor_command() const; // 发送所有电机指令

    // 麦轮混合运动学求解器
    hybrid_kin_t *_kinematics{nullptr};

    // =============================================
    // 腿部反馈与控制内部状态
    // =============================================
    float _last_leg_raw[2]{}; // 上一帧电机原始角
    bool  _leg_fb_init[2]{};  // 首帧初始化标志

    float _imu_pitch_lpf{};    // IMU pitch 一阶低通状态
    float _imu_roll_lpf{};     // IMU roll 一阶低通状态
    bool  _imu_lpf_init{false};

    float _leg_target_smooth[2]{};      // 送入位置环的平滑参考角
    bool  _leg_zero_interp_init[2]{};   // DOWN 限速回零参考已从当前角初始化
    bool  _leg_manual_map_active[2]{};  // MID 首次下推摇杆后的基准映射状态
    float _leg_manual_cmd_offset[2]{};  // MID 摇杆量到平滑目标的映射偏置
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

    // 顶层 FSM 状态实例（实现见 Chassis/fsm/pyro_engineer_*_state.cpp）
    state_passive_t _state_passive;
    state_active_t  _state_active;
    fsm_t<owner>    _main_fsm;

    // =============================================
    // active 内部腿部 HFSM
    // 状态实现按 Hero 风格拆分到 Chassis/fsm/leg/*.cpp
    // =============================================
    struct leg_state_disabled_t final : public state_t<owner>
    {
        void enter(owner *chassis) override;
        void execute(owner *chassis) override;
        void exit(owner *chassis) override;
    };

    struct leg_state_uncalibrated_t final : public state_t<owner>
    {
        void enter(owner *chassis) override;
        void execute(owner *chassis) override;
        void exit(owner *chassis) override;
    };

    struct leg_state_vmc_t final : public state_t<owner>
    {
        void enter(owner *chassis) override;
        void execute(owner *chassis) override;
        void exit(owner *chassis) override;
    };

    struct leg_state_pid_gravity_t final : public state_t<owner>
    {
        void enter(owner *chassis) override;
        void execute(owner *chassis) override;
        void exit(owner *chassis) override;
    };

    struct leg_state_pid_home_t final : public state_t<owner>
    {
        void enter(owner *chassis) override;
        void execute(owner *chassis) override;
        void exit(owner *chassis) override;
    };

    leg_state_disabled_t     _leg_state_disabled;
    leg_state_uncalibrated_t _leg_state_uncalibrated;
    leg_state_vmc_t          _leg_state_vmc;
    leg_state_pid_gravity_t  _leg_state_pid_gravity;
    leg_state_pid_home_t     _leg_state_pid_home;
    fsm_t<owner>             _leg_fsm;
};

} // namespace pyro

#endif // __PYRO_ENGINEER_CHASSIS_H__
