#include "pyro_engineer_chassis.h"

#include "pyro_algo_common.h"
#include "pyro_dji_motor_drv.h"
#include "pyro_ins.h"

#include <cmath>

namespace pyro
{

// =========================================================
// 小坦巡航 VMC 适配函数
// 输入 theta/theta_dot 为小坦站立角坐标系；输出 tau_gravity
// 仍为小坦 theta 帧力矩，由调用层做本车 q 帧符号适配。
// =========================================================
namespace
{
struct leg_xt_vmc_result_t
{
    float j_dynamic{};   // 姿态投影后的动态雅可比
    float tau_gravity{}; // 单腿重力补偿力矩（小坦 theta 帧）
    bool  valid{false};   // 力平衡分母是否有效
};

leg_xt_vmc_result_t calc_leg_xiaotan_vmc(float theta_raw, float pitch)
{
    leg_xt_vmc_result_t result{};

    // 多项式只在小坦实测拟合域内求值
    const float theta =
        fminf(fmaxf(theta_raw, LEG_XT_MIN_POS), LEG_XT_MAX_POS);

    const float cp = cosf(pitch);
    const float sp = sinf(pitch);

    const float j_x = evaluate_polynomial(
        theta, LEG_XT_JX_COEF, LEG_XT_JX_DEGREE);
    const float j_y = evaluate_polynomial(
        theta, LEG_XT_JY_COEF, LEG_XT_JY_DEGREE);
    const float x_b = evaluate_polynomial(
        theta, LEG_XT_XB_COEF, LEG_XT_XB_DEGREE);
    const float y_b = evaluate_polynomial(
        theta, LEG_XT_YB_COEF, LEG_XT_YB_DEGREE);

    // 小坦巡航 _leg_vmc() 前轴 O 力平衡
    const float y_wheel    = y_b - LEG_XT_H_HIP_OFFSET;
    const float numerator  = LEG_XT_DIST_FRONT * cp - LEG_XT_H_COG * sp;
    const float denominator =
        (LEG_XT_DIST_FRONT + LEG_XT_DIST_HIP + x_b) * cp + y_wheel * sp;

    float f_gravity_ff = 0.0f;
    if (fabsf(denominator) > 1e-4f)
    {
        f_gravity_ff =
            LEG_POLY_VEH_MASS * LEG_GRAVITY * numerator / denominator;
        result.valid = true;
    }

    result.j_dynamic   = j_y * cp + j_x * sp;
    result.tau_gravity = result.j_dynamic * (0.5f * f_gravity_ff);
    return result;
}
} // namespace

// IMU 一阶低通系数：每帧保留 15% 新采样、85% 历史值
static constexpr float LEG_IMU_LPF_ALPHA = 0.15f;

// =========================================================
// 构造函数
// =========================================================
engineer_chassis_t::engineer_chassis_t()
    : module_base_t("engineer_chassis", 512, 1024,
                    task_base_t::priority_t::HIGH)
{
    _ctx = {};
}

// =========================================================
// 获取上下文（非 const 版本，基类只有 const 版本）
// =========================================================
engineer_context_t &engineer_chassis_t::get_ctx()
{
    return _ctx;
}

// =========================================================
// _init()：初始化回调（只调用一次）
// =========================================================
status_t engineer_chassis_t::_init()
{
    // 1. 把外部依赖拷贝到上下文
    _ctx.motor = _module_deps.motor_deps;
    _ctx.pid   = _module_deps.pid_deps;

    // 2. 混合底盘运动学求解器（4 个独立 K，支持前后轮距非对称）
    _kinematics = new hybrid_kin_t(TRACK_SPACING, K_FL, K_FR, K_BL, K_BR);

    return PYRO_OK;
}

// =========================================================
// _update_feedback()：反馈更新（每周期先执行）
// =========================================================
void engineer_chassis_t::_update_feedback()
{
    // ---- 麦轮反馈 ----
    for (int i = 0; i < 4; i++)
    {
        _ctx.motor.mecanum[i]->update_feedback();

        // get_current_rotate() 为电机轴 rad/s，乘 1/减速比得轮轴 rad/s，
        // 再经 radps_to_rpm 转成轮轴 RPM，与目标侧量纲保持一致
        _ctx.data.current_wheel_rpm[i] = radps_to_rpm(
            _ctx.motor.mecanum[i]->get_current_rotate() *
            dji_m3508_motor_drv_t::reciprocal_reduction_ratio);

        _ctx.data.wheel_online[i] = _ctx.motor.mecanum[i]->is_online();
    }

    // 右侧两轮反馈取反：统一到逻辑正方向，使 PID 两侧方向一致
    _ctx.data.current_wheel_rpm[1] = -_ctx.data.current_wheel_rpm[1];
    _ctx.data.current_wheel_rpm[3] = -_ctx.data.current_wheel_rpm[3];

    // ---- 履带电机反馈（达妙 4310，输出轴 rad/s，不除减速比）----
    for (int i = 0; i < 2; i++)
    {
        if (!_ctx.motor.track[i]) continue;

        _ctx.motor.track[i]->update_feedback();
        _ctx.data.current_track_rpm[i] =
            radps_to_rpm(_ctx.motor.track[i]->get_current_rotate());
        _ctx.data.track_online[i] = _ctx.motor.track[i]->is_online();
    }

    // 右履带反馈取反：统一到逻辑正方向
    _ctx.data.current_track_rpm[1] = -_ctx.data.current_track_rpm[1];

    // ---- 腿部反馈、校准与 IMU ----
    _leg_feedback();
}

// =========================================================
// _apply_motor_enable()：按右拨杆命令边沿式使能/失能各电机
// =========================================================
void engineer_chassis_t::_apply_motor_enable()
{
    // 麦轮：右拨杆 MID/UP 使能
    for (int i = 0; i < 4; i++)
    {
        if (!_ctx.motor.mecanum[i]) continue;

        if (_ctx.cmd->mecanum_en && !_mec_en_state[i])
        {
            _ctx.motor.mecanum[i]->enable();
            _mec_en_state[i] = true;
        }
        else if (!_ctx.cmd->mecanum_en && _mec_en_state[i])
        {
            _ctx.motor.mecanum[i]->disable();
            _mec_en_state[i] = false;
        }
    }

    // 履带：右拨杆 UP 使能
    for (int i = 0; i < 2; i++)
    {
        if (!_ctx.motor.track[i]) continue;

        if (_ctx.cmd->track_en && !_track_en_state[i])
        {
            _ctx.motor.track[i]->enable();
            _track_en_state[i] = true;
        }
        else if (!_ctx.cmd->track_en && _track_en_state[i])
        {
            _ctx.motor.track[i]->disable();
            _track_en_state[i] = false;
        }
    }

    // 腿：右拨杆 UP 使能
    for (int i = 0; i < 2; i++)
    {
        if (!_ctx.motor.leg[i]) continue;

        if (_ctx.cmd->leg.en && !_leg_en_state[i])
        {
            _ctx.motor.leg[i]->enable();
            _leg_en_state[i] = true;
        }
        else if (!_ctx.cmd->leg.en && _leg_en_state[i])
        {
            _ctx.motor.leg[i]->disable();
            _leg_en_state[i] = false;
        }
    }
}

// =========================================================
// _kinematics_solve()：麦轮逆运动学
// =========================================================
void engineer_chassis_t::_kinematics_solve()
{
    // 当前履带不参与底盘 vx/vy/wz 解算，这里只解算 4 个麦轮
    const auto wheel_speeds = _kinematics->solve(
        _ctx.cmd->chassis.vx,
        _ctx.cmd->chassis.vy,
        _ctx.cmd->chassis.wz,
        false);

    // 线速度 m/s -> 轮轴 RPM（逻辑正方向，不在此取反）
    _ctx.data.target_wheel_rpm[0] = mps_to_rpm(wheel_speeds.mec_fl, WHEEL_RADIUS);
    _ctx.data.target_wheel_rpm[1] = mps_to_rpm(wheel_speeds.mec_fr, WHEEL_RADIUS);
    _ctx.data.target_wheel_rpm[2] = mps_to_rpm(wheel_speeds.mec_bl, WHEEL_RADIUS);
    _ctx.data.target_wheel_rpm[3] = mps_to_rpm(wheel_speeds.mec_br, WHEEL_RADIUS);
}

// =========================================================
// _mecanum_control()：麦轮速度环 PID
// =========================================================
void engineer_chassis_t::_mecanum_control()
{
    for (int i = 0; i < 4; i++)
    {
        _ctx.data.out_wheel_torque[i] = _ctx.pid.mecanum_pid[i]->calculate(
            _ctx.data.target_wheel_rpm[i],
            _ctx.data.current_wheel_rpm[i]);
    }
}

// =========================================================
// _track_control()：履带速度环 PID
// 当前履带目标固定为 0，仅保留速度闭环；左拨杆已复用为腿部 VMC 模式。
// 若后续恢复履带恒速/越障控制，应在此函数内重新生成 target_track_rpm。
// =========================================================
void engineer_chassis_t::_track_control()
{
    _ctx.data.target_track_rpm[0] = 0.0f;
    _ctx.data.target_track_rpm[1] = 0.0f;

    for (int i = 0; i < 2; i++)
    {
        if (!_ctx.pid.track_pid[i]) continue;

        _ctx.data.out_track_torque[i] = _ctx.pid.track_pid[i]->calculate(
            _ctx.data.target_track_rpm[i],
            _ctx.data.current_track_rpm[i]);
    }
}

// =========================================================
// _calibrate_leg()：腿软件置零（小坦式校准，非堵转）
// =========================================================
void engineer_chassis_t::_calibrate_leg()
{
    for (int i = 0; i < 2; i++)
    {
        if (!_ctx.motor.leg[i]) continue;

        // 把当前多圈角记为校准基准，相对角归零
        _ctx.data.leg_theta_cal[i]     = _ctx.data.leg_theta_unwrap[i];
        _ctx.data.current_leg_theta[i] = 0.0f;
        _ctx.data.target_leg_theta[i]  = 0.0f;
        _ctx.data.target_leg_radps[i]  = 0.0f;
        _leg_target_smooth[i]          = 0.0f;
        _leg_manual_map_active[i]       = false;
        _leg_manual_cmd_offset[i]      = 0.0f;
        _ctx.data.leg_calibrated[i]    = true;

        // 清 PID，避免切换瞬间积分突变
        if (_ctx.pid.leg_pos_pid[i]) _ctx.pid.leg_pos_pid[i]->clear();
        if (_ctx.pid.leg_vel_pid[i]) _ctx.pid.leg_vel_pid[i]->clear();
    }
}

// =========================================================
// _leg_feedback()：腿反馈（unwrap、校准、IMU）
// =========================================================
void engineer_chassis_t::_leg_feedback()
{
    // 校准脉冲（右拨杆进入 UP 的边沿触发一次）
    if (_ctx.cmd->leg.calibrate)
    {
        _calibrate_leg();
    }

    // IMU 姿态：get_angles_n 返回度，pitch 转弧度并减固定安装角
    float yaw_deg = 0.0f;
    float pitch_deg = 0.0f;
    float roll_deg = 0.0f;
    ins_drv_t::get_instance()->get_angles_n(&yaw_deg, &pitch_deg, &roll_deg);

    const float raw_pitch = pitch_deg * PI / 180.0f - PITCH_OFFSET_RAD;

    // 一阶低通：首帧直接采用原始值，避免启动时从 0 缓慢爬升
    if (!_imu_lpf_init)
    {
        _imu_pitch_lpf = raw_pitch;
        _imu_lpf_init  = true;
    }
    else
    {
        _imu_pitch_lpf =
            LEG_IMU_LPF_ALPHA * raw_pitch +
            (1.0f - LEG_IMU_LPF_ALPHA) * _imu_pitch_lpf;
    }

    _ctx.data.imu_pitch_raw = raw_pitch;
    _ctx.data.imu_pitch     = _imu_pitch_lpf;

    for (int i = 0; i < 2; i++)
    {
        _ctx.motor.leg[i]->update_feedback();
        _ctx.data.leg_online[i] = _ctx.motor.leg[i]->is_online();

        // 多圈 unwrap：首帧直接初始化，之后按带符号最短角增量累积
        const float raw = _ctx.motor.leg[i]->get_current_position();
        _ctx.data.leg_raw_position[i] = raw;

        if (!_leg_fb_init[i])
        {
            _last_leg_raw[i] = raw;
            _leg_fb_init[i]  = true;
        }

        float delta = raw - _last_leg_raw[i];
        if (delta > PI)
        {
            delta -= 2.0f * PI;
        }
        else if (delta < -PI)
        {
            delta += 2.0f * PI;
        }

        _ctx.data.leg_theta_unwrap[i] += delta;
        _last_leg_raw[i] = raw;

        // 相对角（校准点为 0），按逻辑方向取符号（右腿镜像取反）
        if (_ctx.data.leg_calibrated[i])
        {
            _ctx.data.current_leg_theta[i] =
                LEG_LOGIC_SIGN[i] *
                (_ctx.data.leg_theta_unwrap[i] - _ctx.data.leg_theta_cal[i]);
        }

        // 电机输出轴角速度（按逻辑方向取符号，右腿镜像取反）
        _ctx.data.current_leg_radps[i] =
            LEG_LOGIC_SIGN[i] * _ctx.motor.leg[i]->get_current_rotate();
    }
}

// =========================================================
// _leg_control()：腿控制
// 1. 左拨杆 UP：小坦巡航 VMC 重力补偿 + 窗外安全墙 + 全模式阻尼。
// 2. 左拨杆 MID/DOWN：当前角保持型手动双环；摇杆下推时做无扰增量映射。
// =========================================================
void engineer_chassis_t::_leg_control()
{
    const float pitch    = _ctx.data.imu_pitch;
    const bool  vmc_mode = _ctx.cmd->leg_vmc_mode;

    for (int i = 0; i < 2; i++)
    {
        // ---- 腿未使能：目标/输出清零、清 PID，腿以 0 力矩松开 ----
        if (!_ctx.cmd->leg.en)
        {
            _ctx.data.target_leg_theta[i]       = 0.0f;
            _ctx.data.target_leg_radps[i]       = 0.0f;
            _ctx.data.out_leg_torque[i]         = 0.0f;
            _ctx.data.out_leg_gravity_torque[i] = 0.0f;
            _ctx.data.leg_tau_wall[i]           = 0.0f;
            _ctx.data.leg_tau_damping[i]        = 0.0f;
            _ctx.data.leg_ff_scale[i]           = 0.0f;
            _ctx.data.leg_tau_manual_pid[i]     = 0.0f;
            _ctx.data.leg_control_mode[i]       = 0.0f;

            _leg_target_smooth[i]      = 0.0f;
            _leg_manual_map_active[i]  = false;
            _leg_manual_cmd_offset[i]  = 0.0f;
            _leg_last_logic_torque[i]  = 0.0f;

            if (_ctx.pid.leg_pos_pid[i]) _ctx.pid.leg_pos_pid[i]->clear();
            if (_ctx.pid.leg_vel_pid[i]) _ctx.pid.leg_vel_pid[i]->clear();
            continue;
        }

        // ---- 使能但尚未校准：不输出，等待右拨杆 UP 边沿校准 ----
        if (!_ctx.data.leg_calibrated[i])
        {
            _ctx.data.out_leg_torque[i]         = 0.0f;
            _ctx.data.out_leg_gravity_torque[i] = 0.0f;
            _ctx.data.leg_gravity_valid[i]      = false;
            _ctx.data.leg_control_mode[i]       = 0.0f;
            _leg_last_logic_torque[i]           = 0.0f;
            continue;
        }

        const float q    = _ctx.data.current_leg_theta[i];
        const float qdot = _ctx.data.current_leg_radps[i];

        // UP<->MID 任一边沿都先做无扰切换：目标同步当前角、清 PID 积分
        const bool mode_edge = (vmc_mode != _leg_vmc_prev);
        if (mode_edge)
        {
            _leg_target_smooth[i]     = q;
            _leg_manual_map_active[i] = false;
            _leg_manual_cmd_offset[i] = 0.0f;

            if (_ctx.pid.leg_pos_pid[i]) _ctx.pid.leg_pos_pid[i]->clear();
            if (_ctx.pid.leg_vel_pid[i]) _ctx.pid.leg_vel_pid[i]->clear();
        }

        // =================================================
        // 小坦巡航 VMC 重力链（当前唯一运行的重力补偿链）
        // =================================================
        leg_xt_vmc_result_t vmc{};
        float gravity_tau   = 0.0f;
        float vmc_wall_tau  = 0.0f;
        bool  gravity_enable = false;

        // 多项式有效窗外共用的下端制动墙；起步 q≈0 死区不做主动推腿墙
        if (q < LEG_POLY_Q_VALID_MIN)
        {
            vmc_wall_tau =
                LEG_POLY_WALL_K * (LEG_POLY_Q_VALID_MIN - q) -
                LEG_POLY_WALL_D * qdot;
            vmc_wall_tau = fp32_constrain(
                vmc_wall_tau, 0.0f, LEG_POLY_WALL_MAX);
        }

        const float theta_raw  = LEG_POLY_THETA_SIGN * q + LEG_POLY_THETA_OFFSET;
        const float theta_eval = fp32_constrain(
            theta_raw, LEG_POLY_THETA_MIN, LEG_POLY_THETA_MAX);

        vmc = calc_leg_xiaotan_vmc(theta_eval, pitch);

        const bool q_window_ok =
            (q >= LEG_POLY_Q_VALID_MIN && q <= LEG_POLY_Q_VALID_MAX &&
             theta_raw >= LEG_POLY_THETA_MIN &&
             theta_raw <= LEG_POLY_THETA_MAX);

        // 小坦原生物理输出为左 -tau_XT / 右 +tau_XT；当前统一 q 逻辑帧
        // 经发送层 LEG_LOGIC_SIGN 后，两腿算法输出均使用 LEG_POLY_OUTPUT_SIGN。
        const float poly_gravity_q =
            LEG_POLY_OUTPUT_SIGN * LEG_POLY_GRAVITY_SCALE * vmc.tau_gravity;

        gravity_enable =
            LEG_VMC_GRAVITY_EN && vmc.valid && q_window_ok &&
            (fabsf(poly_gravity_q) > 1e-5f);

        gravity_tau = gravity_enable
            ? fp32_constrain(poly_gravity_q,
                             -LEG_POLY_GRAVITY_TAU_MAX,
                             LEG_POLY_GRAVITY_TAU_MAX)
            : 0.0f;

        // VMC 诊断量
        _ctx.data.leg_alpha[i]                = theta_eval;
        _ctx.data.leg_j_dynamic[i]            = vmc.j_dynamic;
        _ctx.data.leg_tau_wall[i]             = vmc_wall_tau;
        _ctx.data.leg_ff_scale[i]             = 1.0f;
        _ctx.data.out_leg_gravity_torque[i]   = gravity_tau;
        _ctx.data.leg_gravity_valid[i]        = gravity_enable;

        float raw_torque     = 0.0f;
        float manual_pid_tau = 0.0f;

        if (vmc_mode)
        {
            // VMC 模式不追摇杆位置目标；平滑目标持续跟随当前 q，
            // 这样切回 MID 的第一帧位置环目标就是当前腿角。
            _leg_target_smooth[i]            = q;
            _ctx.data.target_leg_theta[i]   = q;
            _ctx.data.target_leg_radps[i]   = 0.0f;
            _ctx.data.leg_tau_manual_pid[i] = 0.0f;
            _ctx.data.leg_control_mode[i]   = 2.0f;

            raw_torque = gravity_tau + vmc_wall_tau;
        }
        else
        {
            // 手动模式采用“中心保持 + 下推增量映射”：
            // 第一次推摇杆时把当前平滑目标作为基准，避免中位命令 0 造成目标跳变。
            const float stick_cmd = _ctx.cmd->leg.target_theta;

            if (stick_cmd < -LEG_MANUAL_STICK_EPS)
            {
                if (!_leg_manual_map_active[i])
                {
                    _leg_manual_map_active[i] = true;
                    _leg_manual_cmd_offset[i] =
                        _leg_target_smooth[i] - stick_cmd;
                }

                float desired = stick_cmd + _leg_manual_cmd_offset[i];
                desired = fp32_constrain(desired, LEG_MANUAL_MIN, LEG_MANUAL_MAX);

                _leg_target_smooth[i] +=
                    (desired - _leg_target_smooth[i]) * LEG_TARGET_LERP;
                _leg_target_smooth[i] = fp32_constrain(
                    _leg_target_smooth[i], LEG_MANUAL_MIN, LEG_MANUAL_MAX);
            }
            else
            {
                // 摇杆回中：冻结并保持当前目标，不自动跳回校准收起位
                _leg_manual_map_active[i] = false;
            }

            _ctx.data.target_leg_theta[i] = _leg_target_smooth[i];

            if (_ctx.pid.leg_pos_pid[i])
            {
                _ctx.data.target_leg_radps[i] =
                    _ctx.pid.leg_pos_pid[i]->calculate(
                        _ctx.data.target_leg_theta[i],
                        _ctx.data.current_leg_theta[i]);
            }

            if (_ctx.pid.leg_vel_pid[i])
            {
                manual_pid_tau = _ctx.pid.leg_vel_pid[i]->calculate(
                    _ctx.data.target_leg_radps[i],
                    _ctx.data.current_leg_radps[i]);
            }

            _ctx.data.leg_tau_manual_pid[i] = manual_pid_tau;
            _ctx.data.leg_control_mode[i]   = 1.0f;

            // 操作员主动下推摇杆时，重力诊断/前馈临时打折；回中保持时保留防摔前馈
            if (_leg_manual_map_active[i])
            {
                gravity_tau *= LEG_MANUAL_CMD_FF_SCALE;
                _ctx.data.leg_ff_scale[i] = LEG_MANUAL_CMD_FF_SCALE;
                _ctx.data.out_leg_gravity_torque[i] = gravity_tau;
            }

            raw_torque = manual_pid_tau;

            // 调试期默认关闭手动保持段重力叠加，确保 MID 手动 PID 可单独辨识
            if (LEG_MANUAL_GRAVITY_FF)
            {
                raw_torque += gravity_tau;
            }
        }

        // ---- 全模式 q 空间阻尼：只抑制瞬时窜动，不负责位置保持 ----
        float tau_damping = -LEG_F_JOINT_DAMP_KD * qdot;
        tau_damping = fp32_constrain(
            tau_damping, -LEG_F_JOINT_DAMP_MAX, LEG_F_JOINT_DAMP_MAX);

        _ctx.data.leg_tau_damping[i] = tau_damping;
        raw_torque += tau_damping;

        raw_torque = fp32_constrain(raw_torque, -LEG_MAX_TORQUE, LEG_MAX_TORQUE);

        // ---- 输出斜率限制：抑制模式切换、有效域跳变和保护造成的力矩阶跃 ----
        float delta = raw_torque - _leg_last_logic_torque[i];
        delta = fp32_constrain(
            delta, -LEG_F_TORQUE_SLEW_STEP, LEG_F_TORQUE_SLEW_STEP);

        const float output_torque = fp32_constrain(
            _leg_last_logic_torque[i] + delta,
            -LEG_MAX_TORQUE,
            LEG_MAX_TORQUE);

        _leg_last_logic_torque[i] = output_torque;
        _ctx.data.out_leg_torque[i] = output_torque;
    }

    _leg_vmc_prev = vmc_mode;
}

// =========================================================
// _send_motor_command()：发送所有电机指令
// =========================================================
void engineer_chassis_t::_send_motor_command() const
{
    // 右侧麦轮发送时取反，把逻辑方向转回电调物理方向
    _ctx.motor.mecanum[0]->send_torque(_ctx.data.out_wheel_torque[0]);
    _ctx.motor.mecanum[1]->send_torque(-_ctx.data.out_wheel_torque[1]);
    _ctx.motor.mecanum[2]->send_torque(_ctx.data.out_wheel_torque[2]);
    _ctx.motor.mecanum[3]->send_torque(-_ctx.data.out_wheel_torque[3]);

    // 右履带发送取反，把逻辑方向转回物理方向
    if (_ctx.motor.track[0])
    {
        _ctx.motor.track[0]->send_torque(_ctx.data.out_track_torque[0]);
    }
    if (_ctx.motor.track[1])
    {
        _ctx.motor.track[1]->send_torque(-_ctx.data.out_track_torque[1]);
    }

    // 腿统一在逻辑 q 坐标系输出，发送层只在这里按 LEG_LOGIC_SIGN
    // 转回电调物理方向，避免双重反向。
    for (int i = 0; i < 2; i++)
    {
        if (_ctx.motor.leg[i])
        {
            const float leg_send_torque =
                LEG_LOGIC_SIGN[i] * _ctx.data.out_leg_torque[i];
            _ctx.motor.leg[i]->send_torque(leg_send_torque);
        }
    }
}

// =========================================================
// _fsm_execute()：状态机执行
// =========================================================
void engineer_chassis_t::_fsm_execute()
{
    // 1. 把当前命令指针存到上下文
    _ctx.cmd = &_current_cmd;

    // 2. 根据 mode 切换顶层状态
    if (cmd_base_t::mode_t::ACTIVE == _current_cmd.mode)
    {
        _main_fsm.change_state(&_state_active);
    }
    else
    {
        _main_fsm.change_state(&_state_passive);
    }

    // 3. 执行当前状态
    _main_fsm.execute(this);
}

} // namespace pyro
