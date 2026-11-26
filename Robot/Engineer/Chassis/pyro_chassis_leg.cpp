#include "pyro_engineer_chassis.h"
#include "pyro_leg_vmc.h"

namespace pyro
{
namespace
{
// Engineer 腿部局部角度展开：不修改 PYRo 通用算法库。
float unwrap_leg_angle(float previous_angle, float raw_angle)
{
    float delta = fmodf(raw_angle - previous_angle, 2.0f * PI);
    if (delta > PI)
    {
        delta -= 2.0f * PI;
    }
    else if (delta < -PI)
    {
        delta += 2.0f * PI;
    }
    return previous_angle + delta;
}
} // namespace

// =========================================================
// 腿部机构算法辅助函数
// =========================================================
// 腿失能：输出和积分全部清零，重新使能时不会携带旧力矩或积分冲击。
void engineer_chassis_t::_leg_handle_disabled(int i)
{
    _ctx.data.target_leg_theta[i] = 0.0f;
    _ctx.data.target_leg_radps[i] = 0.0f;

    _leg_reset_control_output(i);
    _leg_reset_leg_telemetry(i);

    _leg_target_smooth[i]     = 0.0f;
    _leg_zero_interp_init[i]  = false;

    if (_ctx.pid.leg_pos_pid[i]) _ctx.pid.leg_pos_pid[i]->clear();
    if (_ctx.pid.leg_vel_pid[i]) _ctx.pid.leg_vel_pid[i]->clear();
}

// 已使能但未校准：不输出力矩，等待右拨杆进入 UP 的边沿执行软件置零。
void engineer_chassis_t::_leg_handle_uncalibrated(int i)
{
    _leg_reset_control_output(i);
    _leg_reset_leg_telemetry(i);
}

void engineer_chassis_t::_leg_reset_control_output(int i)
{
    _ctx.data.out_leg_torque[i]         = 0.0f;
    _ctx.data.out_leg_gravity_torque[i] = 0.0f;
    _ctx.data.leg_tau_attitude[i]       = 0.0f;
    _leg_last_logic_torque[i]           = 0.0f;
}

void engineer_chassis_t::_leg_reset_leg_telemetry(int i)
{
    _ctx.telemetry.leg_alpha[i]          = 0.0f;
    _ctx.telemetry.leg_gravity_valid[i]  = false;
    _ctx.telemetry.leg_j_dynamic[i]      = 0.0f;
    _ctx.telemetry.leg_tau_damping[i]    = 0.0f;
    _ctx.telemetry.leg_ff_scale[i]       = 0.0f;
    _ctx.telemetry.leg_tau_manual[i]     = 0.0f;
    _ctx.telemetry.leg_control_mode[i]   = 0.0f;
}


// _calibrate_leg()：腿软件置零
void engineer_chassis_t::_calibrate_leg()
{

    _ctx.data.leg_theta_cal[0] = LEG_CALI_LEFT;
    _ctx.data.leg_theta_cal[1] = LEG_CALI_RIGHT;

    for (int i = 0; i < 2; i++)
    {
        if (!_ctx.motor.leg[i]) continue;

        // 把当前多圈角记为校准基准，相对角归零
        // _ctx.data.leg_theta_cal[i]     = _ctx.data.leg_theta_unwrap[i];
        _ctx.data.current_leg_theta[i] = 0.0f;
        _ctx.data.target_leg_theta[i]  = 0.0f;
        _ctx.data.target_leg_radps[i]  = 0.0f;
        _leg_target_smooth[i]          = 0.0f;
        _leg_zero_interp_init[i]       = false;
        _ctx.data.leg_calibrated[i]    = true;

        // 清 PID，避免切换瞬间积分突变
        if (_ctx.pid.leg_pos_pid[i]) _ctx.pid.leg_pos_pid[i]->clear();
        if (_ctx.pid.leg_vel_pid[i]) _ctx.pid.leg_vel_pid[i]->clear();
    }

    // 重新校准时同步清空姿态环，避免旧积分带入校准后的第一次 VMC
    if (_ctx.pid.pitch_pid) _ctx.pid.pitch_pid->clear();
    if (_ctx.pid.roll_pid)  _ctx.pid.roll_pid->clear();
}

//雅可比 以及腿部 X Y 坐标计算相关
leg_vmc_result_t calc_leg_xiaotan_vmc(float theta_raw, float pitch)
{
    leg_vmc_result_t result{};

    
    const float theta =
        fminf(fmaxf(theta_raw, LEG_MIN_POS), LEG_MAX_POS);
    result.theta = theta;

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

    // IMU 姿态：直接读取弧度，并减去固定安装角
    float yaw_rad   = 0.0f;
    float pitch_rad = 0.0f;
    float roll_rad  = 0.0f;
    ins_drv_t::get_instance()->get_rads_n(&yaw_rad, &pitch_rad, &roll_rad);

    const float raw_pitch = pitch_rad - PITCH_OFFSET_RAD;
    const float raw_roll  = roll_rad - ROLL_OFFSET_RAD;

    // 一阶低通：首帧直接采用原始值，避免启动时从 0 缓慢爬升
    if (!_imu_lpf_init)
    {
        _imu_pitch_lpf = raw_pitch;
        _imu_roll_lpf  = raw_roll;
        _imu_lpf_init  = true;
    }
    else
    {
        _imu_pitch_lpf =
            LEG_IMU_LPF_ALPHA * raw_pitch +
            (1.0f - LEG_IMU_LPF_ALPHA) * _imu_pitch_lpf;
        _imu_roll_lpf =
            LEG_IMU_LPF_ALPHA * raw_roll +
            (1.0f - LEG_IMU_LPF_ALPHA) * _imu_roll_lpf;
    }

    _ctx.telemetry.imu_pitch_raw = raw_pitch;
    _ctx.data.imu_pitch          = _imu_pitch_lpf;
    _ctx.telemetry.imu_roll_raw  = raw_roll;
    _ctx.data.imu_roll           = _imu_roll_lpf;

    for (int i = 0; i < 2; i++)
    {
        _ctx.motor.leg[i]->update_feedback();

        // 多圈展开：首帧对齐 raw，之后按带符号最短角增量累积。
        const float raw = _ctx.motor.leg[i]->get_current_position();
        _ctx.data.leg_theta_unwrap[i] =
            unwrap_leg_angle(_ctx.data.leg_theta_unwrap[i], raw);

        // 相对角（校准点为 0）：在反馈边界乘该腿唯一安装方向符号
        if (_ctx.data.leg_calibrated[i])
        {
            _ctx.data.current_leg_theta[i] =
                LEG_MOTOR_SIGN[i] *
                (_ctx.data.leg_theta_unwrap[i] - _ctx.data.leg_theta_cal[i]);
        }

        // 电机输出轴角速度：在反馈边界乘同一个安装方向符号
        _ctx.data.current_leg_radps[i] =
            LEG_MOTOR_SIGN[i] * _ctx.motor.leg[i]->get_current_rotate();
    }
}
// 任一有效腿控子状态进入瞬间：参考角从当前角起步并清双环积分，保证无扰切换。
void engineer_chassis_t::_leg_enter_active_mode(int i, float current_q)
{
    _leg_zero_interp_init[i]  = false;
    _leg_target_smooth[i]     = current_q;

    if (_ctx.pid.leg_pos_pid[i]) _ctx.pid.leg_pos_pid[i]->clear();
    if (_ctx.pid.leg_vel_pid[i]) _ctx.pid.leg_vel_pid[i]->clear();
}

// 纯计算：重力/Pitch-Roll 前馈；不直接写上下文，由状态文件保存结果。
leg_feedforward_output_t engineer_chassis_t::_leg_calc_feedforward(
    int i, float q, float qdot, float pitch,
    bool include_attitude, float f_pitch, float f_roll)
{
    leg_feedforward_output_t output{};

    const float theta_raw = LEG_Q_XT_SIGN * q;
    const leg_vmc_result_t vmc = calc_leg_xiaotan_vmc(theta_raw, pitch);

    const bool q_window_ok =
        q >= LEG_POLY_Q_VALID_MIN && q <= LEG_POLY_Q_VALID_MAX;

    const float gravity_q = LEG_Q_XT_SIGN * vmc.tau_gravity;
    output.gravity_valid =
        LEG_VMC_GRAVITY_EN && vmc.valid && q_window_ok &&
        fabsf(gravity_q) > 1e-5f;

    output.gravity_tau = output.gravity_valid
        ? fp32_constrain(gravity_q,
                         -LEG_POLY_GRAVITY_TAU_MAX,
                         LEG_POLY_GRAVITY_TAU_MAX)
        : 0.0f;

    // Pitch/Roll 虚拟力 -> theta 帧力矩 -> q 逻辑帧力矩。
    if (include_attitude)
    {
        const float roll_force_i = (i == 0) ? f_roll : -f_roll;
        const float attitude_tau_xt =
            vmc.j_dynamic * (f_pitch + roll_force_i);
        output.attitude_tau = LEG_Q_XT_SIGN * attitude_tau_xt;
    }

    output.theta_xt   = vmc.theta;
    output.j_dynamic  = vmc.j_dynamic;
    return output;
}

// 纯双环 PID 档明确撤掉所有 VMC/前馈观测量，避免 Live Watch 残留上一档数值。
void engineer_chassis_t::_leg_clear_feedforward(int i)
{
    _ctx.telemetry.leg_alpha[i]          = 0.0f;
    _ctx.telemetry.leg_j_dynamic[i]      = 0.0f;
    _ctx.data.out_leg_gravity_torque[i]  = 0.0f;
    _ctx.telemetry.leg_gravity_valid[i]  = false;
    _ctx.data.leg_tau_attitude[i]        = 0.0f;
    _ctx.telemetry.leg_ff_scale[i]       = 0.0f;
}

// 左 DOWN：最终目标为校准零点，但位置环参考按最大速度逐步接近，避免目标阶跃。
void engineer_chassis_t::_leg_update_home_target(int i, float current_q)
{
    if (!_leg_zero_interp_init[i])
    {
        _leg_target_smooth[i]    = current_q;
        _leg_zero_interp_init[i] = true;
    }

    constexpr float home_goal = 0.0f;
    const float goal_error    = home_goal - _leg_target_smooth[i];
    const float max_step      = LEG_ZERO_INTERP_VMAX * LEG_CONTROL_DT_S;
    const float interp_step   =
        fp32_constrain(goal_error, -max_step, max_step);

    _leg_target_smooth[i] += interp_step;

    // 参考角进入小死区后直接吸附到最终目标；位置环继续负责保持。
    if (fabsf(home_goal - _leg_target_smooth[i]) <= LEG_ZERO_INTERP_EPS)
    {
        _leg_target_smooth[i] = home_goal;
    }
}

// 位置环 -> 速度环；返回速度环 PID 输出的逻辑力矩。
float engineer_chassis_t::_leg_run_dual_pid(int i)
{
    _ctx.data.target_leg_theta[i] = _leg_target_smooth[i];
    _ctx.data.target_leg_radps[i] = 0.0f;

    if (_ctx.pid.leg_pos_pid[i])
    {
        _ctx.data.target_leg_radps[i] =
            _ctx.pid.leg_pos_pid[i]->calculate(
                _ctx.data.target_leg_theta[i],
                _ctx.data.current_leg_theta[i]);
    }

    float pid_tau = 0.0f;
    if (_ctx.pid.leg_vel_pid[i])
    {
        pid_tau = _ctx.pid.leg_vel_pid[i]->calculate(
            _ctx.data.target_leg_radps[i],
            _ctx.data.current_leg_radps[i]);
    }

    return pid_tau;
}

// 所有有效腿控模式共用：关节阻尼、总力矩限幅和 1ms 输出斜率限制。
void engineer_chassis_t::_leg_apply_common_output(
    int i, float raw_torque, float qdot)
{
    float tau_damping = -LEG_F_JOINT_DAMP_KD * qdot;
    tau_damping = fp32_constrain(
        tau_damping, -LEG_F_JOINT_DAMP_MAX, LEG_F_JOINT_DAMP_MAX);

    _ctx.telemetry.leg_tau_damping[i] = tau_damping;
    raw_torque += tau_damping;
    raw_torque = fp32_constrain(raw_torque, -LEG_MAX_TORQUE, LEG_MAX_TORQUE);

    // 抑制模式切换、有效域跳变和保护造成的力矩阶跃。
    float delta = raw_torque - _leg_last_logic_torque[i];
    delta = fp32_constrain(
        delta, -LEG_F_TORQUE_SLEW_STEP, LEG_F_TORQUE_SLEW_STEP);

    const float output_torque = fp32_constrain(
        _leg_last_logic_torque[i] + delta,
        -LEG_MAX_TORQUE,
        LEG_MAX_TORQUE);

    _leg_last_logic_torque[i]     = output_torque;
    _ctx.data.out_leg_torque[i]   = output_torque;
}

// =========================================================
// _leg_control()：腿部 active 子 HFSM 协调入口
// ---------------------------------------------------------
// 各状态的 enter/execute/exit 拆分在 Chassis/fsm/leg/*.cpp；
// 这里只负责把使能、校准和左拨杆请求映射为目标子状态，并驱动 PYRo FSM。
// 顶层 active/passive FSM 与 Chassis/fsm 下两个顶层状态文件保持不变。
// =========================================================
void engineer_chassis_t::_leg_control()
{
    state_t<owner> *target_state = &_leg_state_pid_home;

    if (!_ctx.cmd->leg.en)
    {
        target_state = &_leg_state_disabled;
    }
    else if (!_ctx.data.leg_calibrated[0] || !_ctx.data.leg_calibrated[1])
    {
        // 两腿共用同一左拨杆模式；任一条腿未完成校准时先进入安全等待态。
        target_state = &_leg_state_uncalibrated;
    }
    else
    {
        switch (_ctx.cmd->left_switch)
        {
            case left_switch_cmd_t::UP:
                target_state = &_leg_state_vmc;
                break;
            case left_switch_cmd_t::MID:
                target_state = &_leg_state_pid_gravity;
                break;
            case left_switch_cmd_t::DOWN:
                target_state = &_leg_state_pid_home;
                break;
        }
    }

    _leg_fsm.change_state(target_state);
    _leg_fsm.execute(this);
}

}