#include "pyro_engineer_chassis.h"
#include "pyro_leg_vmc.h"

namespace pyro
{
// =========================================================
// 腿部机构算法辅助函数
// =========================================================
// 腿失能：输出和积分全部清零，重新使能时不会携带旧力矩或积分冲击。
void engineer_chassis_t::_leg_handle_disabled(int i)
{
    _ctx.data.target_leg_theta[i]       = 0.0f;
    _ctx.data.target_leg_radps[i]       = 0.0f;
    _ctx.data.out_leg_torque[i]         = 0.0f;
    _ctx.data.out_leg_gravity_torque[i] = 0.0f;
    _ctx.data.leg_alpha[i]              = 0.0f;
    _ctx.data.leg_gravity_valid[i]      = false;
    _ctx.data.leg_j_dynamic[i]          = 0.0f;
    _ctx.data.leg_tau_wall[i]           = 0.0f;
    _ctx.data.leg_tau_damping[i]        = 0.0f;
    _ctx.data.leg_ff_scale[i]           = 0.0f;
    _ctx.data.leg_tau_manual_pid[i]     = 0.0f;
    _ctx.data.leg_tau_attitude[i]       = 0.0f;
    _ctx.data.leg_control_mode[i]       = 0.0f;

    _leg_target_smooth[i]         = 0.0f;
    _leg_zero_interp_init[i]      = false;
    _leg_manual_map_active[i]     = false;
    _leg_manual_cmd_offset[i]     = 0.0f;
    _leg_last_logic_torque[i]     = 0.0f;

    if (_ctx.pid.leg_pos_pid[i]) _ctx.pid.leg_pos_pid[i]->clear();
    if (_ctx.pid.leg_vel_pid[i]) _ctx.pid.leg_vel_pid[i]->clear();
}

// 已使能但未校准：不输出力矩，等待右拨杆进入 UP 的边沿执行软件置零。
void engineer_chassis_t::_leg_handle_uncalibrated(int i)
{
    _ctx.data.out_leg_torque[i]         = 0.0f;
    _ctx.data.out_leg_gravity_torque[i] = 0.0f;
    _ctx.data.leg_alpha[i]              = 0.0f;
    _ctx.data.leg_gravity_valid[i]      = false;
    _ctx.data.leg_j_dynamic[i]          = 0.0f;
    _ctx.data.leg_tau_wall[i]           = 0.0f;
    _ctx.data.leg_tau_damping[i]        = 0.0f;
    _ctx.data.leg_ff_scale[i]           = 0.0f;
    _ctx.data.leg_tau_manual_pid[i]     = 0.0f;
    _ctx.data.leg_tau_attitude[i]       = 0.0f;
    _ctx.data.leg_control_mode[i]       = 0.0f;
    _leg_last_logic_torque[i]           = 0.0f;
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
        _leg_manual_map_active[i]       = false;
        _leg_manual_cmd_offset[i]      = 0.0f;
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

    _ctx.data.imu_pitch_raw = raw_pitch;
    _ctx.data.imu_pitch     = _imu_pitch_lpf;
    _ctx.data.imu_roll_raw  = raw_roll;
    _ctx.data.imu_roll      = _imu_roll_lpf;

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
    _leg_manual_map_active[i] = false;
    _leg_manual_cmd_offset[i] = 0.0f;
    _leg_zero_interp_init[i]  = false;
    _leg_target_smooth[i]     = current_q;

    if (_ctx.pid.leg_pos_pid[i]) _ctx.pid.leg_pos_pid[i]->clear();
    if (_ctx.pid.leg_vel_pid[i]) _ctx.pid.leg_vel_pid[i]->clear();
}

// 小坦巡航 VMC 重力链 + q 空间软墙；include_attitude=true 时再叠加 Pitch/Roll 映射力矩。
void engineer_chassis_t::_leg_update_vmc_feedforward(
    int i, float q, float qdot, float pitch,
    bool include_attitude, float f_pitch, float f_roll)
{
    float vmc_wall_tau = 0.0f;

    // 多项式有效窗外共用的下端制动墙；起步 q≈0 死区不做主动推腿墙。
    if (q < LEG_POLY_Q_VALID_MIN)
    {
        vmc_wall_tau =
            LEG_POLY_WALL_K * (LEG_POLY_Q_VALID_MIN - q) -
            LEG_POLY_WALL_D * qdot;
        vmc_wall_tau = fp32_constrain(
            vmc_wall_tau, 0.0f, LEG_POLY_WALL_MAX);
    }

    const float theta_raw  = LEG_Q_XT_SIGN * q + LEG_POLY_THETA_OFFSET;
    const float theta_eval = fp32_constrain(
        theta_raw, LEG_POLY_THETA_MIN, LEG_POLY_THETA_MAX);

    const leg_vmc_result_t vmc = calc_leg_xiaotan_vmc(theta_eval, pitch);

    const bool q_window_ok =
        (q >= LEG_POLY_Q_VALID_MIN && q <= LEG_POLY_Q_VALID_MAX &&
         theta_raw >= LEG_POLY_THETA_MIN &&
         theta_raw <= LEG_POLY_THETA_MAX);

    // theta 帧重力矩按线性坐标变换映射回 q 帧；
    // 左右腿物理镜像统一留给发送边界的 LEG_MOTOR_SIGN。
    const float poly_gravity_q =
        LEG_Q_XT_SIGN * LEG_POLY_GRAVITY_SCALE * vmc.tau_gravity;

    const bool gravity_enable =
        LEG_VMC_GRAVITY_EN && vmc.valid && q_window_ok &&
        (fabsf(poly_gravity_q) > 1e-5f);

    const float gravity_tau = gravity_enable
        ? fp32_constrain(poly_gravity_q,
                         -LEG_POLY_GRAVITY_TAU_MAX,
                         LEG_POLY_GRAVITY_TAU_MAX)
        : 0.0f;

    // Pitch/Roll 虚拟力 -> 小坦 theta 帧力矩 -> 本车 q 逻辑帧力矩。
    // Roll 左右腿反号，与小坦 _leg_vmc() 保持一致；q->电机物理方向在发送层处理。
    float attitude_tau_q = 0.0f;
    if (include_attitude)
    {
        const float roll_force_i = (i == 0) ? f_roll : -f_roll;
        const float tau_attitude_xt =
            vmc.j_dynamic * (f_pitch + roll_force_i);
        attitude_tau_q = LEG_Q_XT_SIGN * tau_attitude_xt;
    }

    _ctx.data.leg_alpha[i]              = theta_eval;
    _ctx.data.leg_j_dynamic[i]          = vmc.j_dynamic;
    _ctx.data.leg_tau_wall[i]           = vmc_wall_tau;
    _ctx.data.leg_tau_attitude[i]       = attitude_tau_q;
    _ctx.data.leg_ff_scale[i]           = 1.0f;
    _ctx.data.out_leg_gravity_torque[i] = gravity_tau;
    _ctx.data.leg_gravity_valid[i]      = gravity_enable;
}

// 纯双环 PID 档明确撤掉所有 VMC/前馈观测量，避免 Live Watch 残留上一档数值。
void engineer_chassis_t::_leg_clear_feedforward(int i)
{
    _ctx.data.leg_alpha[i]              = 0.0f;
    _ctx.data.leg_j_dynamic[i]          = 0.0f;
    _ctx.data.out_leg_gravity_torque[i] = 0.0f;
    _ctx.data.leg_gravity_valid[i]      = false;
    _ctx.data.leg_tau_wall[i]           = 0.0f;
    _ctx.data.leg_tau_attitude[i]       = 0.0f;
    _ctx.data.leg_ff_scale[i]           = 0.0f;
}

// 左 MID：右摇杆下推增量映射；摇杆回中后冻结当前平滑目标，不自动跳回零点。
void engineer_chassis_t::_leg_update_manual_target(int i, float stick_cmd)
{
    _leg_zero_interp_init[i] = false;

    if (stick_cmd < -LEG_MANUAL_STICK_EPS)
    {
        // 第一次推摇杆时把当前平滑目标作为基准，避免中位命令 0 造成目标跳变。
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
        _leg_manual_map_active[i] = false;
    }
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

    _leg_manual_map_active[i] = false;
    _leg_manual_cmd_offset[i] = 0.0f;
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

    _ctx.data.leg_tau_damping[i] = tau_damping;
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
    const bool vmc_request        = _ctx.cmd->track_run;
    const bool pid_gravity_request = _ctx.cmd->leg.home_to_zero;

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
    else if (vmc_request)
    {
        target_state = &_leg_state_vmc;
    }
    else if (pid_gravity_request)
    {
        target_state = &_leg_state_pid_gravity;
    }
    else
    {
        target_state = &_leg_state_pid_home;
    }

    _leg_fsm.change_state(target_state);
    _leg_fsm.execute(this);
}

}