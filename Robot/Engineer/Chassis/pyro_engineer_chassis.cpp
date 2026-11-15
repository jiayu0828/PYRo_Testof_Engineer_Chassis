#include "pyro_engineer_chassis.h"

#include "pyro_algo_common.h"
#include "pyro_dji_motor_drv.h"
#include "pyro_ins.h"

namespace pyro
{

// IMU 一阶低通系数：与小坦一致，每帧保留 15% 新采样、85% 历史值
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
// _init()：初始化回调（只调用一次）
// =========================================================
status_t engineer_chassis_t::_init()
{
    // 1. 把外部依赖拷贝到上下文
    _ctx.motor = _module_deps.motor_deps;
    _ctx.pid   = _module_deps.pid_deps;

    // 2. 混合底盘运动学求解器（4 个独立 K，支持前后轮距非对称）
    _kinematics = new hybrid_kin_t(TRACK_SPACING, K_FL, K_FR, K_BL, K_BR);

    // 3. 后腿四杆正运动学（几何全部来自图纸实测）
    const float b[2]       = {LEG_B_X, LEG_B_Y};
    const float c[2]       = {LEG_C_X, LEG_C_Y};
    const float f_local[2] = {LEG_F_LOCAL_X, LEG_F_LOCAL_Y};
    const float w_local[2] = {LEG_W_LOCAL_X, LEG_W_LOCAL_Y};
    _leg_kinematics = new leg_kin_t(b, c, LEG_L_BE, LEG_L_ED, LEG_L_DC,
                                    f_local, w_local, LEG_BRANCH);

    return PYRO_OK;
}

// =========================================================
// _update_feedback()：反馈更新（每周期先执行）
// =========================================================
void engineer_chassis_t::_update_feedback()
{
    for (int i = 0; i < 4; i++)
    {
        _ctx.motor.mecanum[i]->update_feedback();

        // get_current_rotate() 为电机轴 rad/s，乘 1/减速比 得轮轴 rad/s，
        // 再经 radps_to_rpm 转成轮轴 RPM，与目标侧量纲保持一致
        _ctx.data.current_wheel_rpm[i] = radps_to_rpm(
            _ctx.motor.mecanum[i]->get_current_rotate() *
            dji_m3508_motor_drv_t::reciprocal_reduction_ratio);

        _ctx.data.wheel_online[i] = _ctx.motor.mecanum[i]->is_online();
    }

    // 右侧两轮反馈取反：统一到逻辑正方向，使 PID 两侧方向一致
    _ctx.data.current_wheel_rpm[1] = -_ctx.data.current_wheel_rpm[1];
    _ctx.data.current_wheel_rpm[3] = -_ctx.data.current_wheel_rpm[3];
    // ---- 履带电机（达妙 4310）----
    // get_current_rotate() 已是输出轴 rad/s，直接转驱动轮 RPM，不除减速比
    for (int i = 0; i < 2; i++)
    {
        if (!_ctx.motor.track[i]) continue;
        _ctx.motor.track[i]->update_feedback();
        _ctx.data.current_track_rpm[i] =
            radps_to_rpm(_ctx.motor.track[i]->get_current_rotate());
        _ctx.data.track_online[i] = _ctx.motor.track[i]->is_online();
    }
    // 右履带反馈取反：统一到逻辑正方向，使 PID 两侧方向一致
    _ctx.data.current_track_rpm[1] = -_ctx.data.current_track_rpm[1];

    // 腿部反馈（unwrap、校准、正运动学、IMU）
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
    // 履带已与 vx 解耦（恒速由左拨杆控制），这里只解算麦轮，track_en 传 false
    const auto wheel_speeds = _kinematics->solve(
        _ctx.cmd->chassis.vx, _ctx.cmd->chassis.vy, _ctx.cmd->chassis.wz,
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
            _ctx.data.target_wheel_rpm[i], _ctx.data.current_wheel_rpm[i]);
    }
}

// =========================================================
// _track_control()：履带速度环 PID
// =========================================================
void engineer_chassis_t::_track_control()
{
    // 当前速度目标固定为 0，仅保留速度闭环。
    _ctx.data.target_track_rpm[0] = 0.0f;
    _ctx.data.target_track_rpm[1] = 0.0f;

    for (int i = 0; i < 2; i++)
    {
        if (!_ctx.pid.track_pid[i]) continue;
        _ctx.data.out_track_torque[i] = _ctx.pid.track_pid[i]->calculate(
            _ctx.data.target_track_rpm[i], _ctx.data.current_track_rpm[i]);
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
        _leg_target_smooth[i]          = 0.0f; // 平滑目标回到校准位
        _ctx.data.leg_calibrated[i]    = true;

        // 清 PID，避免切换瞬间积分突变
        if (_ctx.pid.leg_pos_pid[i]) _ctx.pid.leg_pos_pid[i]->clear();
        if (_ctx.pid.leg_vel_pid[i]) _ctx.pid.leg_vel_pid[i]->clear();
    }
}

// =========================================================
// _leg_feedback()：腿反馈（unwrap、校准、正运动学、IMU）
// =========================================================
void engineer_chassis_t::_leg_feedback()
{
    // 校准脉冲（右拨杆进入 UP 的边沿触发一次）
    if (_ctx.cmd->leg.calibrate)
    {
        _calibrate_leg();
    }

    // IMU 姿态：get_angles_n 返回度，pitch 转弧度并减固定安装角（车水平时读数为0）
    float yaw_deg = 0.0f, pitch_deg = 0.0f, roll_deg = 0.0f;
    ins_drv_t::get_instance()->get_angles_n(&yaw_deg, &pitch_deg, &roll_deg);
    const float raw_pitch = pitch_deg * PI / 180.0f - PITCH_OFFSET_RAD;

    // 一阶低通：首帧直接采用原始值，避免启动时从 0 缓慢爬升；
    // 原始值和滤波值同时保留，控制只使用滤波值
    if (!_imu_lpf_init)
    {
        _imu_pitch_lpf = raw_pitch;
        _imu_lpf_init  = true;
    }
    else
    {
        _imu_pitch_lpf += LEG_IMU_LPF_ALPHA * (raw_pitch - _imu_pitch_lpf);
    }
    _ctx.data.imu_pitch_raw = raw_pitch;
    _ctx.data.imu_pitch     = _imu_pitch_lpf;

    for (int i = 0; i < 2; i++)
    {
        if (!_ctx.motor.leg[i]) continue;
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
            delta -= 2.0f * PI;
        else if (delta < -PI)
            delta += 2.0f * PI;
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

        // 正运动学：电机相对角 -> 曲柄角 -> 四杆位姿
        const float theta_rel =
            _ctx.data.leg_theta_unwrap[i] - _ctx.data.leg_theta_cal[i];
        const float alpha =
            LEG_CAL_ALPHA +
            static_cast<float>(LEG_MOTOR_DIR[i]) * theta_rel / LEG_EXTERNAL_RATIO;
        _ctx.data.leg_alpha[i] = alpha;

        const auto pose = _leg_kinematics->forward(alpha);
        _ctx.data.leg_pose_valid[i] = pose.valid;
        if (pose.valid)
        {
            _ctx.data.leg_wx[i] = pose.w_x;
            _ctx.data.leg_wy[i] = pose.w_y;
        }
        else
        {
            // 禁止无效姿态继续沿用上一帧轮心，重力有效性标志会同步置假
            _ctx.data.leg_wx[i] = 0.0f;
            _ctx.data.leg_wy[i] = 0.0f;
        }
    }
}

// =========================================================
// _leg_control()：腿控制
// 右拨杆 UP 使能腿；履带启动（左拨杆 UP）输出纯重力前馈，
// 履带停止时跑手动双环（位置环 -> 速度环）；未使能则清零、0 力矩松开
// =========================================================
void engineer_chassis_t::_leg_control()
{
    const float pitch = _ctx.data.imu_pitch;

    // 履带启动（左拨杆 UP）-> 纯重力前馈；否则手动双环
    const bool vmc_mode = _ctx.cmd->track_run;

    for (int i = 0; i < 2; i++)
    {
        if (!_ctx.motor.leg[i]) continue;

        // 腿未使能：目标/输出清零、清 PID，腿以 0 力矩松开
        if (!_ctx.cmd->leg.en)
        {
            _ctx.data.target_leg_theta[i] = 0.0f;
            _ctx.data.target_leg_radps[i] = 0.0f;
            _ctx.data.out_leg_torque[i]         = 0.0f;
            _ctx.data.out_leg_gravity_torque[i] = 0.0f;
            _ctx.data.leg_j_vertical[i]         = 0.0f;
            _ctx.data.leg_support_arm[i]        = 0.0f;
            _ctx.data.leg_n_each[i]             = 0.0f;
            _ctx.data.leg_gravity_valid[i]      = false;
            _leg_target_smooth[i]               = 0.0f;
            if (_ctx.pid.leg_pos_pid[i]) _ctx.pid.leg_pos_pid[i]->clear();
            if (_ctx.pid.leg_vel_pid[i]) _ctx.pid.leg_vel_pid[i]->clear();
            continue;
        }

        // 使能但尚未校准：不输出（等校准脉冲）
        if (!_ctx.data.leg_calibrated[i])
        {
            _ctx.data.out_leg_torque[i]         = 0.0f;
            _ctx.data.out_leg_gravity_torque[i] = 0.0f;
            _ctx.data.leg_gravity_valid[i]      = false;
            continue;
        }

        // 目标角一阶插值平滑：每帧向摇杆命令靠近固定比例，使动作柔和、
        // 摇杆回中时缓慢收回（两种模式都更新，保证手动/VMC 切换无突变）
        _leg_target_smooth[i] +=
            (_ctx.cmd->leg.target_theta - _leg_target_smooth[i]) * LEG_TARGET_LERP;
        // 非对称限幅兜底：目标始终夹在完全放下与完全收起之间，防止任何越界
        _leg_target_smooth[i] =
            fp32_constrain(_leg_target_smooth[i], LEG_MANUAL_MIN, LEG_MANUAL_MAX);
        _ctx.data.target_leg_theta[i] = _leg_target_smooth[i];

        // W 对逻辑电机角的雅可比（两种模式都用）
        float jw[2] = {0.0f, 0.0f};
        if (_leg_kinematics)
        {
            _leg_kinematics->jacobian_w(_ctx.data.leg_alpha[i], jw);
        }
        const float k =
            static_cast<float>(LEG_LOGIC_SIGN[i] * LEG_MOTOR_DIR[i]) /
            LEG_EXTERNAL_RATIO;
        const float cp = cosf(pitch), sp = sinf(pitch);
        // 世界竖直向上的支撑力在车体系中为 (sp, cp)，因此力到逻辑电机
        // 力矩的映射必须同时使用 Jx、Jy；pitch=0 时退化为原来的 jwy
        const float j_vertical = k * (jw[0] * sp + jw[1] * cp);
        _ctx.data.leg_j_vertical[i] = j_vertical;

        // 一阶段整车重力补偿：以前轮轮心 O 为支点，对整车取矩。
        // 轮心 W 相对 O 的矢量 = 髋关节 B 相对 O + W 相对 B（四杆实时计算）。
        // 世界竖直后轮总支撑力 N 对 O 的力臂 = x_w*cp - y_w*sp；
        // 重力对 O 的力矩幅值 = Mg*(x_g*cp + y_g*sp)。
        const float x_w = LEG_O_HIP_X + _ctx.data.leg_wx[i] - LEG_B_X;
        const float y_w = LEG_O_HIP_Y + _ctx.data.leg_wy[i] - LEG_B_Y;
        const float support_arm = x_w * cp - y_w * sp;
        const float gravity_moment =
            LEG_VEH_MASS * LEG_GRAVITY *
            (LEG_O_COG_X * cp + LEG_O_COG_Y * sp);
        const bool support_valid = fabsf(support_arm) > 1e-4f;
        float n_i = 0.0f;
        if (LEG_VMC_GRAVITY_EN && support_valid)
        {
            // 左右腿均匀分担后轮总支撑力
            n_i = 0.5f * gravity_moment / support_arm;
        }
        _ctx.data.leg_support_arm[i] = support_arm;
        _ctx.data.leg_n_each[i] = n_i;
        _ctx.data.leg_gravity_valid[i] =
            _ctx.data.leg_pose_valid[i] && support_valid &&
            (fabsf(j_vertical) > 1e-6f);
        _ctx.data.out_leg_gravity_torque[i] = j_vertical * n_i;

        if (vmc_mode)
        {
            // 当前只测试重力前馈，不叠加俯仰扶正；最终限制在电机允许范围内
            _ctx.data.out_leg_torque[i] = fp32_constrain(
                _ctx.data.out_leg_gravity_torque[i],
                -LEG_MAX_TORQUE, LEG_MAX_TORQUE);
        }
        else
        {
            // 手动双环：位置环（目标为上方平滑后的 target_leg_theta）-> 速度环 -> 力矩。
            // 重力补偿只在 VMC 模式接入，手动模式不叠加
            if (_ctx.pid.leg_pos_pid[i])
            {
                _ctx.data.target_leg_radps[i] =
                    _ctx.pid.leg_pos_pid[i]->calculate(
                        _ctx.data.target_leg_theta[i],
                        _ctx.data.current_leg_theta[i]);
            }
            if (_ctx.pid.leg_vel_pid[i])
            {
                _ctx.data.out_leg_torque[i] =
                    _ctx.pid.leg_vel_pid[i]->calculate(
                        _ctx.data.target_leg_radps[i],
                        _ctx.data.current_leg_radps[i]);
            }
        }
    }
}

// =========================================================
// _send_motor_command()：发送所有电机指令
// =========================================================
void engineer_chassis_t::_send_motor_command() const
{
    // 右侧两轮发送时取反，把逻辑方向转回电调物理方向
    _ctx.motor.mecanum[0]->send_torque(_ctx.data.out_wheel_torque[0]);
    _ctx.motor.mecanum[1]->send_torque(-_ctx.data.out_wheel_torque[1]);
    _ctx.motor.mecanum[2]->send_torque(_ctx.data.out_wheel_torque[2]);
    _ctx.motor.mecanum[3]->send_torque(-_ctx.data.out_wheel_torque[3]);

    // 履带：右履带发送取反，把逻辑方向转回物理方向
    _ctx.motor.track[0]->send_torque(_ctx.data.out_track_torque[0]);
    _ctx.motor.track[1]->send_torque(-_ctx.data.out_track_torque[1]);

    // 腿：按逻辑方向取符号（右腿镜像取反），空指针保护
    for (int i = 0; i < 2; i++)
    {
        if (_ctx.motor.leg[i])
        {
            _ctx.motor.leg[i]->send_torque(
                LEG_LOGIC_SIGN[i] * _ctx.data.out_leg_torque[i]);
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
