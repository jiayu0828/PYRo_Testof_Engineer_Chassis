#ifndef __PYRO_LEG_VMC_H__
#define __PYRO_LEG_VMC_H__

#include <cstdint>

// IMU 一阶低通系数：每帧保留 15% 新采样、85% 历史值
static constexpr float LEG_IMU_LPF_ALPHA = 0.15f;
// module_base 控制任务使用 vTaskDelayUntil 固定 1 ms 周期；插值按标称 dt 计算。
static constexpr float LEG_CONTROL_DT_S = 0.001f;

struct leg_vmc_result_t
{
    float theta{};       // 实际参与多项式计算的 clamp 后角度
    float j_dynamic{};   // 姿态投影后的动态雅可比
    float tau_gravity{}; // 单腿重力补偿力矩（小坦 theta 帧）
    bool  valid{false};  // 力平衡分母是否有效
};

struct leg_feedforward_output_t
{
    float theta_xt{};
    float j_dynamic{};
    float gravity_tau{};
    float attitude_tau{};
    bool  gravity_valid{false};
};

leg_vmc_result_t calc_leg_xiaotan_vmc(float theta_raw, float pitch);

#endif