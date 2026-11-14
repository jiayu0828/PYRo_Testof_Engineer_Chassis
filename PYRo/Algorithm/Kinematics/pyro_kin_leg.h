#ifndef __PYRO_KIN_LEG_H__
#define __PYRO_KIN_LEG_H__

#include <cstdint>

namespace pyro
{

/**
 * @brief Rear-leg four-bar forward kinematics
 * 后腿四杆机构正运动学（精确两圆交点解 + 数值雅可比）
 * 几何参数全部来自 SolidWorks 实测，单位 m。
 */
class leg_kin_t
{
  public:
    /// 正运动学输出
    struct leg_pose_t
    {
        float d_x, d_y;   // 摇杆铰 D 世界坐标
        float phi;        // 小腿角（C->F 相对世界 +x）
        float f_x, f_y;   // 末端 F 世界坐标
        float w_x, w_y;   // 轮心 W 世界坐标
        bool  valid;      // 两圆是否有解
    };

    /**
     * @brief 构造：传入全部实测几何
     * @param b 固定铰 B 坐标
     * @param c 固定铰 C 坐标
     * @param l_be 曲柄长
     * @param l_ed 连杆长
     * @param l_dc 摇杆长
     * @param f_local 小腿局部系 F（C 原点）
     * @param w_local 小腿局部系 W（C 原点）
     * @param branch 装配分支（+1 / -1）
     */
    leg_kin_t(const float b[2], const float c[2], float l_be, float l_ed,
              float l_dc, const float f_local[2], const float w_local[2],
              int branch);

    /// 正运动学：曲柄角 alpha（rad）-> 位姿
    [[nodiscard]] leg_pose_t forward(float alpha) const;

    /// 数值雅可比：小腿角对曲柄角的瞬时传动比 dphi/dalpha
    [[nodiscard]] float jacobian_phi(float alpha) const;

    /// 末端 F 的数值雅可比（dF/dalpha），分量写入 j[2]
    void jacobian_f(float alpha, float j[2]) const;

    /// 轮心 W 的数值雅可比（dW/dalpha），分量写入 j[2]
    void jacobian_w(float alpha, float j[2]) const;

  private:
    float _b[2];
    float _c[2];
    float _l_be;
    float _l_ed;
    float _l_dc;
    float _f_local[2];
    float _w_local[2];
    int   _branch;
};

} // namespace pyro

#endif
