#include "pyro_kin_leg.h"

#include <cmath>

namespace pyro
{

leg_kin_t::leg_kin_t(const float b[2], const float c[2], float l_be, float l_ed,
                     float l_dc, const float f_local[2], const float w_local[2],
                     int branch)
{
    _b[0] = b[0];
    _b[1] = b[1];
    _c[0] = c[0];
    _c[1] = c[1];
    _l_be = l_be;
    _l_ed = l_ed;
    _l_dc = l_dc;
    _f_local[0] = f_local[0];
    _f_local[1] = f_local[1];
    _w_local[0] = w_local[0];
    _w_local[1] = w_local[1];
    _branch = (branch >= 0) ? 1 : -1;
}

leg_kin_t::leg_pose_t leg_kin_t::forward(float alpha) const
{
    leg_pose_t p{};

    // Step 1: 定曲柄销 E
    const float ex = _b[0] + _l_be * cosf(alpha);
    const float ey = _b[1] + _l_be * sinf(alpha);

    // Step 2: 两圆心距离 d 与单位向量 u
    const float dx = _c[0] - ex;
    const float dy = _c[1] - ey;
    const float d  = sqrtf(dx * dx + dy * dy);

    // 无解保护：两圆不相交或圆心重合
    if (d < 1e-9f || d > (_l_ed + _l_dc) || d < fabsf(_l_ed - _l_dc))
    {
        p.valid = false;
        return p;
    }
    const float ux = dx / d;
    const float uy = dy / d;

    // Step 3: 两圆交点，选装配分支
    const float a  = (_l_ed * _l_ed - _l_dc * _l_dc + d * d) / (2.0f * d);
    const float h2 = _l_ed * _l_ed - a * a;
    if (h2 < 0.0f)
    {
        p.valid = false;
        return p;
    }
    const float h  = sqrtf(h2);
    const float qx = ex + a * ux;
    const float qy = ey + a * uy;
    const float vx = -uy;
    const float vy = ux;
    p.d_x = qx + _branch * h * vx;
    p.d_y = qy + _branch * h * vy;

    // Step 4: 小腿角
    p.phi = atan2f(_c[1] - p.d_y, _c[0] - p.d_x);

    // Step 5: 旋转局部坐标求 F、W
    const float cp = cosf(p.phi);
    const float sp = sinf(p.phi);
    p.f_x = _c[0] + cp * _f_local[0] - sp * _f_local[1];
    p.f_y = _c[1] + sp * _f_local[0] + cp * _f_local[1];
    p.w_x = _c[0] + cp * _w_local[0] - sp * _w_local[1];
    p.w_y = _c[1] + sp * _w_local[0] + cp * _w_local[1];

    p.valid = true;
    return p;
}

float leg_kin_t::jacobian_phi(float alpha) const
{
    constexpr float eps = 1e-6f;
    const auto pp = forward(alpha + eps);
    const auto pm = forward(alpha - eps);
    if (!pp.valid || !pm.valid)
    {
        return 0.0f;
    }
    return (pp.phi - pm.phi) / (2.0f * eps);
}

void leg_kin_t::jacobian_f(float alpha, float j[2]) const
{
    constexpr float eps = 1e-6f;
    const auto pp = forward(alpha + eps);
    const auto pm = forward(alpha - eps);
    if (!pp.valid || !pm.valid)
    {
        j[0] = 0.0f;
        j[1] = 0.0f;
        return;
    }
    j[0] = (pp.f_x - pm.f_x) / (2.0f * eps);
    j[1] = (pp.f_y - pm.f_y) / (2.0f * eps);
}

void leg_kin_t::jacobian_w(float alpha, float j[2]) const
{
    constexpr float eps = 1e-6f;
    const auto pp = forward(alpha + eps);
    const auto pm = forward(alpha - eps);
    if (!pp.valid || !pm.valid)
    {
        j[0] = 0.0f;
        j[1] = 0.0f;
        return;
    }
    j[0] = (pp.w_x - pm.w_x) / (2.0f * eps);
    j[1] = (pp.w_y - pm.w_y) / (2.0f * eps);
}

} // namespace pyro
