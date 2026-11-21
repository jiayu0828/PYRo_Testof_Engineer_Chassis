#include "pyro_engineer_chassis.h"

namespace pyro
{

// 已使能但未校准：不输出腿力矩，等待右拨杆进入 UP 的边沿触发软件置零。
void engineer_chassis_t::leg_state_uncalibrated_t::enter(owner *chassis)
{
    for (int i = 0; i < 2; ++i)
    {
        chassis->_leg_handle_uncalibrated(i);
    }
}

void engineer_chassis_t::leg_state_uncalibrated_t::execute(owner *chassis)
{
    for (int i = 0; i < 2; ++i)
    {
        chassis->_leg_handle_uncalibrated(i);
    }
}

void engineer_chassis_t::leg_state_uncalibrated_t::exit(owner *chassis)
{
    (void)chassis;
}

} // namespace pyro
