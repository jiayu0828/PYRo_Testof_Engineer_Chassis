#include "pyro_engineer_chassis.h"

namespace pyro
{

// 腿部失能态：右拨杆未处于 UP。
// 顶层 ACTIVE 仍可运行麦轮，但腿电机保持 0 力矩并持续清理腿部 PID/输出状态。
void engineer_chassis_t::leg_state_disabled_t::enter(owner *chassis)
{
    for (int i = 0; i < 2; ++i)
    {
        chassis->_leg_handle_disabled(i);
    }
}

void engineer_chassis_t::leg_state_disabled_t::execute(owner *chassis)
{
    for (int i = 0; i < 2; ++i)
    {
        chassis->_leg_handle_disabled(i);
    }
}

void engineer_chassis_t::leg_state_disabled_t::exit(owner *chassis)
{
    (void)chassis;
}

} // namespace pyro
