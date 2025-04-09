#include "kortex/JointFilterBlock.hpp"
#include <customized_msgs/Messages.hpp>

namespace KinovaRobustControl
{
namespace Kortex
{

void JointFilterBlock::run()
{
    while (true)
    {
        auto ptr = input->get_new<msgs::Measurement>();
        msgs::Measurement joint_info = *ptr;
        joint_info.vel = vel_filter.filter(joint_info.vel);
        default_output->set(joint_info);
    }
}
} // namespace Kortex
} // namespace KinovaRobustControl
