#pragma once
#include "filter/Filters.hpp"
#include "customized_msgs/Messages.hpp"
#include "system/BaseBlock.hpp"
#include <system/Port/OutputPort.hpp>

namespace KinovaRobustControl
{
namespace Kortex
{
using System::BaseBlock;
/**
 * @brief filter target joint
 **/
class JointFilterBlock final : public BaseBlock
{
  public:
    static inline BaseBlock::SharedPtr make_shared(const std::string &name,
                                                   double a0, double a1,
                                                   double b0, double b1,
                                                   double b2)
    {
        return BaseBlock::SharedPtr(
            new JointFilterBlock(name, a0, a1, b0, b1, b2));
    }
    virtual void run() final;

  private:
    JointFilterBlock(const std::string &name, double a0, double a1, double b0,
                     double b1, double b2)
        : BaseBlock(name, System::OutputPort<msgs::Measurement>::make_shared()),
          input(register_input<msgs::Measurement>("input")),
          vel_filter(b0, b1, b2, a0, a1)
    {
    }

    System::AbstractInputPort::SharedPtr input;
    Filter::BiquadFilter<Eigen::VectorXd> vel_filter;
};
} // namespace Kortex
} // namespace KinovaRobustControl
