#include "roahm_kortex/KortexBlock.hpp"
#include "roahm_kortex/KortexRobot.hpp"
#include <stdexcept>

namespace Roahm
{
namespace Kortex
{
std::shared_ptr<KortexBlock> KortexBlock::make_shared(
    const std::string &block_name, const KortexSettings &settings)
{
    return std::shared_ptr<KortexBlock>(new KortexBlock(block_name, settings));
}

void KortexBlock::run()
{
    // check connection
    if (!this->control_msg->is_connected())
        throw std::runtime_error(
            "Need to connect to control message before running KortexBlock");

    // NOTE: wait for control initialization
    this->control_msg->get_new<ControlMsg>();

    // get control function
    auto get_control = [this]() -> std::shared_ptr<const ControlMsg> {
        return this->control_msg->get_nowait<ControlMsg>();
    };

    auto send_measurement = [this](const Measurement &measure) {
        this->default_output->set(measure);
    };

    rob.lowlevel_control(false, send_measurement, get_control);
}

KortexBlock::KortexBlock(const std::string &block_name,
                         const KortexSettings &settings)
    : System::BaseBlock(block_name,
                        System::OutputPort<Measurement>::make_shared()),
      rob(settings), control_msg(register_input<ControlMsg>("control_msg"))
{
    // init robot
    rob.Init();
}
} // namespace Kortex
} // namespace Roahm
