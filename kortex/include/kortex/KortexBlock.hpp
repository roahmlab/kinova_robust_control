#pragma once
#include "kortex/KortexRobot.hpp"
#include "system/BaseBlock.hpp"
#include <bitset>
#include <rclcpp/rclcpp.hpp>

namespace KinovaRobustControl
{
namespace Kortex
{
/**
 * @brief wrapper block for operating Kortex Robot
 * @details outputport: %Measurement
 *          inputports: %ControlMsg control_msg
 **/
class KortexBlock final : public System::BaseBlock
{
  public:
    /**
     * @brief factory function
     **/
    static std::shared_ptr<KortexBlock> make_shared(
        const std::string &block_name, const KortexSettings &settings);

    virtual void run() override;

    inline bool MovePredefined(const std::string &name)
    {
        return rob.MovePredefined(name);
    }

    inline bool MoveJointPos(const std::vector<float> &des)
    {
        return rob.MoveJointPos(des);
    }

    inline bool MoveCartesian(const std::vector<float> &des)
    {
      return rob.MoveCartesian(des);
    }

    inline bool CloseGripper()
    {
      return rob.CloseGripper();
    }

    inline bool ResetGripper()
    {
      return rob.ResetGripper();
    }

  private:
    /**
     * @brief default Ctor, private
     * @param block_name: name of the block
     * @param output: output port
     * @param settings: settings for kortex robot
     **/
    KortexBlock(const std::string &block_name, const KortexSettings &settings);

    KortexRobot rob;
    System::AbstractInputPort::SharedPtr control_msg;
};

} // namespace Kortex
} // namespace KinovaRobustControl
