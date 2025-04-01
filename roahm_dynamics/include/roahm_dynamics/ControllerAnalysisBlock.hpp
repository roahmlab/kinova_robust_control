#pragma once
#include "roahm_msgs/Messages.hpp"
#include "roahm_msgs/msg/debug.hpp"
#include "roahm_trajectories/TrajectoryManager.hpp"
#include "roahm_system/BaseBlock.hpp"
#include <deque>
#include <rclcpp/rclcpp.hpp>
#include <roahm_system/Port/InputPort.hpp>

namespace Roahm
{
namespace Dynamics
{
class ControllerAnalysisBlock final : public System::BaseBlock
{
  public:
    using VecX = Eigen::VectorXd;
    using SharedPtr = std::shared_ptr<ControllerAnalysisBlock>;

  private:
    using ControlMsg = ::Roahm::msgs::ControlMsg;
    using Measurement = ::Roahm::msgs::Measurement;
    using Trajectory = ::Roahm::Trajectory;
    using Debug = ::roahm_msgs::msg::Debug;

  public:
    static inline SharedPtr make_shared(const std::string &block_name,
                                        const std::string &topic_name,
                                        rclcpp::Node::SharedPtr node)
    {
        return SharedPtr(
            new ControllerAnalysisBlock(block_name, topic_name, node));
    }

    void run() override;

  private:
    /**
     * @brief Ctor to be called by derived class
     * @param block_name: name of the block
     * @param pub_topic: topic name of the publisher
     * @param node: ros node
     **/
    ControllerAnalysisBlock(const std::string &block_name,
                            const std::string &pub_topic,
                            rclcpp::Node::SharedPtr node);

  private:
    // trajectory publisher
    System::AbstractInputPort::SharedPtr traj_port;

    // joint state
    System::AbstractInputPort::SharedPtr state_port;

    // control output
    System::AbstractInputPort::SharedPtr control_port;

    /// time when controller starts working
    decltype(std::chrono::system_clock::now()) t0;

    // ros
    rclcpp::Node::SharedPtr node;
    typename rclcpp::Publisher<Debug>::SharedPtr pub;

    // trajectory manager
    std::shared_ptr<TrajectoryManager> trajectories;
};
} // namespace Dynamics
} // namespace Roahm
