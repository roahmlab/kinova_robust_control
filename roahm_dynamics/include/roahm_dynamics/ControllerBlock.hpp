#pragma once
#include "roahm_utils/Utils.hpp"
#include "roahm_dynamics/RNEA.hpp"
#include "roahm_trajectories/TrajectoryManager.hpp"
#include "roahm_msgs/Messages.hpp"
#include "roahm_system/BaseBlock.hpp"
#include <eigen3/Eigen/Dense>
#include <deque>

namespace Roahm
{
namespace Dynamics
{

// constexpr double TORQUE_LIMITS[] = {56.7, 56.7, 56.7, 56.7, 29.4, 29.4, 29.4};
constexpr double TORQUE_LIMITS[] = {85.0, 85.0, 85.0, 85.0, 40.0, 40.0, 40.0};

class ControllerBlock : public ::Roahm::System::BaseBlock
{
  protected:
    using ControlMsg = ::Roahm::msgs::ControlMsg;
    using Measurement = ::Roahm::msgs::Measurement;
    struct InputPack;

  public:
    using VecX = Eigen::VectorXd;
    using Vec10 = Eigen::Matrix<double, 10, 1>;
    using MatX = Eigen::MatrixXd;

    /**
     * @brief calculation structure of controller
     **/
    virtual void run() final override;

    /**
     * @brief specific function to update control torque
     * @param inputs: %InputPack to use
     **/
    virtual VecX update(const InputPack &inputs) = 0;

    /**
     * @brief reset the controller, for example, set error integral to 0
     **/
    virtual void reset() {};

  protected:
    /**
     * @brief constructor directly given robot model
     **/
    ControllerBlock(const std::string &block_name,
                    const std::shared_ptr<Model::model> &robot_model_input);

    /**
     * @brief default destructor
     * @details close the torque output file
     */
    ~ControllerBlock();

    /**
     * @brief package for easy passing of all the value needed
     **/
    struct InputPack
    {
        // desired state
        const VecX& pos_des;
        const VecX& vel_des;
        const VecX& acc_des;

        // actual state
        const VecX& pos;
        const VecX& vel;

        // pos and vel error
        const VecX& e;
        const VecX& ed;

        // time difference between previous update and current update
        double delta_t;
    };

  protected:
    /// trajectory message input
    System::AbstractInputPort::SharedPtr traj_port;
    /// state from robot
    System::AbstractInputPort::SharedPtr state_port;

    /// robot model
    std::shared_ptr<Model::model> robot_model; 

    /// time when controller starts working
    decltype(std::chrono::system_clock::now()) t0;

    /// time of the previous message
    decltype(std::chrono::system_clock::now()) t_prev;

    // trajectory manager
    std::shared_ptr<TrajectoryManager> trajectories;

    // logger for torque messages
    std::ofstream data_file;
    std::ofstream tracking_error_file;

    std::ifstream endeffector_inertial_file;

    size_t previous_trajectory_id = 0;
    bool is_tracking_trajectory = false;

    // current joint position
    VecX q_current;
    bool is_gripper_open = false;
};

} // namespace Dynamics
} // namespace Roahm