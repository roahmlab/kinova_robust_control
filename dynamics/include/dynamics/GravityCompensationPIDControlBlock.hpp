#pragma once
#include "dynamics/PIDControlBlock.hpp"
#include "dynamics/Model.hpp"
#include "dynamics/RNEA.hpp"
#include <memory>
#include <system/BaseBlock.hpp>

#include "pinocchio/algorithm/rnea.hpp"

namespace KinovaRobustControl
{
namespace Dynamics
{
class GravityCompensationPIDControlBlock final: public PIDControlBlock
{
  public:
    using VecX = Eigen::VectorXd;
    using MatX = Eigen::MatrixXd;
    using SharedPtr = std::shared_ptr<GravityCompensationPIDControlBlock>;

    /**
     * @brief block maker
     **/
    static SharedPtr make_shared(
        const std::string &block_name,
        const std::shared_ptr<Model::model> &robot_model_input);

  protected:
    using Base = ::KinovaRobustControl::System::BaseBlock;

  public:
    /**
     * @brief implement robust control
     **/
    virtual VecX update(const InputPack &inputs) final override;

  protected:
    /**
     * @brief Ctor taking non-interval model directly
     * @param block_name: name of control block
     * @param rob_model: model of robot
     * @param variance: variance of the interval
     **/
    GravityCompensationPIDControlBlock(
        const std::string &block_name,
        const std::shared_ptr<Model::model> &robot_model_input);

    // dynamics pointer
    std::shared_ptr<MultiBodyDynamics> dynamics; 
};
} // namespace Dynamics
} // namespace KinovaRobustControl
