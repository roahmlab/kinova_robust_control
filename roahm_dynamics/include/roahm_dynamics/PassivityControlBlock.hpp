#pragma once
#include "roahm_dynamics/ControllerBlock.hpp"
#include "roahm_dynamics/Model.hpp"
#include "roahm_dynamics/RNEA.hpp"
#include <memory>
#include <roahm_system/BaseBlock.hpp>

namespace Roahm
{
namespace Dynamics
{
class PassivityControlBlock : public ControllerBlock
{
  public:
    using VecX = Eigen::VectorXd;
    using MatX = Eigen::MatrixXd;
    using SharedPtr = std::shared_ptr<PassivityControlBlock>;

  protected:
    using Base = ::Roahm::System::BaseBlock;

  public:
    /**
     * @brief implement robust control
     **/
    virtual VecX update(const InputPack &inputs) final override;

    /**
     * @brief stabilize input
     **/
    virtual VecX stabilize_input(const InputPack &inputs) = 0;

  protected:
    /**
     * @brief Ctor taking non-interval model directly
     * @param block_name: name of control block
     * @param rob_model: model of robot
     * @param variance: variance of the interval
     **/
    PassivityControlBlock(
        const std::string &block_name,
        const std::shared_ptr<Model::model> &robot_model_input);

    bool is_coeff_set{false};

    // dynamics pointer
    std::shared_ptr<MultiBodyDynamics> dynamics; 
    
    VecX K_r;
};
} // namespace Dynamics
} // namespace Roahm
