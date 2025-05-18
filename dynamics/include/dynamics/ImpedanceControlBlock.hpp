#pragma once
#include "dynamics/ControllerBlock.hpp"
#include "dynamics/Model.hpp"
#include "dynamics/RNEA.hpp"
#include <memory>
#include <system/BaseBlock.hpp>

namespace KinovaRobustControl
{
namespace Dynamics
{
class ImpedanceControlBlock : public ControllerBlock
{
  public:
    using VecX = Eigen::VectorXd;
    using MatX = Eigen::MatrixXd;
    using SharedPtr = std::shared_ptr<ImpedanceControlBlock>;

  protected:
    using Base = ::KinovaRobustControl::System::BaseBlock;

  public:
    /**
     * @brief block maker
     **/
    static SharedPtr make_shared(
      const std::string &block_name,
      const std::shared_ptr<Model::model> &robot_model_input);
      
    /**
     * @brief set coefficients
     * @param K_p: stiffness
     * @param K_d: damping
     **/
    void setParameters(
      const VecX &K_p, 
      const VecX &K_d);

    /**
     * @brief implement robust control
     **/
    virtual VecX update(const InputPack &inputs) final override;

  protected:
    /**
     * @brief Ctor taking non-interval model directly
     * @param block_name: name of control block
     * @param rob_model: model of robot
     **/
    ImpedanceControlBlock(
        const std::string &block_name,
        const std::shared_ptr<Model::model> &robot_model_input);

    bool is_coeff_set{false};

    // dynamics pointer
    std::shared_ptr<MultiBodyDynamics> dynamics; 
    
    VecX K_p; // stiffness
    VecX K_d; // damping
};
} // namespace Dynamics
} // namespace KinovaRobustControl
