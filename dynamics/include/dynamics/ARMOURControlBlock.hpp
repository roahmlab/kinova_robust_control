#pragma once
#include "dynamics/PassivityControlBlock.hpp"

namespace KinovaRobustControl
{
namespace Dynamics
{
class ARMOURControlBlock final : public PassivityControlBlock
{
  public:
    using VecX = Eigen::VectorXd;
    using MatX = Eigen::MatrixXd;
    using SharedPtr = std::shared_ptr<ARMOURControlBlock>;

  private:
    using Base = ::KinovaRobustControl::System::BaseBlock;

  public:
    /**
     * @brief block maker
     **/
    static SharedPtr make_shared(
        const std::string &block_name,
        const std::shared_ptr<Model::model> &robot_model_input);

    /**
     * @brief implement robust control
     **/
    virtual VecX stabilize_input(const InputPack &inputs) final override;

    /**
     * @brief set coefficients
     * @param K_r
     * @param alpha
     * @param V_max
     * @param r_norm_threshold
     **/
    void setParameters(
        const VecX &K_r, 
        const double alpha, 
        const double V_max,
        const double r_norm_threshold);

  private:
    /**
     * @brief Ctor taking non-interval model directly
     * @param block_name: name of control block
     * @param rob_model: model of robot
     * @param variance: variance of the interval
     **/
    ARMOURControlBlock(
        const std::string &block_name,
        const std::shared_ptr<Model::model> &robot_model_input);
    
    // ARMOUR parameters
    double alpha = 0;
    double V_max = 0;
    double r_norm_threshold = 1e-10;
};
} // namespace Dynamics
} // namespace KinovaRobustControl
