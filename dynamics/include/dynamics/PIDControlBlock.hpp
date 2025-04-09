#pragma once
#include "dynamics/ControllerBlock.hpp"

namespace KinovaRobustControl
{
namespace Dynamics
{
class PIDControlBlock: public ControllerBlock
{
public:
    using SharedPtr = std::shared_ptr<PIDControlBlock>;

private:
    using Base = ::KinovaRobustControl::System::BaseBlock;

public:
    using VecX = Eigen::VectorXd;

    /**
     * @brief block maker
     **/
    static SharedPtr make_shared(
        const std::string &block_name,
        const std::shared_ptr<Model::model> &robot_model_input);

    /**
     * @brief implement PID control
     **/
    virtual VecX update(const InputPack &inputs) override;

    /**
     * @brief set PID gains
     * @param Kp: proportional gain
     * @param Ki: integral gain
     * @param Kd: derivative gain
     **/
    void setParameters(
        const VecX& Kp_input,
        const VecX& Ki_input,
        const VecX& Kd_input
    );

    /**
     * @brief Ctor taking non-interval model directly
     * @param block_name: name of control block
     * @param rob_model: model of robot
     **/
    PIDControlBlock(
        const std::string &block_name,
        const std::shared_ptr<Model::model> &robot_model_input);

    bool is_coeff_set{false};
    
    VecX Kp;
    VecX Ki;
    VecX Kd;

    VecX accum_error; // accumulated error
};
} // namespace Dynamics
} // namespace KinovaRobustControl
