#include "roahm_dynamics/GravityCompensationPIDControlBlock.hpp"
#include "roahm_dynamics/Model.hpp"

namespace Roahm
{
namespace Dynamics
{

GravityCompensationPIDControlBlock::SharedPtr GravityCompensationPIDControlBlock::make_shared(
    const std::string &block_name, 
    const std::shared_ptr<Model::model> &robot_model_input)
{
    return SharedPtr(
        new GravityCompensationPIDControlBlock(block_name, robot_model_input));
}

GravityCompensationPIDControlBlock::GravityCompensationPIDControlBlock(
    const std::string &block_name, 
    const std::shared_ptr<Model::model> &robot_model_input)
    : PIDControlBlock(block_name, robot_model_input)
{
    dynamics = std::make_shared<MultiBodyDynamics>(robot_model);
}

Eigen::VectorXd GravityCompensationPIDControlBlock::update(const InputPack &inputs)
{
    if (!is_coeff_set)
    {
        const char *err_msg = "Coefficient for gravity-compensation PID controller isn't set yet!";
        logger.critical(err_msg);
        throw std::invalid_argument(err_msg);
    }

    // Step 1, calculate gravity compensation
    const pinocchio::Model& model_pinocchio = robot_model->model_pinocchio;
    pinocchio::Data& data_pinocchio = robot_model->data_pinocchio;
    pinocchio::computeGeneralizedGravity(model_pinocchio, data_pinocchio, inputs.pos);
    const VecX grav_comp = data_pinocchio.g;

    // Step 2, calculate additional inputs to stabilize the system
    accum_error += inputs.e * inputs.delta_t;

    const VecX stab_input =
        Kp.cwiseProduct(inputs.e) +
        Ki.cwiseProduct(accum_error) +
        Kd.cwiseProduct(inputs.ed);

    return grav_comp + stab_input;
}

} // namespace Dynamics
} // namespace Roahm
