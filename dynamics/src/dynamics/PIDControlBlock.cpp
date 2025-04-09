#include "dynamics/PIDControlBlock.hpp"

namespace KinovaRobustControl
{
namespace Dynamics
{

PIDControlBlock::SharedPtr PIDControlBlock::make_shared(
    const std::string &block_name, 
    const std::shared_ptr<Model::model> &robot_model_input)
{
    return SharedPtr(
        new PIDControlBlock(block_name, robot_model_input));
}

PIDControlBlock::PIDControlBlock(
    const std::string &block_name, 
    const std::shared_ptr<Model::model> &robot_model_input)
    : ControllerBlock(block_name, robot_model_input)
{
    Kp = VecX::Zero(robot_model->NB);
    Ki = VecX::Zero(robot_model->NB);
    Kd = VecX::Zero(robot_model->NB);
    accum_error = VecX::Zero(robot_model->NB);
}

Eigen::VectorXd PIDControlBlock::update(const InputPack &inputs)
{
    if (!is_coeff_set)
    {
        const char *err_msg = "PID gains are not set yet!";
        logger.critical(err_msg);
        throw std::invalid_argument(err_msg);
    }

    accum_error += inputs.e * inputs.delta_t;

    const VecX torque = 
        Kp.cwiseProduct(inputs.e) +
        Ki.cwiseProduct(accum_error) +
        Kd.cwiseProduct(inputs.ed);

    return torque;
}

void PIDControlBlock::setParameters(
    const VecX& Kp_input,
    const VecX& Ki_input,
    const VecX& Kd_input
)
{
    if (Kp_input.size() != robot_model->NB ||
        Ki_input.size() != robot_model->NB ||
        Kd_input.size() != robot_model->NB)
    {
        const char *err_msg = "Size of PID gains doesn't match with the model!";
        logger.critical(err_msg);
        throw std::invalid_argument(err_msg);
    }

    this->Kp = Kp_input;
    this->Ki = Ki_input;
    this->Kd = Kd_input;

    is_coeff_set = true;
}

} // namespace Dynamics
} // namespace KinovaRobustControl
