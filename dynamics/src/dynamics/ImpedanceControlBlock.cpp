#include "dynamics/ImpedanceControlBlock.hpp"
#include "dynamics/Model.hpp"

namespace KinovaRobustControl
{
namespace Dynamics
{

ImpedanceControlBlock::SharedPtr ImpedanceControlBlock::make_shared(
    const std::string &block_name, 
    const std::shared_ptr<Model::model> &robot_model_input)
{
    return SharedPtr(
        new ImpedanceControlBlock(block_name, robot_model_input));
}

ImpedanceControlBlock::ImpedanceControlBlock(
    const std::string &block_name, 
    const std::shared_ptr<Model::model> &robot_model_input)
    : ControllerBlock(block_name, robot_model_input)
{
    dynamics = std::make_shared<MultiBodyDynamics>(robot_model);
    K_p = VecX::Zero(robot_model->NB);
    K_d = VecX::Zero(robot_model->NB);
    logger.info("Creating Impedance control block");
}

void ImpedanceControlBlock::setParameters(
    const VecX &K_p, 
    const VecX &K_d)
{
    this->K_p = K_p;
    this->K_d = K_d;

    if (K_p.size() != robot_model->NB)
    {
        const char *err_msg = "Size of K_p doesn't match with the model!";
        logger.critical(err_msg);
        throw std::invalid_argument(err_msg);
    }

    if (K_d.size() != robot_model->NB)
    {
        const char *err_msg = "Size of K_d doesn't match with the model!";
        logger.critical(err_msg);
        throw std::invalid_argument(err_msg);
    }

    is_coeff_set = true;
}

Eigen::VectorXd ImpedanceControlBlock::update(const InputPack &inputs)
{
    if (!is_coeff_set)
    {
        const char *err_msg = "Coefficient for passivity-based controller isn't set yet!";
        logger.critical(err_msg);
        throw std::invalid_argument(err_msg);
    }
    
    const VecX qdd_des = K_p.cwiseProduct(inputs.e) + K_d.cwiseProduct(inputs.ed);

    dynamics->rnea(inputs.pos, inputs.vel, inputs.vel, qdd_des);

    return dynamics->tau;
}

} // namespace Dynamics
} // namespace KinovaRobustControl
