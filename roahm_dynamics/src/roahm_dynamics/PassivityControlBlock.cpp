#include "roahm_dynamics/PassivityControlBlock.hpp"
#include "roahm_dynamics/Model.hpp"

namespace Roahm
{
namespace Dynamics
{

PassivityControlBlock::PassivityControlBlock(
    const std::string &block_name, 
    const std::shared_ptr<Model::model> &robot_model_input)
    : ControllerBlock(block_name, robot_model_input)
{
    dynamics = std::make_shared<MultiBodyDynamics>(robot_model);
    K_r = VecX::Zero(robot_model->NB);
}

Eigen::VectorXd PassivityControlBlock::update(const InputPack &inputs)
{
    if (!is_coeff_set)
    {
        const char *err_msg = "Coefficient for passivity-based controller isn't set yet!";
        logger.critical(err_msg);
        throw std::invalid_argument(err_msg);
    }
    
    // Step 1, calculate reference terms
    const VecX qd_aux = inputs.vel_des + K_r.cwiseProduct(inputs.e);
    const VecX qdd_aux = inputs.acc_des + K_r.cwiseProduct(inputs.ed);

    // Step 2, calculate nominal torque from passivity RNEA
    dynamics->rnea(inputs.pos, inputs.vel, qd_aux, qdd_aux);
    const VecX u_nominal = dynamics->tau;

    // Step 3, calculate additional inputs to stabilize the system
    const VecX v = stabilize_input(inputs);

    if (v.size() != robot_model->NB)
    {
        const char *err_msg = "Size of v doesn't match with the model!";
        logger.critical(err_msg);
        throw std::runtime_error(err_msg);
    }

    return u_nominal + v;
}

} // namespace Dynamics
} // namespace Roahm
