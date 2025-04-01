#include "roahm_dynamics/ARMOURControlBlock.hpp"

namespace Roahm
{
namespace Dynamics
{

ARMOURControlBlock::SharedPtr ARMOURControlBlock::make_shared(
    const std::string &block_name, 
    const std::shared_ptr<Model::model> &robot_model_input)
{
    return SharedPtr(
        new ARMOURControlBlock(block_name, robot_model_input));
}

ARMOURControlBlock::ARMOURControlBlock(
    const std::string &block_name, 
    const std::shared_ptr<Model::model> &robot_model_input)
    : PassivityControlBlock(block_name, robot_model_input)
{
    logger.info("Creating ARMOUR control block");
}

Eigen::VectorXd ARMOURControlBlock::stabilize_input(const InputPack &inputs) 
{
    if (!is_coeff_set)
    {
        const char *err_msg = "Coefficient for ARMOUR controller isn't set yet!";
        logger.critical(err_msg);
        throw std::invalid_argument(err_msg);
    }

    const VecX qd_aux = inputs.vel_des + K_r.cwiseProduct(inputs.e);
    const VecX qdd_aux = inputs.acc_des + K_r.cwiseProduct(inputs.ed);
    const VecX r = inputs.ed + K_r.cwiseProduct(inputs.e);

    // Step 1, calculate interval torque from passivity RNEA
    dynamics->rnea_interval(inputs.pos, inputs.vel, qd_aux, qdd_aux);

    // sanity check for size since constrained system might be involved
    if (dynamics->tau.size() != dynamics->tau_sup.size() || 
        dynamics->tau.size() != dynamics->tau_inf.size()) {
        throw std::runtime_error("controller.cpp: update(): Size mismatch in nominal torque and interval torque!");
    }

    // Interval check
    for (Eigen::Index i = 0; i < dynamics->tau.size(); i++) {
        if (dynamics->tau(i) > dynamics->tau_sup(i) || dynamics->tau(i) < dynamics->tau_inf(i)) {
            printf("tau: %f\n", dynamics->tau(i));
            printf("[tau_inf, tau_sup]: [%f %f]\n", dynamics->tau_inf(i), dynamics->tau_sup(i));
            throw std::runtime_error("controller.cpp: update(): Nominal torque output falls outside interval output!");
        }
    }

    // Step 2, calculate error bound introduced by model uncertainty
    VecX bound(dynamics->tau.size());
    for (int i = 0; i < dynamics->tau.size(); i++) {
        bound(i) = std::max(
            dynamics->tau_sup(i) - dynamics->tau(i), 
            dynamics->tau(i)     - dynamics->tau_inf(i));
    }

    // Step 3, calculate upper bound of Lyapunov function
    // Only compute robust input when r is large enough to avoid numerical issues
    const double r_norm = r.norm();
    double V_sup = 0;
    if (r_norm > r_norm_threshold) {
        // Step 5a, calculate V = 0.5 * r' * M * r
        VecX q_d_zero = VecX::Zero(robot_model->NB);
        dynamics->rnea_interval(inputs.pos, q_d_zero, q_d_zero, r, false);

        // dot product between r and M * r
        for (int i = 0; i < dynamics->tau.size(); i++) {
            if (r(i) > 0) {
                V_sup += 0.5 * r(i) * dynamics->tau_sup(i);
            }
            else {
                V_sup += 0.5 * r(i) * dynamics->tau_inf(i);
            }
        }
    }
    // else {
    //     v is just zero vector then
    // }

    // Step 4, calculate robust input
    VecX v = VecX::Zero(dynamics->tau.size());

    // Only compute robust input when r is large enough to avoid numerical issues
    if (r_norm > r_norm_threshold) {
        // Step 5b, compute robust input
        double h = V_max - V_sup;
        double lambda = std::max(0.0, -alpha * h / r_norm + bound.norm());
        v = lambda * r / r_norm;
    }

    return v;
}

void ARMOURControlBlock::setParameters(
    const VecX &K_r, 
    const double alpha,
    const double V_max, 
    const double r_norm_threshold)
{
    this->alpha = alpha;
    this->V_max = V_max;
    this->r_norm_threshold = r_norm_threshold;
    this->K_r = K_r;

    if (K_r.size() != robot_model->NB)
    {
        const char *err_msg = "Size of K_r doesn't match with the model!";
        logger.critical(err_msg);
        throw std::invalid_argument(err_msg);
    }

    is_coeff_set = true;
}

} // namespace Dynamics
} // namespace Roahm
