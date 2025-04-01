#pragma once
#include "roahm_dynamics/Model.hpp"
#include "roahm_utils/Utils.hpp"

namespace Roahm
{
namespace Dynamics
{

constexpr double FRICTION_APPLY_VELOCITY_THRESHOLD = 1e-4; //!< Threshold on joint velocity for applying friction.

/**
 * @brief Class representing multi-body dynamics.
 * 
 * This class represents the dynamics of a multi-body system, including
 * methods for computing the dynamics and updating relevant variables.
 */
class MultiBodyDynamics {
public:
    using Vec3 = Eigen::Vector3d; //!< Type alias for Eigen 3D vector.
    using Mat3 = Eigen::Matrix3d; //!< Type alias for Eigen 3x3 matrix.
    using Vec6 = Eigen::Matrix<double, 6, 1>; //!< Type alias for Eigen 6D vector.
    using Mat6 = Eigen::Matrix<double, 6, 6>; //!< Type alias for Eigen 6x6 matrix.
    using VecX = Eigen::VectorXd; //!< Type alias for Eigen dynamic vector.
    using MatX = Eigen::MatrixXd; //!< Type alias for Eigen dynamic matrix.

    std::shared_ptr<Model::model> modelPtr_ = nullptr; //!< Pointer to the robot model.

    int NB = 0; //!< Number of joints.
    Mat6 Xj; //!< Spatial rotation matrix.
    std::vector<Vec6> S; //!< Unit screw vectors indicating joint types.
    std::vector<Mat6> Xup; //!< Spatial transform matrices.
    Vec6 vJ; //!< Joint velocity vector.
    Vec6 vJ_aux; //!< Auxiliary joint velocity vector.
    std::vector<Vec6> v; //!< Velocity vectors.
    std::vector<Vec6> v_aux; //!< Auxiliary velocity vectors.
    std::vector<Vec6> a; //!< Acceleration vectors.
    std::vector<Eigen::Matrix<double, 6, 10>> K; //!< Assembly matrices.
    MatX Yfull; //!< Full dynamics regressor matrix.
    MatX Y; //!< Dynamics regressor matrix.
    VecX tau; //!< Joint torque vector.
    VecX tau_inf; //!< Lower bound of joint torques.
    VecX tau_sup; //!< Upper bound of joint torques.

    VecX q_copy; //!< Copy of joint position vector that has been evaluated.
    VecX q_d_copy; //!< Copy of joint velocity vector that has been evaluated.
    VecX q_aux_d_copy; //!< Copy of auxiliary joint velocity vector that has been evaluated.
    VecX q_dd_copy; //!< Copy of joint acceleration vector that has been evaluated.

    /**
     * @brief Default constructor.
     */
    MultiBodyDynamics();

    /**
     * @brief Constructor with model pointer.
     * 
     * @param modelPtr_in Pointer to the robot model.
     */
    MultiBodyDynamics(const std::shared_ptr<Model::model>& modelPtr_in);

    /**
     * @brief Constructor with model pointer and tolerance parameter.
     * 
     * @param modelPtr_in Pointer to the robot model.
     * @param phi_eps_in Model uncertainty for each inertial parameter.
     */
    MultiBodyDynamics(const std::shared_ptr<Model::model>& modelPtr_in, 
                      const double phi_eps_in);

    /**
     * @brief Destructor.
     */
    ~MultiBodyDynamics() = default;

    /**
     * @brief Return the actual system dimension.
     * 
     * For system without constraints, the actual system dimension is the number of joints.
     * For systems with constraints, the actual system dimension is the number of actuated joints.
     */
    virtual int actual_system_dimension() const {
        return NB;
    }

    /**
     * @brief Update dynamics matrices.
     * 
     * This function updates the dynamics matrices Y and Yfull for passive dynamics.
     * 
     * @param q Joint position vector.
     * @param q_d Joint velocity vector.
     * @param q_aux_d Auxiliary joint velocity vector.
     * @param q_dd Joint acceleration vector.
     * @param add_gravity Flag indicating whether to add gravity effects (default: true).
     */
    void Yphi_passive(const VecX& q, 
                      const VecX& q_d, 
                      const VecX& q_aux_d,
                      const VecX& q_dd, 
                      const bool add_gravity = true);

    /**
     * @brief Compute RNEA and update dynamics matrices.
     * 
     * This function computes the Recursive Newton-Euler Algorithm (RNEA) and updates
     * the dynamics matrices Y, Yfull, and joint torques tau.
     * 
     * @param q Joint position vector.
     * @param q_d Joint velocity vector.
     * @param q_aux_d Auxiliary joint velocity vector.
     * @param q_dd Joint acceleration vector.
     * @param add_gravity Flag indicating whether to add gravity effects (default: true).
     */
    virtual void rnea(const VecX& q, 
                      const VecX& q_d, 
                      const VecX& q_aux_d,
                      const VecX& q_dd,
                      const bool add_gravity = true);

    /**
     * @brief Compute RNEA with interval arithmetic and update dynamics matrices.
     * 
     * This function computes the Recursive Newton-Euler Algorithm (RNEA) using
     * interval arithmetic and updates the dynamics matrices Y, Yfull, joint torques
     * tau, and their respective lower and upper bounds.
     * 
     * @param q Joint position vector.
     * @param q_d Joint velocity vector.
     * @param q_aux_d Auxiliary joint velocity vector.
     * @param q_dd Joint acceleration vector.
     * @param add_gravity Flag indicating whether to add gravity effects (default: true).
     */
    virtual void rnea_interval(const VecX& q, 
                               const VecX& q_d, 
                               const VecX& q_aux_d,
                               const VecX& q_dd,
                               const bool add_gravity = true);
};

} // namespace Dynamics
} // namespace Roahm
