#pragma once
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/multibody/data.hpp>

// #include <yaml-cpp/yaml.h>

#include "dynamics/Spatial.hpp"
#include "utils/Utils.hpp"
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>

namespace KinovaRobustControl
{
namespace Model
{

/**
 * @brief Convert Pinocchio joint type string to integer.
 * 
 * This function converts a string representing a Pinocchio joint type to an integer.
 * 
 * @param jtype The string representing the Pinocchio joint type.
 * @return int The integer corresponding to the joint type.
 *         1 = revolute X axis 'Rx'
 *         2 = revolute Y axis 'Ry'
 *         3 = revolute Z axis 'Rz'
 *         -1 = reversed revolute X axis '-Rx'
 *         -2 = reversed revolute Y axis '-Ry'
 *         -3 = reversed revolute Z axis '-Rz'
 *         4 = prismatic X axis 'Px' 
 *         5 = prismatic Y axis 'Py'
 *         6 = prismatic Z axis 'Pz'
 *         -4 = reversed prismatic X axis '-Px'
 *         -5 = reversed prismatic Y axis '-Py'
 *         -6 = reversed prismatic Z axis '-Pz'
 *         0 = fixed joint (actually can not identified from a pinocchio model since it's already merged)
 */
int convertPinocchioJointType(const std::string& jtype);
 
/**
 * @brief Class representing a robot model.
 * 
 * This class represents a robot model, containing various attributes and methods
 * for describing the robot's structure and properties.
 */
class model {
public:
    using Vec3 = Eigen::Vector3d; //!< Type alias for Eigen 3D vector.
    using Mat3 = Eigen::Matrix3d; //!< Type alias for Eigen 3x3 matrix.
    using Vec6 = Eigen::Matrix<double, 6, 1>; //!< Type alias for Eigen 6D vector.
    using Mat6 = Eigen::Matrix<double, 6, 6>; //!< Type alias for Eigen 6x6 matrix.
    using Vec10 = Eigen::Matrix<double, 10, 1>;
    using VecX = Eigen::VectorXd; //!< Type alias for Eigen dynamic vector.

    int NB = 0; //!< Number of joints.
    Vec6 a_grav; //!< Gravity acceleration vector.
    std::vector<int> jtype; //!< Vector of joint types.
    std::vector<int> parent; //!< Vector of parent joint indices.
    std::vector<Mat6> Xtree; //!< Vector of spatial transform matrices.
    VecX phi; //!< Vector of inertial parameters.
    VecX phi_eps; //!< Vector of uncertainty of inertial parameters.
    VecX phi_lb; //!< Vector of lower bound of inertial parameters.
    VecX phi_ub; //!< Vector of upper bound of inertial parameters.

    // motor dynamics parameters
    VecX friction; //!< Vector of friction parameters.
    VecX friction_eps; //!< Vector of uncertainty of friction parameters.
    VecX damping; //!< Vector of damping parameters.
    VecX damping_eps; //!< Vector of uncertainty of damping parameters.
    VecX transmissionInertia; //!< Vector of transmission inertia parameters.
    VecX transmissionInertia_eps; //!< Vector of uncertainty of transmission inertia parameters.    
    VecX offset; //!< Vector of offset parameters.

    pinocchio::Model model_pinocchio; //!< Pinocchio model.
    pinocchio::Data data_pinocchio; //!< Pinocchio data.

    /**
     * @brief Default constructor.
     */
    model() = default;

    /**
     * @brief Constructor to load model from URDF file.
     * 
     * This constructor initializes the robot model by loading it from a URDF file.
     * 
     * @param urdf_filename The filename of the URDF file.
     * @param phi_eps_input The model uncertainty for each inertial parameter.
     */
    model(const std::string& urdf_filename,
          const double phi_eps_input = 0.0);

    /**
     * @brief Constructor to load model from URDF file with additional parameters.
     * 
     * This constructor initializes the robot model by loading it from a URDF file,
     * with additional parameters for friction, damping, and transmission inertia.
     * 
     * @param urdf_filename The filename of the URDF file.
     * @param friction_input Vector of friction parameters.
     * @param friction_eps_input Vector of friction uncertainty parameters.
     * @param damping_input Vector of damping parameters.
     * @param damping_eps_input Vector of damping uncertainty parameters.
     * @param transmissionInertia_input Vector of transmission inertia parameters.
     * @param transmissionInertia_eps_input Vector of transmission inertia uncertainty parameters.
     * @param offset_input Vector of offset parameters.
     * @param mass_eps_input Vector of uncertainty of link mass.
     * @param com_eps_input Vector of uncertainty of link center of mass.
     * @param inertia_eps_input Vector of uncertainty of link inertia.
     */
    model(const std::string& urdf_filename,
          const VecX& friction_input,
          const VecX& friction_eps_input,
          const VecX& damping_input,
          const VecX& damping_eps_input,
          const VecX& transmissionInertia_input,
          const VecX& transmissionInertia_eps_input,
          const VecX& offset_input,
          const VecX& mass_eps_input,
          const VecX& com_eps_input,
          const VecX& inertia_eps_input);

    // /**
    //  * @brief Constructor.
    //  * 
    //  * This constructor initializes the robot model by loading it from a URDF file and a configuration file.
    //  * 
    //  * @param urdf_filename The filename of the URDF file.
    //  * @param config_filename The filename of the configuration file.
    //  */
    // model(const std::string& urdf_filename,
    //       const std::string& config_filename);

    /**
     * @brief Read robot dynamics from Pinocchio model.
     */
    void readRobotDynamicsFromPinocchio();

    /**
     * @brief Change end-effector inertial parameters.
     * 
     * This function changes the inertial parameters of the end-effector link.
     * 
     * @param new_phi The new inertial parameters.
     * @param new_phi_lb The new lower bound of inertial parameters.
     * @param new_phi_ub The new upper bound of inertial parameters.
     */
    void changeEndEffectorInertial(const Vec10& new_phi, 
                                   const Vec10& new_phi_lb,
                                   const Vec10& new_phi_ub);

    /**
     * @brief Print model information.
     */
    void print();

    /**
     * @brief Destructor.
     */
    ~model() = default;
};

} // namespace Model
} // namespace KinovaRobustControl
