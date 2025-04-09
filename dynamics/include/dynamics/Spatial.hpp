#pragma once
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <cstdlib>

namespace KinovaRobustControl
{
namespace Spatial
{

using Vec3 = Eigen::Vector3d; //!< Type alias for Eigen 3D vector.
using Mat3 = Eigen::Matrix3d; //!< Type alias for Eigen 3x3 matrix.
using Vec6 = Eigen::Matrix<double, 6, 1>; //!< Type alias for Eigen 6D vector.
using Mat6 = Eigen::Matrix<double, 6, 6>; //!< Type alias for Eigen 6x6 matrix.

/**
 * @brief Calculate the spatial rotation matrix and the unit screw vector.
 * 
 * This function computes the spatial rotation matrix and the unit screw vector 
 * for a given joint type and joint configuration.
 * 
 * @param Xj The spatial rotation matrix to be computed.
 * @param S The unit screw vector to be computed.
 * @param jtyp The type of the joint.
 * @param q The joint configuration.
 */
void jcalc(Mat6& Xj, 
           Vec6& S, 
           const int jtyp, 
           const double q);

/**
 * @brief Calculate the spatial rotation matrix, its time derivative, and the unit screw vector.
 * 
 * This function computes the spatial rotation matrix, its time derivative, and the unit screw vector 
 * for a given joint type, joint configuration, and joint velocity.
 * 
 * @param Xj The spatial rotation matrix to be computed.
 * @param dXjdt The time derivative of the spatial rotation matrix to be computed.
 * @param S The unit screw vector to be computed.
 * @param jtyp The type of the joint.
 * @param q The joint configuration.
 * @param q_d The joint velocity.
 */
void jcalc(Mat6& Xj, 
           Mat6& dXjdt,
           Vec6& S, 
           const int jtyp, 
           const double q,
           const double q_d);

/**
 * @brief Compute the cross-product matrix of a 6D vector.
 * 
 * This function computes the cross-product matrix of a 6D vector.
 * 
 * @param v The 6D vector.
 * @return Mat6 The resulting cross-product matrix.
 */
Mat6 crm(const Vec6& v);

/**
 * @brief Compute the skew-symmetric matrix of a 3D vector.
 * 
 * This function computes the skew-symmetric matrix of a 3D vector.
 * 
 * @param v The 3D vector.
 * @return Mat3 The resulting skew-symmetric matrix.
 */
Mat3 skew(const Vec3& v);

/**
 * @brief Compute the skew-symmetric vector of a 3x3 matrix.
 * 
 * This function computes the skew-symmetric vector of a 3x3 matrix.
 * 
 * @param m The 3x3 matrix.
 * @return Vec3 The resulting skew-symmetric vector.
 */
Vec3 skew(const Mat3& m);

/**
 * @brief Compute the Plücker transform matrix from a rotation matrix and a translation vector.
 * 
 * This function computes the Plücker transform matrix from a rotation matrix and a translation vector.
 * 
 * @param R The rotation matrix.
 * @param p The translation vector.
 * @return Mat6 The resulting Plücker transform matrix.
 */
Mat6 plux(const Mat3& R, const Vec3& p);

} // namespace Spatial
} // namespace KinovaRobustControl
