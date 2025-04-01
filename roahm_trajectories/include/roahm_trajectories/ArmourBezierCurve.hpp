#pragma once

#include <eigen3/Eigen/Dense>

namespace Roahm 
{
namespace ArmourBezierCurve
{

inline void compute(
    const double t_rel,
    const double trajectory_duration,
    const int dof,
    const double* data,
    Eigen::VectorXd& pos,
    Eigen::VectorXd& vel,
    Eigen::VectorXd& acc) 
{
    const int degree = 5;

    // compute the normalized time 
    // since Bezier curves are defined in [0, 1]
    const double t = t_rel / trajectory_duration;

    // initialize binomial coefficients
    Eigen::VectorXd Bionomials = Eigen::VectorXd::Ones(degree + 1);
    for (int j = 1; j <= degree / 2; j++) 
    {
        Bionomials(j) = Bionomials(j - 1) * (degree + 1 - j) / j;
        Bionomials(degree - j) = Bionomials(j);
    }

    // assign Bezier coefficients
    Eigen::MatrixXd coefficients = Eigen::MatrixXd::Zero(degree + 1, dof);
    for (int i = 0; i < dof; i++) 
    {
        const double q0 = data[4 * i + 0];
        const double qd0 = data[4 * i + 1];
        const double qdd0 = data[4 * i + 2];
        const double q1 = data[4 * i + 3];

        coefficients(0, i) = q0;
        coefficients(1, i) = q0 + 
                             (trajectory_duration * qd0) / degree;
        coefficients(2, i) = q0 + 
                             (2.0 * trajectory_duration * qd0) / degree + 
                             (trajectory_duration * trajectory_duration * qdd0) / (degree * (degree - 1));
        coefficients(3, i) = q1;
        coefficients(4, i) = q1;
        coefficients(5, i) = q1;
    }

    // compute tA(i, j) = t(i)^j, 
    //         tB(i, j) = (1-t(i))^(degree-j)
    Eigen::VectorXd tA = Eigen::VectorXd::Ones(degree + 1);
    Eigen::VectorXd tB = Eigen::VectorXd::Ones(degree + 1);
    Eigen::VectorXd dtA = Eigen::VectorXd::Zero(degree + 1);
    Eigen::VectorXd dtB = Eigen::VectorXd::Zero(degree + 1);
    Eigen::VectorXd ddtA = Eigen::VectorXd::Zero(degree + 1);
    Eigen::VectorXd ddtB = Eigen::VectorXd::Zero(degree + 1);

    // loop to compute tA and tB
    for (int j = 1; j <= degree; j++) 
    {
        tA(j) = t * tA(j - 1);
        tB(degree - j) = (1 - t) * tB(degree - j + 1);

        dtA(j) = j * tA(j - 1);
        dtB(degree - j) = -j * tB(degree - j + 1);

        ddtA(j) = j * dtA(j - 1);
        ddtB(degree - j) = -j * dtB(degree - j + 1);
    }

    // compute the Bernstein polynomials
    Eigen::VectorXd B = Bionomials.array() * 
                            tA.array() * tB.array();
    Eigen::VectorXd dB = Bionomials.array() * 
                            (dtA.array() * tB.array() +  
                             tA.array() * dtB.array()) / 
                                trajectory_duration;
    Eigen::VectorXd ddB = Bionomials.array() * 
                            (ddtA.array() * tB.array() + 
                             2 * dtA.array() * dtB.array() + 
                             tA.array() * ddtB.array()) / 
                                (trajectory_duration * trajectory_duration);

    // compute the trajectory
    pos = coefficients.transpose() * B;
    vel = coefficients.transpose() * dB;
    acc = coefficients.transpose() * ddB;
}
    
} // namespace Trajectory::ArmourBezierCurve
} // namespace Roahm