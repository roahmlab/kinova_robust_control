#pragma once

#include <eigen3/Eigen/Dense>

namespace Roahm 
{
namespace ArmtdCurve
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
    const double t_brake = data[3 * dof]; // time to brake

    if (t_brake >= trajectory_duration) 
    {
        throw std::invalid_argument("Brake time is out of range!");
    }
    else if (t_brake <= 0) 
    {
        throw std::invalid_argument("Brake time is non-positive!");
    }

    pos.resize(dof);
    vel.resize(dof);
    acc.resize(dof);

    if (t_rel < t_brake) 
    {
        for (int i = 0; i < dof; i++) 
        {
            const double q0 = data[3 * i + 0];
            const double qd0 = data[3 * i + 1];
            const double k = data[3 * i + 2];

            pos(i) = q0 + 
                     qd0 * t_rel + 
                     0.5 * k * t_rel * t_rel;
            vel(i) = qd0 + 
                     k * t_rel;
            acc(i) = k;
        }
    }
    else 
    {
        for (int i = 0; i < dof; i++) 
        {
            const double q0 = data[3 * i + 0];
            const double qd0 = data[3 * i + 1];
            const double k = data[3 * i + 2];

            const double q_peak = q0 + 
                                  qd0 * t_brake + 
                                  0.5 * k * t_brake * t_brake;
            const double qd_peak = qd0 + 
                                   k * t_brake;
            const double qdd_brake = -qd_peak / (trajectory_duration - t_brake);

            const double t_rel_brake = t_rel - t_brake;

            pos(i) = q_peak + 
                     qd_peak * t_rel_brake + 
                     0.5 * qdd_brake * t_rel_brake * t_rel_brake;
            vel(i) = qd_peak +
                     qdd_brake * t_rel_brake;
            acc(i) = qdd_brake;
        }
    }
}
    
} // namespace Trajectory::ArmtdCurve
} // namespace Roahm