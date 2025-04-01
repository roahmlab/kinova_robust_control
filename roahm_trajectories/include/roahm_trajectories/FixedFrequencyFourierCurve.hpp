#pragma once

#include "roahm_utils/Utils.hpp"

namespace Roahm 
{
namespace FixedFreqeuncyFourierCurve
{

using VecX = Eigen::VectorXd;
using MatX = Eigen::MatrixXd;

constexpr int FOURIER_DEGREE = 5;

inline void compute(
    const double t_rel,
    const double trajectory_duration,
    const int dof,
    const double* data,
    VecX& pos,
    VecX& vel,
    VecX& acc) 
{
    const int degree = FOURIER_DEGREE;

    VecX F = VecX ::Zero(2 * degree + 1);
    VecX dF = VecX ::Zero(2 * degree + 1);
    VecX ddF = VecX ::Zero(2 * degree + 1);

    VecX F0 = VecX ::Zero(2 * degree + 1);
    VecX dF0 = VecX ::Zero(2 * degree + 1);

    // recover Fourier coefficients from traj_data
    int process_idx = 0;

    MatX coefficients(2 * degree + 1, dof);
    for (int i = 0; i < dof; i++) {
        for (int j = 0; j < 2 * degree + 1; j++) {
            coefficients(j, i) = data[process_idx];
            process_idx++;
        }
    }

    VecX q_act0(dof);
    for(int i = 0; i < dof; i++) {
        q_act0(i) = data[process_idx];
        process_idx++;
    }

    VecX q_act_d0(dof);
    for(int i = 0; i < dof; i++) {
        q_act_d0(i) = data[process_idx];
        process_idx++;
    }

    const double w = data[process_idx];

    if (w <= 0) {
        throw std::invalid_argument("The base frequency should be positive!");
    }

    pos = VecX::Zero(dof);
    vel = VecX::Zero(dof);
    acc = VecX::Zero(dof);

    for (int i = 0; i < dof; i++) {
        const VecX& kernel = coefficients.col(i);
        
        ddF(0) = 1;
        dF(0)  = t_rel;
        F(0)   = t_rel * t_rel * 0.5;

        dF0(0) = 0;
        F0(0) = 0;

        for (int j = 0; j < degree; j++) {
            double jt = (j + 1) * t_rel;
            double sinjwt = sin(w * jt);
            double cosjwt = cos(w * jt);

            ddF(2 * j + 1) = cosjwt;
            ddF(2 * j + 2) = sinjwt;

            double jw = (j + 1) * w;
            dF(2 * j + 1) = sinjwt / jw;
            dF(2 * j + 2) = -cosjwt / jw;
            dF0(2 * j + 2) = -1 / jw;

            double j2w2 = jw * jw;
            F(2 * j + 1) = -cosjwt / j2w2;
            F(2 * j + 2) = -sinjwt / j2w2;
            F0(2 * j + 1) = -1 / j2w2;
        }

        acc(i) = ddF.dot(kernel);

        double q_d_raw = dF.dot(kernel);
        double q_d_raw0 = dF0.dot(kernel);
        vel(i) = q_d_raw + (q_act_d0(i) - q_d_raw0);

        double q_raw = F.dot(kernel) + (q_act_d0(i) - q_d_raw0) * t_rel;
        double q_raw0 = F0.dot(kernel);
        pos(i) = q_raw + (q_act0(i) - q_raw0);
    }
}

} // namespace FixedFreqeuncyFourierCurve
} // namespace Roahm