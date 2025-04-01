#pragma once

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>

#include <iostream>

#include "roahm_trajectories/Trajectories.hpp"

namespace Roahm
{

namespace py = pybind11;

class TrajectoryPybindWrapper
{
public:
    TrajectoryPybindWrapper() = default;

    void setup(
        double start_time_input, 
        double trajectory_duration_input, 
        double duration_input, 
        int dof_input,
        int trajectory_type_input,
        const py::array_t<double> traj_data_input);

    py::tuple compute(const double t);

private:
    std::shared_ptr<Trajectory> trajPtr_;

    Eigen::VectorXd pos_result;
    Eigen::VectorXd vel_result;
    Eigen::VectorXd acc_result;
};

} // namespace Roahm