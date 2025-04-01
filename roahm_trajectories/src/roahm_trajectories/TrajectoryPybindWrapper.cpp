#include "roahm_trajectories/TrajectoryPybindWrapper.hpp"

namespace Roahm
{

void TrajectoryPybindWrapper::setup(
    double start_time_input, 
    double trajectory_duration_input, 
    double duration_input, 
    int dof_input,
    int trajectory_type_input,
    const py::array_t<double> traj_data_input)
{
    py::buffer_info buf = traj_data_input.request();

    if (buf.ndim > TRAJECTORY_DATA_SIZE) 
    {
        std::cerr << "traj_data_input size: " << buf.ndim << std::endl;
        throw std::invalid_argument("The trajectory data size is too large!");
    }

    const double* ptr = (const double*)buf.ptr;

    trajPtr_ = std::make_shared<Trajectory>(
        start_time_input, 
        trajectory_duration_input, 
        duration_input, 
        dof_input,
        trajectory_type_input,
        ptr);

    pos_result = Eigen::VectorXd::Zero(dof_input);
    vel_result = Eigen::VectorXd::Zero(dof_input);
    acc_result = Eigen::VectorXd::Zero(dof_input);
}

py::tuple TrajectoryPybindWrapper::compute(const double t) 
{
    const TrajectoryData result = trajPtr_->compute(t);
    
    pos_result = result.pos;
    vel_result = result.vel;
    acc_result = result.acc;

    py::array_t<double> pos = py::array_t<double>(
        {pos_result.size()}, {sizeof(double)}, pos_result.data());
    py::array_t<double> vel = py::array_t<double>(
        {vel_result.size()}, {sizeof(double)}, vel_result.data());
    py::array_t<double> acc = py::array_t<double>(
        {acc_result.size()}, {sizeof(double)}, acc_result.data());

    return py::make_tuple(pos, vel, acc);
}

} // namespace Roahm