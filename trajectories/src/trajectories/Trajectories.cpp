#include "trajectories/Trajectories.hpp"

namespace KinovaRobustControl 
{

Trajectory::Trajectory() {
    std::memset(traj_data, 0, sizeof(traj_data));
}

Trajectory::Trajectory(
    double start_time_input, 
    double trajectory_duration_input, 
    double duration_input, 
    int dof_input,
    int trajectory_type_input,
    const double* traj_data_input) :
        start_time(start_time_input), 
        trajectory_duration(trajectory_duration_input), 
        duration(duration_input),
        dof(dof_input),
        trajectory_type(trajectory_type_input),
        is_gripper_open(false), reset(false)
{
    if (dof <= 0) {
        throw std::invalid_argument("The degree of freedom should be positive!");
    }

    if (dof > 7) {
        throw std::invalid_argument("The degree of freedom should be less than or equal to 7!");
    }

    if (duration > trajectory_duration) {
        throw std::invalid_argument("The duration of the trajectory is longer than the trajectory itself!");
    }

    std::memset(traj_data, 0, sizeof(traj_data));
    if (trajectory_type == ARMTD_TRAJ)
    {
        std::copy(traj_data_input, traj_data_input + 3 * dof + 1, traj_data);
    }
    else if (trajectory_type == ARMOUR_TRAJ)
    {
        std::copy(traj_data_input, traj_data_input + 4 * dof, traj_data);
    }
    else if (trajectory_type == FOURIER_TRAJ)
    {
        std::copy(traj_data_input, traj_data_input + (2 * FixedFreqeuncyFourierCurve::FOURIER_DEGREE + 3) * dof + 1, traj_data);
    }
    else if (trajectory_type == FIFTH_ORDER_BEZIER_TRAJ) {
        std::copy(traj_data_input, traj_data_input + 6 * dof, traj_data);
    }
    else
    {
        throw std::invalid_argument("Invalid trajectory type!");
    }
}

TrajectoryData Trajectory::compute(double t) const 
{
    if (t < start_time) {
        throw std::runtime_error("Time is out of range! This trajectory has not started yet!");
    }

    if (t > start_time + duration) {
        // warning("Time is out of the desired range! This trajectory should have already ended!");
    }

    if (t > start_time + trajectory_duration) {
        throw std::runtime_error("Time is out of range of the definition of the trajectorty! Should have switched to the next trajectory!");
    }

    TrajectoryData result;
    const double t_rel = t - start_time;

    if (trajectory_type == ARMTD_TRAJ)
    {
        ArmtdCurve::compute(
            t_rel, 
            trajectory_duration, 
            dof, 
            traj_data, 
            result.pos, 
            result.vel, 
            result.acc);
    }
    else if (trajectory_type == ARMOUR_TRAJ)
    {
        ArmourBezierCurve::compute(
            t_rel, 
            trajectory_duration, 
            dof, 
            traj_data, 
            result.pos, 
            result.vel, 
            result.acc);
    }
    else if (trajectory_type == FOURIER_TRAJ)
    {
        FixedFreqeuncyFourierCurve::compute(
            t_rel, 
            trajectory_duration, 
            dof, 
            traj_data, 
            result.pos, 
            result.vel, 
            result.acc);
    }
    else if (trajectory_type == FIFTH_ORDER_BEZIER_TRAJ)
    {
        FifthOrderBezierCurve::compute(
            t_rel, 
            trajectory_duration, 
            dof, 
            traj_data, 
            result.pos, 
            result.vel, 
            result.acc);
    }
    else
    {
        throw std::invalid_argument("Invalid trajectory type!");
    }

    result.pos = Utils::wrapToPi(result.pos);

    return result;
}

} // namespace KinovaRobustControl