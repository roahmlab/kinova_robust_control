/**
 * @brief Trajectory base class
 * @file
 **/
#pragma once

#include "trajectories/ArmtdCurve.hpp"
#include "trajectories/ArmourBezierCurve.hpp"
#include "trajectories/FixedFrequencyFourierCurve.hpp"
#include "trajectories/FifthOrderBezierCurve.hpp"

namespace KinovaRobustControl 
{

#define TRAJECTORY_DATA_SIZE 100

#define ARMTD_TRAJ 0x3001
#define ARMOUR_TRAJ 0x3002
#define FOURIER_TRAJ 0x3003
#define FIFTH_ORDER_BEZIER_TRAJ 0x3004

/**
 * @brief the output of the trajectory class
 **/
typedef struct TrajectoryData_ {
    Eigen::VectorXd pos;
    Eigen::VectorXd vel;
    Eigen::VectorXd acc;
} TrajectoryData;

/**
 * @brief trajectory class to be passed to controllerblock
 * @note the implementation is required and can be found in trajectories/
 **/
class Trajectory
{
public:
    using VecX = Eigen::VectorXd;

    Trajectory();
    
    Trajectory(
        double start_time_input, 
        double trajectory_duration_input, 
        double duration_input, 
        int dof_input,
        int trajectory_type_input,
        const double* traj_data_input);

    ~Trajectory() = default;

    /**
     * @brief compute the trajectory at time t
     * @param t time
     * @return TrajectoryData
     * @note the time should be in the range of [start_time, start_time + duration]
     *       if t is out of range, the function should return the last point of the trajectory
     *       the implementation is required and can be found in trajectories/
     **/
    virtual TrajectoryData compute(double t) const;

    // the traj_data required to define or compute the trajectory
    // this contains, for example, coefficients of the Bezier curve
    double traj_data[TRAJECTORY_DATA_SIZE] = {0};

    // global start time of the trajectory
    double start_time = 0.0;

    // actual duration of the trajectory
    double trajectory_duration = 0.0;

    // duration we would like to play the trajectory
    // duration should always be less than or equal to trajectory_duration
    //
    // if duration is less than trajectory_duration:
    //     the controller will switch to the next trajectory after duration
    //     this is used for receding horizon planning and control
    //     but the next trajectory should be sent before duration
    //     and the position, velocity, and acceleration should be continuous at duration between the two trajectories
    //     
    // if duration is equal to trajectory_duration:
    //     the controller will switch to the next trajectory only when the entire trajectory is played
    //     this is used for playing the entire trajectory
    //     and the end velocity and acceleration is supposed to be zero so that the robot can take a full stop
    double duration = 0.0;

    // degree of freedom of the robot
    int dof = 0;

    // trajectory type (defined in customized_msgs/TrajectoryTypeMacros.hpp)
    int trajectory_type = ARMOUR_TRAJ;

    bool is_gripper_open = false;
    bool reset = false;
};
} // namespace KinovaRobustControl