/**
 * @brief Message structs and classes
 * @file
 **/
#pragma once
#include <chrono>
#include <cstdlib>
#include <eigen3/Eigen/Dense>
#include "trajectories/Trajectories.hpp"

namespace KinovaRobustControl
{
namespace msgs
{
/**
 * @warning default Ctor is provided because Port need them to initialize
 *          should explicitly set length if using elsewhere
 **/
struct Measurement
{
    size_t frame_id = 0;
    std::chrono::time_point<std::chrono::system_clock> stamp; 
    Eigen::VectorXd pos;
    Eigen::VectorXd vel;
    Eigen::VectorXd torque;

    /** Ctors */
    Measurement() = default;
    Measurement(uint32_t len) : 
        frame_id(0), 
        stamp(std::chrono::system_clock::now()),
        pos(len), 
        vel(len), 
        torque(len)
    {
        pos.setZero();
        vel.setZero();
        torque.setZero();
    }
};

struct ControlMsg
{
    size_t frame_id = 0;
    std::chrono::time_point<std::chrono::system_clock> stamp;
    Eigen::VectorXd torque;
    bool is_gripper_open;

    /** Ctors */
    ControlMsg() = default;
    ControlMsg(uint32_t len) : 
        frame_id(0), 
        stamp(std::chrono::system_clock::now()),
        torque(len), 
        is_gripper_open(true)
    {
        torque.setZero();
    }
};

} // namespace customized_msgs
} // namespace KinovaRobustControl
