/**
 * @brief A class to manage multiple incoming trajectories
 * @file
 **/
#pragma once

#include <deque>
#include <iostream>

#include "roahm_utils/Utils.hpp"
#include "roahm_trajectories/Trajectories.hpp"
#include "roahm_system/BaseBlock.hpp"

namespace Roahm 
{
class TrajectoryManager
{
public:
    using VecX = Eigen::VectorXd;

    TrajectoryManager() = default;

    ~TrajectoryManager() = default;

    /**
     * @brief Add a trajectory to the manager
     * @param t The time the trajectory message is sent (global time)
     * @param traj The trajectory to add
     **/
    bool add_trajectory(const std::shared_ptr<const Trajectory>& traj);

    /**
     * @brief Get the desired trajectory data (position, velocity, acceleration) at a given time
     * @param t The time to get the trajectory at (global time)
     * @return The desired trajectory data
     **/
    TrajectoryData get_desired(const decltype(std::chrono::system_clock::now()) t);

    bool is_empty() const
    {
        return trajectories.empty();
    }

    /**
     * @brief Get the current trajectory
     * @return The current trajectory
     **/
    const Trajectory* get_current_trajectory() const
    {
        if (trajectories.empty()) 
        {
            return nullptr;
        }

        return trajectories.front().get();
    }

    /**
     * @brief Reset the manager
     **/
    void reset()
    {
        trajectories.clear();
    }

    /**
     * @brief Get the trajectory id
     * @return The trajectory id
     **/
    size_t get_trajectory_id() const
    {
        return trajectory_id;
    }

private:
    std::deque<std::shared_ptr<const Trajectory>> trajectories;

    // number of trajectories finished
    size_t trajectory_id = 0;
};
} // namespace Roahm