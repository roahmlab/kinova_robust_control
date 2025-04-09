#include "trajectories/TrajectoryManager.hpp"

namespace KinovaRobustControl
{
bool TrajectoryManager::add_trajectory(const std::shared_ptr<const Trajectory>& traj) 
{
    // check if the trajectory is already in the queue
    for (const auto& existing_traj : trajectories) 
    {
        if (traj.get() == existing_traj.get()) 
        {
            // already in the queue, do nothing and return
            return false;
        }
    }

    // get global system time in seconds right now
    const double t_rel = 
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count()
                / 1.0e9;

    // this is strictly forbidden!!!
    // the trajctory must start in the future
    if (t_rel > traj->start_time) 
    {
        // printf("Warning: The trajectory is already started!\n");
        return false;
    }

    if (trajectories.empty()) {
        trajectories.push_back(traj);
        printf("Received a new trajectory, start at %.9f with duration %f\n", traj->start_time, traj->duration);
        return true;
    }

    const auto& last_traj = trajectories.back();

    // this is strictly forbidden!!!
    // the incoming trajectories must follow the right time sequence
    if (traj->start_time < last_traj->start_time + last_traj->duration) {
        // throw std::invalid_argument("The new trajectory starts before the last one!");
        printf("Warning: The new trajectory starts before the last one!\n");
        return false;
    }

    // the incoming trajectory starts right after the last one
    // and we want a smooth transfer
    if (traj->start_time == last_traj->start_time + last_traj->duration) {
        const TrajectoryData last_traj_end = 
            last_traj->compute(last_traj->start_time + last_traj->duration); 
        const TrajectoryData traj_start = 
            traj->compute(traj->start_time);

        if (!Utils::ifTwoVectorEqual(Utils::wrapToPi(last_traj_end.pos), Utils::wrapToPi(traj_start.pos), 1e-4)) {
            Utils::ifTwoVectorEqual(Utils::wrapToPi(last_traj_end.pos), Utils::wrapToPi(traj_start.pos), 1e-4, true);
            // throw std::invalid_argument("The new trajectory does not start smoothly in position!");
            printf("Warning: The new trajectory does not start smoothly in position!\n");
            return false;
        }
        if (!Utils::ifTwoVectorEqual(last_traj_end.vel, traj_start.vel, 1e-4)) {
            Utils::ifTwoVectorEqual(last_traj_end.vel, traj_start.vel, 1e-4, true);
            // throw std::invalid_argument("The new trajectory does not start smoothly in velocity!");
            printf("Warning: The new trajectory does not start smoothly in velocity!\n");
            return false;
        }
        // this condition is relaxed for ARMTD trajectories
        // if (!Utils::ifTwoVectorEqual(last_traj_end.acc, traj_start.acc, 1e-4)) {
        //     Utils::ifTwoVectorEqual(last_traj_end.acc, traj_start.acc, 1e-4, true);
        //     throw std::invalid_argument("The new trajectory does not start smoothly in acceleration!");
        // }
    }

    trajectories.push_back(traj);
    printf("Received a new trajectory, start at %.9f with duration %f\n", traj->start_time, traj->duration);

    return true;
}

TrajectoryData TrajectoryManager::get_desired(const decltype(std::chrono::system_clock::now()) t) 
{
    // get time relative to the trajectory manager's start time (seconds)
    const double t_rel = 
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            t.time_since_epoch()).count()
                / 1.0e9;

    // initialize an invalid trajectory
    // the controller should apply zero torque (do nothing) in this case
    TrajectoryData result;
    result.pos = VecX::Zero(0);
    result.vel = VecX::Zero(0);
    result.acc = VecX::Zero(0);

    while (!trajectories.empty())
    {
        const auto& first_traj = trajectories.front();

        if (t_rel < first_traj->start_time)
        {
            // printf("TrajectoryManager::get_desired: The first trajectory still in the future!\n");
            return result;
        }
        // the first trajectory has been executed
        // remove it from the queue and move on to the next one
        else if (t_rel > first_traj->start_time + first_traj->duration)
        {
            if (trajectories.size() > 1) 
            {
                trajectories.pop_front();
                trajectory_id++;
                printf("TrajectoryManager::get_desired: The first trajectory has been executed!\n");
            }
            else // if (trajectories.size() == 1) 
            {
                if (t_rel <= first_traj->start_time + first_traj->trajectory_duration) 
                {
                    // the last trajectory should be stopped at the current time
                    // but no new trajectory is received yet
                    // we will just continue this trajectory until its end
                    break;
                }
                else 
                {
                    // the last trajectory has been completely executed
                    // we will just let the robot stop here
                    trajectories.pop_front();
                    trajectory_id++;
                    printf("TrajectoryManager::get_desired: The last trajectory has been executed!\n");
                }
            }
            // else // if (trajectories.size() == 0) 
            // {
            //     // this is impossible!
            // }
        }
        // passed the check!
        // we stay at the current trajectory
        else
        {
            break;
        }
    }

    // fetch valid trajectory
    if (trajectories.empty())
    {
        return result;
    }

    return trajectories.front()->compute(t_rel);
}
} // namespace KinovaRobustControl