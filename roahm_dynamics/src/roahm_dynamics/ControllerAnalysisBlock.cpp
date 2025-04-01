#include "roahm_dynamics/ControllerAnalysisBlock.hpp"
#include <pthread.h>

namespace Roahm
{
namespace Dynamics
{
void ControllerAnalysisBlock::run()
{
    {
        logger.info("Check Traj initailization...");
        auto ctr_ptr = traj_port->get_new<Trajectory>();
        t0 = std::chrono::system_clock::now();
    }

    /// loop
    logger.info("Controller Analysis Block started");
    Debug debug;

    while (true)
    {
        // time
        auto now = std::chrono::system_clock::now();

        // retrieving values
        const auto& state = state_port->get_new<Measurement>();
        const auto& latest_traj = traj_port->get_nowait<Trajectory>();
        
        if (latest_traj.get() != nullptr) {
            trajectories->add_trajectory(latest_traj);
        }

        // get desired trajectory
        const auto desired_traj_data = trajectories->get_desired(now);
        const VecX& pdes = desired_traj_data.pos;
        const VecX& vdes = desired_traj_data.vel;
        const VecX& ades = desired_traj_data.acc;

        // update errors
        VecX e = Roahm::Utils::wrapToPi(pdes - state->pos);
        VecX ed = vdes - state->vel;

        auto control = control_port->get_new<ControlMsg>();

        // header
        debug.time = std::chrono::duration_cast<std::chrono::microseconds>(
                         state->timestamp - t0)
                         .count() /
                     1.0e6;
        // traj
        {
            const Trajectory* traj = trajectories->get_current_trajectory();
            if (traj != nullptr) 
            {
                std::copy(traj->traj_data, traj->traj_data + TRAJECTORY_DATA_SIZE,
                        debug.traj.traj_data.begin());
                debug.traj.start_time = traj->start_time;
                debug.traj.trajectory_duration = traj->trajectory_duration;
                debug.traj.duration = traj->duration;
                debug.traj.dof = traj->dof;
                debug.traj.trajectory_type = traj->trajectory_type;
                debug.traj.is_gripper_open = traj->is_gripper_open;
                debug.traj.reset = traj->reset;
            }
        }

        // control
        {
            std::copy(control->control.data(),
                      control->control.data() + control->control.size(),
                      debug.torque.torque.begin());
            debug.torque.is_gripper_open = control->is_gripper_open;
            debug.torque.frame_id = control->frame_id;
        }

        // joint states
        {
            std::copy(state->pos.data(), state->pos.data() + state->pos.size(),
                      debug.joint_states.pos.begin());

            std::copy(state->vel.data(), state->vel.data() + state->vel.size(),
                      debug.joint_states.vel.begin());

            std::copy(state->torque.data(),
                      state->torque.data() + state->torque.size(),
                      debug.joint_states.torque.begin());

            std::copy(state->torque.data(),
                      state->torque.data() + state->torque.size(),
                      debug.joint_states.torque.begin());

            debug.joint_states.frame_id = state->frame_id;
        }

        // error
        {
            std::copy(e.data(), e.data() + e.size(), debug.e.begin());
            std::copy(ed.data(), ed.data() + ed.size(), debug.ed.begin());
        }

        // desired
        {
            std::copy(pdes.data(), pdes.data() + pdes.size(),
                      debug.q_des.begin());
            std::copy(vdes.data(), vdes.data() + vdes.size(),
                      debug.qd_des.begin());
            std::copy(ades.data(), ades.data() + ades.size(),
                      debug.qdd_des.begin());
        }

        pub->publish(debug);
    }
}

ControllerAnalysisBlock::ControllerAnalysisBlock(const std::string &block_name,
                                                 const std::string &pub_topic,
                                                 rclcpp::Node::SharedPtr node)
    : System::BaseBlock(block_name, System::OutputPort<Debug>::make_shared()),
      traj_port(register_input<Trajectory>("trajectory")),
      state_port(register_input<Measurement>("state")),
      control_port(register_input<ControlMsg>("control")), node(node),
      pub(node->create_publisher<Debug>(pub_topic, 10))
{
    trajectories = std::make_shared<TrajectoryManager>();
}
} // namespace Dynamics
} // namespace Roahm
