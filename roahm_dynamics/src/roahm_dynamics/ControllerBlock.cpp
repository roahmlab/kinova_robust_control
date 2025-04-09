#include "roahm_dynamics/ControllerBlock.hpp"

// using namespace std;

namespace Roahm
{
namespace Dynamics
{
void ControllerBlock::run()
{
    // init output
    ControlMsg ctrl(robot_model->NB);
    default_output->set(ctrl);

    // initialization
    // init values
    {
        logger.info("Check input initialization...");
        // make sure planner initialized
        auto traj_ptr = traj_port->get_new<Trajectory>();
        t0 = std::chrono::system_clock::now();
        auto state_ptr = state_port->get_new<Measurement>();
    }

    /// loop
    logger.info("Controller Block {} started", get_block_name());
    while (true)
    {
        // time between controller call
        auto now = std::chrono::system_clock::now();
        double dt =
            std::chrono::duration_cast<std::chrono::microseconds>(now - t_prev)
                .count() /
            1.0e6;
        t_prev = now;

        // retrieving values
        const auto& state = state_port->get_new<Measurement>();
        const auto& latest_traj = traj_port->get_nowait<Trajectory>();
        
        bool if_new_trajectory_arrived = false;
        if (latest_traj.get() != nullptr) {
            if_new_trajectory_arrived = trajectories->add_trajectory(latest_traj);
        }

        // if received a new trajectory, read from the end effector inertial parameters file
        // to get the latest end effector inertial parameters
        if (if_new_trajectory_arrived) {
            endeffector_inertial_file.clear();
            endeffector_inertial_file.seekg(0, std::ios::beg);
            MatX endeffector_params = Utils::initializeEigenMatrixFromFile(endeffector_inertial_file);
            if (endeffector_params.rows() == 3 || endeffector_params.cols() == 10) {
                const Vec10& phi_new = endeffector_params.row(0);
                const Vec10& phi_lb_new = endeffector_params.row(1);
                const Vec10& phi_ub_new = endeffector_params.row(2);
                robot_model->changeEndEffectorInertial(phi_new, phi_lb_new, phi_ub_new);
                logger.info("End effector inertial parameters updated!");
            }
        }

        // get desired trajectory
        const auto desired_traj_data = trajectories->get_desired(now);
        const Trajectory* traj = trajectories->get_current_trajectory();
        const size_t current_trajectory_id = trajectories->get_trajectory_id();
        const VecX& pdes = desired_traj_data.pos;
        const VecX& vdes = desired_traj_data.vel;
        const VecX& ades = desired_traj_data.acc;

        // update trajectory status
        if (current_trajectory_id != previous_trajectory_id) {
             // previously tracked a trajectory and just finished, close the current torque log file
            data_file.close();

            // now it's going to be a new trajectory, create a new torque log file
            data_file.open("log/data_output_" + std::to_string(current_trajectory_id) + ".txt");

            if (traj != nullptr) {
                printf("New trajectory started: %ld at time %.9f", 
                        current_trajectory_id, 
                        std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count() / 1.0e9);
                printf(" Trajectory duration: %.9f\n", traj->duration);
            }

            // reset the controller
            reset();
        }

        previous_trajectory_id = current_trajectory_id;

        if (traj == nullptr || 
            pdes.size() != robot_model->NB ||
            vdes.size() != robot_model->NB || 
            ades.size() != robot_model->NB) 
        {
            // no valid trajectory at the moment
            if (q_current.size() == 0) {
                // q_current is not initialized, no trajectory has been received
                // simply do nothing
                ctrl.frame_id = state->frame_id + 1;
                ctrl.stamp = std::chrono::system_clock::now();
                ctrl.torque = VecX::Zero(robot_model->NB);
                ctrl.is_gripper_open = true;

                // update tracking status
                is_tracking_trajectory = false;

                // do not send torque command
                // default_output->set<ControlMsg>(ctrl);
            }
            else if (q_current.size() == robot_model->NB) {
                // q_current is initialized, trajectory has been received before
                // keep the robot static at q_current
                VecX pdes_static = q_current;
                VecX vdes_static = VecX::Zero(robot_model->NB);
                VecX ades_static = VecX::Zero(robot_model->NB);

                // update errors
                const VecX e = Utils::wrapToPi(pdes_static - state->pos);
                const VecX ed = vdes_static - state->vel;

                // call for torque update
                InputPack inputs{pdes_static, vdes_static, ades_static, 
                                 state->pos,  state->vel, 
                                 e,           ed,   
                                 dt};

                ctrl.frame_id = state->frame_id + 1;
                ctrl.stamp = std::chrono::system_clock::now();
                ctrl.torque = update(inputs);
                ctrl.is_gripper_open = is_gripper_open;
                
                // check torque limits
                bool exceed_limit = false;
                for (int i = 0; i < robot_model->NB; i++) {
                    if (fabs(ctrl.torque(i)) >= TORQUE_LIMITS[i]) {
                        logger.warn("Torque exceeds limit when trying to stay at current position!");
                        exceed_limit = true;

                        printf("Motor %d: Torque: %f, Limit: %f\n", i, ctrl.torque(i), TORQUE_LIMITS[i]);
                        
                        // throw std::runtime_error("controller.cpp: update(): Torque exceeds limit!");

                        // saturate torque (95% of limit)
                        ctrl.torque(i) = copysign(0.95 * TORQUE_LIMITS[i], ctrl.torque(i));
                    }
                }

                // send out torque command but do not log the torque
                default_output->set<ControlMsg>(ctrl);

                // log tracking error
                auto duration_since_epoch = ctrl.stamp.time_since_epoch();
                tracking_error_file << std::chrono::duration_cast<std::chrono::nanoseconds>(duration_since_epoch).count() << " ";
                tracking_error_file << e.transpose() << " ";
                tracking_error_file << 0 << std::endl;
            }
            else {
                throw std::runtime_error("controller.cpp: run(): Invalid stopping position!");
            }
        }
        else 
        {
            // update errors
            const VecX e = Utils::wrapToPi(pdes - state->pos);
            const VecX ed = vdes - state->vel;

            // update q_current
            q_current = pdes;
            is_gripper_open = traj->is_gripper_open;

            // call for torque update
            InputPack inputs{pdes,       vdes,       ades, 
                             state->pos, state->vel, 
                             e,          ed,   
                             dt};

            ctrl.frame_id = state->frame_id + 1;
            ctrl.stamp = std::chrono::system_clock::now();
            ctrl.torque = update(inputs);

            ctrl.is_gripper_open = traj->is_gripper_open;
            
            // check torque limits
            VecX torque_copy = ctrl.torque;
            bool exceed_limit = false;
            for (int i = 0; i < robot_model->NB; i++) {
                if (fabs(ctrl.torque(i)) >= TORQUE_LIMITS[i]) {
                    logger.warn("Torque exceeds limit when tracking a trajectory!");
                    exceed_limit = true;

                    printf("Motor %d: Torque: %f, Limit: %f\n", i, ctrl.torque(i), TORQUE_LIMITS[i]);

                    // throw std::runtime_error("controller.cpp: update(): Torque exceeds limit!");

                    // saturate torque (95% of limit)
                    ctrl.torque(i) = copysign(0.95 * TORQUE_LIMITS[i], ctrl.torque(i));
                }
            }

            if (exceed_limit) {
                // std::cerr << "Debug info:\n";
                std::cerr << "pdes: " << pdes.transpose() << std::endl;
                std::cerr << "vdes: " << vdes.transpose() << std::endl;
                std::cerr << "ades: " << ades.transpose() << std::endl;
                std::cerr << "state->pos: " << state->pos.transpose() << std::endl;
                std::cerr << "state->vel: " << state->vel.transpose() << std::endl;
                std::cerr << "e: " << e.transpose() << std::endl;
                std::cerr << "ed: " << ed.transpose() << std::endl;
                std::cerr << "tau: " << torque_copy.transpose() << std::endl;
                // throw std::runtime_error("controller.cpp: update(): Torque exceeds limit!");
            }

            // send out torque command
            default_output->set<ControlMsg>(ctrl);

            // update tracking status
            is_tracking_trajectory = true;
            // log torque
            auto duration_since_epoch = ctrl.stamp.time_since_epoch();
            data_file << std::chrono::duration_cast<std::chrono::nanoseconds>(duration_since_epoch).count() << " ";
            data_file << state->pos.transpose() << " ";
            data_file << state->vel.transpose() << " ";
            data_file << ctrl.torque.transpose() << std::endl;
            // data_file << -state->torque.transpose() << std::endl;
            tracking_error_file << std::chrono::duration_cast<std::chrono::nanoseconds>(duration_since_epoch).count() << " ";
            tracking_error_file << e.transpose() << " ";
            tracking_error_file << 1 << std::endl;
        }
    }
}

ControllerBlock::ControllerBlock(const std::string &block_name,
                                 const std::shared_ptr<Model::model> &robot_model_input)
    : ::Roahm::System::BaseBlock(block_name,
                                 System::OutputPort<ControlMsg>::make_shared()),
      traj_port(register_input<Trajectory>("trajectory")),
      state_port(register_input<Measurement>("state")),
      robot_model(robot_model_input),
      t0(std::chrono::system_clock::now())
{
    trajectories = std::make_shared<TrajectoryManager>();
    
    data_file.open("log/data_output_" + std::to_string(trajectories->get_trajectory_id()) + ".txt");
    data_file << std::setprecision(12);

    tracking_error_file.open("log/tracking_error.txt");

    // make sure the endeffector_inertial_parameters.txt file is cleared
    std::ofstream endeffector_inertial_clear;
    endeffector_inertial_clear.open("models/endeffector_inertial_parameters.txt");

    endeffector_inertial_file.open("models/endeffector_inertial_parameters.txt");
    if (!endeffector_inertial_file.is_open()) {
        throw std::runtime_error("controller.cpp: ControllerBlock(): Cannot open endeffector_inertial.txt!");
    }

    q_current = VecX::Zero(0);
}

ControllerBlock::~ControllerBlock() {
    data_file.close();
    tracking_error_file.close();
}

} // namespace Dynamics
} // namespace Roahm