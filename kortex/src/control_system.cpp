#include "dynamics/ARMOURControlBlock.hpp"
#include "dynamics/PassivityControlBlock.hpp"
#include "dynamics/PIDControlBlock.hpp"
#include "dynamics/GravityCompensationPIDControlBlock.hpp"
#include "utils/Utils.hpp"
#include "kortex/JointFilterBlock.hpp"
#include "kortex/KortexBlock.hpp"
#include "customized_msgs/Messages.hpp"
#include "customized_msgs/msg/kortex_measurements.hpp"
#include "customized_msgs/msg/trajectory_msg.hpp"
#include "system/HelperBlocks.hpp"
#include "system/ROSHelper.hpp"
#include "system/System.hpp"
#include <limits>

using namespace KinovaRobustControl;
using namespace customized_msgs::msg;

class Wrapper
{
  public:
    using VecX = Eigen::VectorXd;

    struct ModelSettings
    {
        // urdf path
        std::string model_path;
        std::string gripper_path;

        // motor dynamics parameters
        VecX friction;
        VecX damping;
        VecX transmissionInertia;
        VecX offset;

        // model uncertainty parameters
        VecX friction_eps;
        VecX damping_eps;
        VecX transmissionInertia_eps;
        VecX mass_eps;
        VecX com_eps;
        VecX inertia_eps;
    };

    struct PIDParams
    {
        VecX Kp;
        VecX Ki;
        VecX Kd;
    };

    struct ArmourParams
    {
        VecX Kr;
        double alpha;
        double V_max;
        double r_norm_threshold;
    };

    struct AltoffParams
    {
        VecX Kr;
        double kappa_p;
        double kappa_i;
        double phi_p;
        double phi_i;
        double max_error;
    };

    struct FilterSettings
    {
        bool use_filter;
        std::vector<double> vfilter_a;
        std::vector<double> vfilter_b;
    };

  public:
    Wrapper(const std::string &sys_name)
        : ros_node(std::make_shared<rclcpp::Node>(sys_name)),
          sys(System::System(sys_name))
    {
        declare_parameters();
        load_parameters();
    }

    void run()
    {
        configure_blocks();
        sys.run();
    }

  private:
    /**
     * @brief declare parameters to be used
     **/
    void declare_parameters()
    {
        // kinova hardware settings
        ros_node->declare_parameter<std::string>("ip", "192.168.1.10");
        ros_node->declare_parameter<int>("latency_warn", 50);
        ros_node->declare_parameter<int>("latency_error", 100);
        ros_node->declare_parameter<bool>("dual_arm", false);
        ros_node->declare_parameter<int>("robot_id", 0);
        ros_node->declare_parameter<bool>("debug", true);
        ros_node->declare_parameter<bool>("has_init_pos", false);
        ros_node->declare_parameter<bool>("is_gripper_open", true);
        ros_node->declare_parameter<std::vector<double>>(
            "init_pos",
            std::vector<double>{
                0., 0.26179939, 3.14159265, -2.2689280271795864, 0., 0.95993109, 1.57079633});

        // whether to use ros for trajectory or not
        ros_node->declare_parameter<bool>("ros_traj_enb", true);
        ros_node->declare_parameter<std::string>("ros_traj_topic",
                                                 "trajectory");

        // robot model
        ros_node->declare_parameter<std::string>(
            "model_path", "./models/urdf/gen3_2f85_fixed.urdf");
        ros_node->declare_parameter<std::string>(
            "gripper_path", "");

        ros_node->declare_parameter<std::vector<double>>(
            "friction", std::vector<double>{0, 0, 0, 0, 0, 0, 0});
        ros_node->declare_parameter<std::vector<double>>(
            "damping", std::vector<double>{0, 0, 0, 0, 0, 0, 0});
        ros_node->declare_parameter<std::vector<double>>(
            "transmissionInertia", std::vector<double>{0, 0, 0, 0, 0, 0, 0});
        ros_node->declare_parameter<std::vector<double>>(
            "offset", std::vector<double>{0, 0, 0, 0, 0, 0, 0});

        ros_node->declare_parameter<std::vector<double>>(
            "friction_eps", std::vector<double>{0, 0, 0, 0, 0, 0, 0});
        ros_node->declare_parameter<std::vector<double>>(
            "damping_eps", std::vector<double>{0, 0, 0, 0, 0, 0, 0});
        ros_node->declare_parameter<std::vector<double>>(
            "transmissionInertia_eps", std::vector<double>{0, 0, 0, 0, 0, 0, 0});
        ros_node->declare_parameter<std::vector<double>>(
            "mass_eps", std::vector<double>{0, 0, 0, 0, 0, 0, 0});
        ros_node->declare_parameter<std::vector<double>>(
            "com_eps", std::vector<double>{0, 0, 0, 0, 0, 0, 0});
        ros_node->declare_parameter<std::vector<double>>(
            "inertia_eps", std::vector<double>{0, 0, 0, 0, 0, 0, 0});

        // controller parameters
        ros_node->declare_parameter<std::string>(
            "controller_type", "PID");

        ros_node->declare_parameter<std::vector<double>>(
            "Kp", std::vector<double>{0, 0, 0, 0, 0, 0, 0});
        ros_node->declare_parameter<std::vector<double>>(
            "Ki", std::vector<double>{0, 0, 0, 0, 0, 0, 0});
        ros_node->declare_parameter<std::vector<double>>(
            "Kd", std::vector<double>{0, 0, 0, 0, 0, 0, 0});

        ros_node->declare_parameter<std::vector<double>>(
            "Kr", std::vector<double>{0, 0, 0, 0, 0, 0, 0});

        ros_node->declare_parameter<double>("alpha", 0);
        ros_node->declare_parameter<double>("V_max", 0);
        ros_node->declare_parameter<double>("r_norm_threshold", 0);

        ros_node->declare_parameter<double>("kappa_p", 0);
        ros_node->declare_parameter<double>("kappa_i", 0);
        ros_node->declare_parameter<double>("phi_p", 0);
        ros_node->declare_parameter<double>("phi_i", 0);
        ros_node->declare_parameter<double>("max_error", 1000);

        // filter
        ros_node->declare_parameter<bool>(
            "enable_joint_filter", false);
        ros_node->declare_parameter<std::vector<double>>(
            "vfilter_a", std::vector<double>{0.1, 0.1});
        ros_node->declare_parameter<std::vector<double>>(
            "vfilter_b", std::vector<double>{0.7, 0.2, 0.1});
    }

    /**
     * @brief load ros parameters to settings
     **/
    void load_parameters()
    {
        // kortex settings
        kortex_settings.JOINT_ENB = 0b1111111;
        kortex_settings.ip = ros_node->get_parameter("ip").as_string();
        kortex_settings.latency_warn = ros_node->get_parameter("latency_warn").as_int();
        kortex_settings.latency_error = ros_node->get_parameter("latency_error").as_int();
        kortex_settings.use_gripper = true;
        kortex_settings.TORQUE_LIMITS =
            // std::vector<double>{56.7, 56.7, 56.7, 56.7, 29.4, 29.4, 29.4};
            std::vector<double>{85.0, 85.0, 85.0, 85.0, 40.0, 40.0, 40.0};

        // common
        debug = ros_node->get_parameter("debug").as_bool();
        ros_traj_enb = ros_node->get_parameter("ros_traj_enb").as_bool();
        ros_traj_topic = ros_node->get_parameter("ros_traj_topic").as_string();
        has_init_pos = ros_node->get_parameter("has_init_pos").as_bool();
        init_pos = ros_node->get_parameter("init_pos").as_double_array();

        // model_settings
        model_settings.model_path =
            ros_node->get_parameter("model_path").as_string();
        model_settings.gripper_path =
            ros_node->get_parameter("gripper_path").as_string();

        model_settings.friction = Utils::convertVectorToEigen(
            ros_node->get_parameter("friction").as_double_array());
        model_settings.damping = Utils::convertVectorToEigen(
            ros_node->get_parameter("damping").as_double_array());
        model_settings.transmissionInertia = Utils::convertVectorToEigen(
            ros_node->get_parameter("transmissionInertia").as_double_array());
        model_settings.offset = Utils::convertVectorToEigen(
            ros_node->get_parameter("offset").as_double_array());

        model_settings.friction_eps = Utils::convertVectorToEigen(
            ros_node->get_parameter("friction_eps").as_double_array());
        model_settings.damping_eps = Utils::convertVectorToEigen(
            ros_node->get_parameter("damping_eps").as_double_array());
        model_settings.transmissionInertia_eps = Utils::convertVectorToEigen(
            ros_node->get_parameter("transmissionInertia_eps").as_double_array());
        model_settings.mass_eps = Utils::convertVectorToEigen(
            ros_node->get_parameter("mass_eps").as_double_array());
        model_settings.com_eps = Utils::convertVectorToEigen(
            ros_node->get_parameter("com_eps").as_double_array());
        model_settings.inertia_eps = Utils::convertVectorToEigen(
            ros_node->get_parameter("inertia_eps").as_double_array());

        // controller settings
        controller_type = ros_node->get_parameter("controller_type").as_string();

        if (controller_type == "PID") 
        {
            pid_params.Kp = Utils::convertVectorToEigen(
                ros_node->get_parameter("Kp").as_double_array());
            pid_params.Ki = Utils::convertVectorToEigen(
                ros_node->get_parameter("Ki").as_double_array());
            pid_params.Kd = Utils::convertVectorToEigen(
                ros_node->get_parameter("Kd").as_double_array());
        }
        else if (controller_type == "GRAV_PID") 
        {
            pid_params.Kp = Utils::convertVectorToEigen(
                ros_node->get_parameter("Kp").as_double_array());
            pid_params.Ki = Utils::convertVectorToEigen(
                ros_node->get_parameter("Ki").as_double_array());
            pid_params.Kd = Utils::convertVectorToEigen(
                ros_node->get_parameter("Kd").as_double_array());
        }
        else if (controller_type == "ARMOUR")
        {
            armour_params.Kr = Utils::convertVectorToEigen(
                ros_node->get_parameter("Kr").as_double_array());
            armour_params.V_max = 
                ros_node->get_parameter("V_max").as_double();
            armour_params.alpha = 
                ros_node->get_parameter("alpha").as_double();
            armour_params.r_norm_threshold =
                ros_node->get_parameter("r_norm_threshold").as_double();
        }
        // else if (controller_type == "ALTHOFF")
        // {
        //     altoff_params.Kr = Utils::DiagonalMatrixXd(
        //         ros_node->get_parameter("Kr").as_double_array());
        //     altoff_params.kappa_i =
        //         ros_node->get_parameter("kappa_i").as_double();
        //     altoff_params.kappa_p =
        //         ros_node->get_parameter("kappa_p").as_double();
        //     altoff_params.phi_i = 
        //         ros_node->get_parameter("phi_i").as_double();
        //     altoff_params.phi_p = 
        //         ros_node->get_parameter("phi_p").as_double();
        // }
        else {
            std::cerr << controller_type << std::endl;
            throw std::invalid_argument("Invalid controller type");
        }

        // filter settings
        filter_settings.use_filter =
            ros_node->get_parameter("enable_joint_filter").as_bool();
        filter_settings.vfilter_a =
            ros_node->get_parameter("vfilter_a").as_double_array();
        filter_settings.vfilter_b =
            ros_node->get_parameter("vfilter_b").as_double_array();
    }

    void configure_blocks()
    {
        if (ros_node->get_parameter("dual_arm").as_bool()) {
            robot_id = ros_node->get_parameter("robot_id").as_int();
        }

        /** Controller Block */
        std::shared_ptr<Model::model> modelPtr_ = 
            std::make_shared<Model::model>(
                model_settings.model_path,
                model_settings.friction,
                model_settings.friction_eps,
                model_settings.damping,
                model_settings.damping_eps,
                model_settings.transmissionInertia,
                model_settings.transmissionInertia_eps,
                model_settings.offset,
                model_settings.mass_eps,
                model_settings.com_eps,
                model_settings.inertia_eps);

        if (controller_type == "PID") 
        {
            auto ctrl_block = Dynamics::PIDControlBlock::make_shared(
                "Controller", modelPtr_);

            ctrl_block->setParameters(
                pid_params.Kp, 
                pid_params.Ki, 
                pid_params.Kd);

            sys.add_block(ctrl_block);
        }
        else if (controller_type == "GRAV_PID") 
        {
            auto ctrl_block = Dynamics::GravityCompensationPIDControlBlock::make_shared(
                "Controller", modelPtr_);

            ctrl_block->setParameters(
                pid_params.Kp, 
                pid_params.Ki, 
                pid_params.Kd);

            sys.add_block(ctrl_block);
        }
        else if (controller_type == "ARMOUR")
        {
            auto ctrl_block = Dynamics::ARMOURControlBlock::make_shared(
                "Controller", modelPtr_);

            ctrl_block->setParameters(
                armour_params.Kr, 
                armour_params.alpha,
                armour_params.V_max,
                armour_params.r_norm_threshold);

            sys.add_block(ctrl_block);
        }
        // else if (controller_type == "ALTHOFF")
        // {
        //     auto ctrl_block = Dynamics::PassivityControlBlock::make_shared(
        //         "Controller", modelPtr_);

        //     ctrl_block->setParameters(
        //         altoff_params.kappa_p, 
        //         altoff_params.kappa_i,
        //         altoff_params.phi_p, 
        //         altoff_params.phi_i,
        //         altoff_params.Kr, 
        //         altoff_params.max_error);

        //     sys.add_block(ctrl_block);
        // }
        else 
        {
            std::cerr << controller_type << std::endl;
            throw std::invalid_argument("Invalid controller type");
        }

        // retrieve controller block from system
        auto ctrl_block = sys.get_block("Controller");

        /** Kortex Block */
        auto kortex_block =
            Kortex::KortexBlock::make_shared("Kortex Robot", kortex_settings);

        // directly connect joint info to controller
        sys.connect_blocks(kortex_block, ctrl_block);

        /** Trajectory Block */
        auto cvt_traj =
            [](const customized_msgs::msg::TrajectoryMsg &msg) -> Trajectory {
            Trajectory out;
            std::copy(msg.traj_data.begin(), msg.traj_data.end(), out.traj_data);
            out.start_time = msg.start_time;
            out.trajectory_duration = msg.trajectory_duration;
            out.duration = msg.duration;
            out.dof = msg.dof;
            out.trajectory_type = msg.trajectory_type;
            out.is_gripper_open = msg.is_gripper_open;
            out.reset = msg.reset;
            return out;
        };
        auto ros_traj_block = System::SubscriberBlock<
            customized_msgs::msg::TrajectoryMsg, 
            Trajectory>::make_shared(
                "Trajectory",
                ros_traj_topic,
                ros_node, cvt_traj);
        sys.connect_blocks(ros_traj_block, ctrl_block);

        /** Joint info publisher */
        std::string joint_info_name = "joint_info";
        if (ros_node->get_parameter("dual_arm").as_bool()) {
            joint_info_name += "_" + std::to_string(robot_id);
        }

        auto cvt_measure =
            [](const msgs::Measurement &msg) -> KortexMeasurements {
            KortexMeasurements res;
            res.frame_id = msg.frame_id;

            std::time_t seconds = std::chrono::system_clock::to_time_t(msg.stamp);
            auto duration = msg.stamp.time_since_epoch();
            auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count() % 1000000000;
            res.stamp.sec = seconds;
            res.stamp.nanosec = nanoseconds;

            // convert from Eigen::VectorXd to std::array<float, 7>
            for (size_t i = 0; i < msg.pos.size(); i++) {
                res.pos[i] = msg.pos(i);
                res.vel[i] = msg.vel(i);
                res.torque[i] = msg.torque(i);
            }

            return res;
        };

        auto joint_pub_block = System::
            PublisherBlock<msgs::Measurement, KortexMeasurements>::make_shared(
                "State Publisher", joint_info_name, ros_node, cvt_measure);

        /// final block connections
        sys.connect_blocks(ctrl_block, kortex_block)
           .connect_blocks(kortex_block, joint_pub_block)
           .set_verbose()
           .set_timeout(0);

        /// move robot to initial position
        if (has_init_pos)
            kortex_block->MoveJointPos(
                std::vector<float>{init_pos.begin(), init_pos.end()});

        // if (ros_node->get_parameter("is_gripper_open").as_bool()) {
        //     printf("Opening gripper\n");
        //     // kortex_block->ResetGripper();
        // }
        // else {
        //     printf("Closing gripper\n");
        //     kortex_block->CloseGripper();
        // }
        kortex_block->ResetGripper();
    }

  private:
    rclcpp::Node::SharedPtr ros_node;
    System::System sys;

    /// settings
    bool debug;
    // whether to listen from ros topic
    bool ros_traj_enb;
    std::string ros_traj_topic;

    // initial position of the robot
    bool has_init_pos;
    std::vector<double> init_pos;

    std::string controller_type;

    Kortex::KortexSettings kortex_settings;
    ModelSettings model_settings;
    PIDParams pid_params;
    ArmourParams armour_params;
    AltoffParams altoff_params;
    FilterSettings filter_settings;

    int robot_id = -1;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto wrapper = Wrapper("control_system");
    wrapper.run();
}
