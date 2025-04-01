#include "rclcpp/rclcpp.hpp"
#include "roahm_kortex/KortexBlock.hpp"
#include "roahm_msgs/Messages.hpp"
#include "roahm_msgs/msg/kortex_measurements.hpp"
#include "roahm_msgs/msg/torque_control.hpp"
#include "roahm_msgs/srv/bool.hpp"
#include "roahm_msgs/srv/goto.hpp"
#include "roahm_msgs/srv/goto_name.hpp"
#include "roahm_msgs/srv/goto_cartesian.hpp"
#include "roahm_msgs/srv/gripper_action.hpp"
#include "roahm_system/HelperBlocks.hpp"
#include "std_msgs/msg/bool.hpp"
#include "roahm_system/ROSHelper.hpp"
#include "roahm_system/System.hpp"
#include <functional>
#include <rmw/types.h>
#include <roahm_system/BaseBlock.hpp>

using namespace Roahm;
using namespace std::placeholders;
using namespace roahm_msgs::msg;
using namespace roahm_msgs::srv;

const std::string NODE_NAME = "ros_kortex_system";

class RosSystemWrapper
{
  public:
    RosSystemWrapper()
        : node(std::make_shared<rclcpp::Node>(NODE_NAME)),
          kortex_block(nullptr), sys(System::System("ROSKortexSystem"))
    {
        node->declare_parameter<int>("arm_enb", 0b1111111);
        node->declare_parameter<std::string>("ip", "192.168.1.10");

        // subscriber
        auto cvt_torque = [](const TorqueControl &msg) -> msgs::ControlMsg {
            msgs::ControlMsg ctrl(7);
            ctrl.frame_id = msg.frame_id;

            std::chrono::seconds sec(msg.stamp.sec);
            std::chrono::nanoseconds nanosec(msg.stamp.nanosec);  
            ctrl.stamp = std::chrono::system_clock::time_point(sec + nanosec);
            
            // convert from std::array<float, 7> to Eigen::VectorXd
            for (size_t i = 0; i < msg.torque.size(); i++) {
                ctrl.torque(i) = msg.torque[i];
            }

            return ctrl;
        };
        auto sub_block = System::SubscriberBlock<
            TorqueControl, msgs::ControlMsg>::make_shared("control_input",
                                                          "dynamics", node,
                                                          cvt_torque);

        // kortex
        Kortex::KortexSettings settings;
        settings.JOINT_ENB = node->get_parameter("arm_enb").as_int();
        settings.ip = node->get_parameter("ip").as_string();
        kortex_block = Kortex::KortexBlock::make_shared("kortex", settings);

        // publisher
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
                res.pos[i] = msg.pos[i];
                res.vel[i] = msg.vel[i];
                res.torque[i] = msg.torque[i];
            }

            return res;
        };
        auto pub_block = System::
            PublisherBlock<msgs::Measurement, KortexMeasurements>::make_shared(
                "measurement publisher", "joint_info", node, cvt_measure);

        sys.connect_blocks(sub_block, kortex_block)
            .connect_blocks(kortex_block, pub_block)
            .set_verbose();
    }

    // run system
    void run()
    {
        auto goto_srv = node->create_service<roahm_msgs::srv::Goto>(
            NODE_NAME + "/goto",
            std::bind(&RosSystemWrapper::goto_pos, this, _1, _2));

        auto goto_name_srv = node->create_service<roahm_msgs::srv::GotoName>(
            NODE_NAME + "/goto_name",
            std::bind(&RosSystemWrapper::goto_predefined, this, _1, _2));
        
        auto goto_cartesian_srv = node->create_service<roahm_msgs::srv::GotoCartesian>(
            NODE_NAME + "/goto_cartesian",
            std::bind(&RosSystemWrapper::goto_cartesian, this, _1, _2));

        auto close_gripper_srv = node->create_service<roahm_msgs::srv::GripperAction>(
            NODE_NAME + "/close_gripper",
            std::bind(&RosSystemWrapper::close_gripper, this, _1, _2));

        auto reset_gripper_srv = node->create_service<roahm_msgs::srv::GripperAction>(
            NODE_NAME + "/reset_gripper",
            std::bind(&RosSystemWrapper::reset_gripper, this, _1, _2));

        auto run_srv = node->create_service<Bool>(
            NODE_NAME + "/run",
            std::bind(&RosSystemWrapper::run_system, this, _1, _2));

        sys.run();
        // rclcpp::spin(node);
    }

  private:
    void goto_pos(const std::shared_ptr<Goto_Request> request,
                  std::shared_ptr<Goto_Response> response)
    {
        std::vector<float> jnt(request->pos.begin(), request->pos.end());
        kortex_block->MoveJointPos(jnt);
        response->pos[0] = 0;
    }

    void goto_predefined(const std::shared_ptr<GotoName_Request> request,
                         std::shared_ptr<GotoName_Response> response)
    {
        kortex_block->MovePredefined(request->name);
        response->pos[0] = 0;
    }

    void goto_cartesian(const std::shared_ptr<GotoCartesian_Request> request,
                        std::shared_ptr<GotoCartesian_Response> response)
    {
        std::vector<float> pose(request->pose.begin(), request->pose.end());
        std_msgs::msg::Bool res;
        res.data = kortex_block->MoveCartesian(pose);
        response->success = res;
    }

    void close_gripper(const std::shared_ptr<GripperAction_Request> request,
                    std::shared_ptr<GripperAction_Response> response)
    {
        std_msgs::msg::Bool res;
        bool success = kortex_block->CloseGripper();
        response->success = res;
    }

    void reset_gripper(const std::shared_ptr<GripperAction_Request> request,
                    std::shared_ptr<GripperAction_Response> response)
    {
        std_msgs::msg::Bool res;
        res.data = kortex_block->ResetGripper();
        response->success = res;
    }

    void run_system([[maybe_unused]] const Bool::Request::SharedPtr request,
                    Bool::Response::SharedPtr response)
    {
        // sys.run();
        response->res = true;
    }

    rclcpp::Node::SharedPtr node;
    std::shared_ptr<Kortex::KortexBlock> kortex_block;
    System::System sys;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    RosSystemWrapper wrapper;
    wrapper.run();
}
