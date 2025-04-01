#pragma once
#include "BaseCyclic.pb.h"
#include "GripperCyclicMessage.pb.h"
#define _OS_UNIX
#include <array>
#include <bitset>
#include <chrono>
#include <cmath>
#include <iostream>
#include <string>
#include <unistd.h>
#include <vector>

#include <ActuatorConfigClientRpc.h>
#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <KDetailedException.h>
#include <NotificationHandler.h>
#include <RouterClient.h>
#include <SessionClientRpc.h>
#include <SessionManager.h>
#include <TransportClientTcp.h>
#include <TransportClientUdp.h>
#include <google/protobuf/util/json_util.h>

#include "Errors.pb.h"
#include "roahm_msgs/Messages.hpp"
#include "roahm_utils/Logger.hpp"
#include "roahm_utils/Utils.hpp"
#include <memory>

namespace Roahm
{
namespace KApi = Kinova::Api;
/**
 * @brief Settings that can be changed
 **/
namespace Kortex
{
using Roahm::msgs::ControlMsg, Roahm::msgs::Measurement;

/**
 * @brief structure contain settings
 **/
struct KortexSettings
{
    std::string ip = "192.168.1.10";
    std::string uname = "admin";
    std::string passwd = "admin";
    uint16_t PORT = 10000;
    uint16_t PORT_RT = 10001;
    int32_t latency_error = 50;
    int32_t latency_warn = 20;
    std::chrono::seconds TIMEOUT_PROMISE_DURATION{20};
    // std::vector<double> TORQUE_LIMITS{56.7, 56.7, 56.7, 56.7, 29.4, 29.4, 29.4};
    std::vector<double> TORQUE_LIMITS{85.0, 85.0, 85.0, 85.0, 40.0, 40.0, 40.0};
    std::bitset<7> JOINT_ENB{0b0000000};
    bool use_gripper = false; // whether to use gripper
};

/**
 * @brief wrapper for Kortex API to control and get information from the
 *        robot arm
 **/
class KortexRobot : private Utils::noncopyable
{
  protected:
    class Gripper;

  public:
    static constexpr std::size_t JOINT_NUM = 7;

  public:
    /**
     * @brief default constructor
     * @param settings: KortexSettings
     **/
    KortexRobot(const KortexSettings &settings);

    /**
     * @brief default destructor
     * @details free all the memories allocated
     */
    ~KortexRobot();

    /**
     * @brief initialize the robot
     * @NOTE: I passed all the Ctor with unique_ptr.get()
     * @return true on success
     **/
    bool Init();

    /** HIGH LEVEL APIs */
    /**
     * @brief Move to joint position
     *
     * @param des - Desired joint angles in radians
     *
     * @return Success or failure
     */
    bool MoveJointPos(const std::vector<float> &des);

    bool MovePredefined(const std::string &name);

    bool MoveCartesian(const std::vector<float> &des);

    bool CloseGripper();

    bool ResetGripper();

    /**
     * @brief move the robot arm home position
     * @return true on success
     **/
    inline bool MoveHome()
    {
        return MovePredefined("Home");
    };

    inline bool MoveZero()
    {
        return MovePredefined("Zero");
    }

    /**
     * @brief Reset all joints to position control mode.
     * Used at the end of torque control or on error to ensure safety
     */
    void SetAllPositionControlMode();

    /**
     * @brief begin lowlevel control loop
     * @param use_position: whether to use position
     * @param send_measurement: function handler to send measurement
     * @param get_control: get control from outside
     **/
    void lowlevel_control(
        bool use_position,
        std::function<void(const Measurement &)> send_mearsurement,
        std::function<std::shared_ptr<const ControlMsg>(void)> get_control);

    /**
     * @brief enable joint control on several joints
     * Will cause error if robot is already in torque control mode
     *
     * @param joint_enb: %array recording enable bits of the joints
     *
     * @return success
     **/
    bool EnableTorqueControl(const std::bitset<JOINT_NUM> &joint_enb);

    /**
     * @brief Check if robot is available for services or high level commands.
     * - Criteria: Robot should not be in torque control mode
     *
     * @return Availability
     */
    inline bool available()
    {
        return !torque_control_mode;
    }

    // protected functions

    /**
     * @brief clear faults in the robot arm, declare robot as safe to move
     * @details exit when fault can't be cleared
     **/
    void ClearFaults();

  protected:
    /**
     * @brief perform safety check on the control message as well as kinova
     * feedback. Sets is_safe variable
     * @param joint_info: Joint feedback from the robot
     *
     * @return true if safe else false
     **/
    bool lowlevel_safety_check(ControlMsg &control_info);

    /**
     * @brief set servoing mode
     * @param mode: servoing mode to set
     *   available options:
     *       UNSPECIFIED_SERVOING_MODE = 0
     *       SINGLE_LEVEL_SERVOING = 2
     *       LOW_LEVEL_SERVOING = 3
     *       BYPASS_SERVOING = 4
     **/
    void SetServoingMode(KApi::Base::ServoingMode mode);

    /**
     * @brief Set joints from joint_torque_enb to torque control mode
     */
    void SetTorqueControlMode();

    /**
     * @brief call back function when error happens
     **/
    void OnError(KApi::KDetailedException &ex);

    /**
     * @brief create callback function for event by promise
     * @return event listener as an std::function
     **/
    std::function<void(KApi::Base::ActionNotification)>
    create_event_listener_by_promise(
        std::promise<KApi::Base::ActionEvent> &finish_promise);

    inline bool get_safe() const
    {
        return is_safe;
    }

    inline void set_safe()
    {
        is_safe = true;
    }

    inline void set_unsafe()
    {
        is_safe = false;
    }

    /**
     * @brief set torque
     * @details meant to be used in torque control mode
     * @param base_command:
     * @param base_feedback:
     * @param control_info
     **/
    void _set_torques(KApi::BaseCyclic::Command &base_command,
                      const KApi::BaseCyclic::Feedback &base_feedback,
                      const ControlMsg &control_info);

    /**
     * @brief apply command
     * @param base_command
     * @param base_feedback
     **/
    void apply_command_and_get_feedback(
        const KApi::BaseCyclic::Command &base_command,
        KApi::BaseCyclic::Feedback &base_feedback);

    /**
     * @brief check and throw error if not initialized
     **/
    inline void require_init()
    {
        if (!is_init)
            throw std::runtime_error("KortexRobot not initialized!");
    }

    // protected variables
  protected:
    // logger wrapper
    Logger logger;
    const KortexSettings settings;
    bool is_connected;

    // TCP connections
    KApi::TransportClientTcp *transport{nullptr};
    KApi::RouterClient *router{nullptr};
    KApi::SessionManager *session_manager{nullptr};
    KApi::Base::BaseClient *base{nullptr};
    KApi::ActuatorConfig::ActuatorConfigClient *actuator_config{nullptr};

    // UDP (real time) connections
    KApi::TransportClientUdp *transport_rt{nullptr};
    KApi::RouterClient *router_rt{nullptr};
    KApi::SessionManager *session_manager_rt{nullptr};
    KApi::BaseCyclic::BaseCyclicClient *base_cyclic{nullptr};

    // status recorder
    KApi::Base::ServoingModeInformation servoing_mode;

    // used to fix joints in torque control
    std::bitset<JOINT_NUM> joint_torque_enb;

    bool torque_control_mode = false;
    bool need_lowlevel_init = true;

    std::unique_ptr<Gripper> gripper;

  private:
    // Safety.
    bool is_safe;
    bool is_init;

  protected:
    class Gripper
    {
      public:
        /**
         * @brief Ctor
         * @param base_command
         * @param init_pos: init position of the gripper
         * @param force: force of the gripper
         * @param speed: speed when opening and closing gripper, 0 - 100
         **/
        Gripper(KApi::BaseCyclic::Command &base_command, float init_pos,
                float force = 10., float speed = 100)
            : gripper_cmd(base_command.mutable_interconnect()
                              ->mutable_gripper_command()
                              ->add_motor_cmd()),
              gripper_state(GripperState::opening), gripper_speed(speed)
        {
            gripper_cmd->set_position(init_pos);
            gripper_cmd->set_velocity(0.0);
            gripper_cmd->set_force(force);
        };

        /**
         * @brief set gripper force
         * @param force: force to set
         **/
        void set_force(float force)
        {
            gripper_cmd->set_force(force);
        }

        /**
         * @brief update gripper status
         * @param should_gripper_open: desired gripper state
         * @param base_feedback: feedback from robot
         **/
        void update(bool should_gripper_open,
                    const KApi::BaseCyclic::Feedback &base_feedback)
        {
            // switch state if necessary
            float gripper_position = base_feedback.interconnect()
                                                 .gripper_feedback()
                                                 .motor()[0]
                                                 .position();
            
            // the threshold number is currently hardcoded
            if (gripper_position < 10.0)
            {
                gripper_state = GripperState::opened;
            }
            else if (gripper_position > 90.0)
            {
                gripper_state = GripperState::closed;
            }

            if (should_gripper_open && gripper_state != GripperState::opened)
            {
                gripper_state = GripperState::opening;
                gripper_cmd->set_position(0);
                gripper_cmd->set_velocity(gripper_speed);
            }
            else if (!should_gripper_open &&
                     gripper_state != GripperState::closed)
            {
                gripper_state = GripperState::closing;
                gripper_cmd->set_position(100);
                gripper_cmd->set_velocity(gripper_speed);
            }

            if (gripper_state == GripperState::opening || gripper_state == GripperState::closing)
            {
                float speed_cur = base_feedback.interconnect()
                                      .gripper_feedback()
                                      .motor()[0]
                                      .velocity();
                if (speed_cur < 10)
                {
                    stop_counter++;
                    if (stop_counter == max_stop_count)
                    {
                        float pos_cur = base_feedback.interconnect()
                                            .gripper_feedback()
                                            .motor()[0]
                                            .position();
                        gripper_cmd->set_position(pos_cur);
                        gripper_cmd->set_velocity(0);
                        if (gripper_state == GripperState::opening)
                        {
                            gripper_state = GripperState::opened;
                        }
                        else
                        {
                            gripper_state = GripperState::closed;
                        }
                        stop_counter = 0;
                    }
                }
                else
                {
                    stop_counter = 0;
                }
            }
        }

      private:
        /** @brief encode state of the gripper */
        enum class GripperState
        {
            opened,
            closed,
            opening,
            closing,
            unknown
        };

      private:
        std::unique_ptr<KApi::GripperCyclic::MotorCommand> gripper_cmd;
        GripperState gripper_state;
        float gripper_speed;
        uint32_t stop_counter = 0;
        const uint32_t max_stop_count = 200;
    };
};
} // namespace Kortex
} // namespace Roahm
