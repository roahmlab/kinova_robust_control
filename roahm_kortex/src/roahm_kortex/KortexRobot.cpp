#include "roahm_kortex/KortexRobot.hpp"
#include "SessionManager.h"
#include <chrono>
#include <stdexcept>

namespace Roahm
{
namespace Kortex
{
KortexRobot::KortexRobot(const KortexSettings &settings)
    : logger(Logger("kortex_robot")), settings(settings),
      joint_torque_enb(settings.JOINT_ENB), gripper(nullptr), is_safe(false),
      is_init(false)

{
}

KortexRobot::~KortexRobot()
{
    if (is_connected)
    {
        session_manager->CloseSession();
        router->SetActivationStatus(false);
        transport->disconnect();

        session_manager_rt->CloseSession();
        router_rt->SetActivationStatus(false);
        transport_rt->disconnect();
    }

    delete base;
    delete session_manager;
    delete router;
    delete transport;

    delete base_cyclic;
    delete router_rt;
    delete transport_rt;
}

bool KortexRobot::Init()
{
    // setting up connection
    auto error_callback = [this](KApi::KError err) {
        logger.error("___callback error___\n" + err.toString());
    };
    transport = new KApi::TransportClientTcp();
    router = new KApi::RouterClient(transport, error_callback);
    transport_rt = new KApi::TransportClientUdp();
    router_rt = new KApi::RouterClient(transport_rt, error_callback);

    // try connection
    logger.info("connecting {}", settings.ip);
    if (!transport->connect(settings.ip, settings.PORT) ||
        !transport_rt->connect(settings.ip, settings.PORT_RT))
    {
        logger.critical("Failed to connect");
        return false;
    }

    try
    {
        // setting up session
        session_manager = new KApi::SessionManager(router);
        session_manager_rt = new KApi::SessionManager(router_rt);

        auto session_info = KApi::Session::CreateSessionInfo();
        session_info.set_username(settings.uname);
        session_info.set_password(settings.passwd);
        session_info.set_session_inactivity_timeout(60000);
        session_info.set_connection_inactivity_timeout(2000);

        session_manager->CreateSession(session_info);
        session_manager_rt->CreateSession(session_info);

        this->is_connected = true;

        base = new KApi::Base::BaseClient(router);
        base_cyclic = new KApi::BaseCyclic::BaseCyclicClient(router_rt);

        actuator_config =
            new KApi::ActuatorConfig::ActuatorConfigClient(router);
        // initialized
        is_init = true;
        logger.info("Robot at {} initialized", settings.ip);
    }
    catch (KApi::KDetailedException &ex)
    {
        OnError(ex);
    }
    return is_init;
}

bool KortexRobot::MoveJointPos(const std::vector<float> &des)
{
    require_init();
    if (torque_control_mode)
        return false;
    SetServoingMode(KApi::Base::ServoingMode::SINGLE_LEVEL_SERVOING);

    auto action = KApi::Base::Action();
    action.set_name("Joint Pos");
    action.set_application_data("");

    auto reach_joint_angles = action.mutable_reach_joint_angles();
    auto joint_angles = reach_joint_angles->mutable_joint_angles();

    auto actuator_count = base->GetActuatorCount();
    assert(des.size() == actuator_count.count());

    std::vector<float> des_deg(des.size());
    std::transform(des.begin(), des.end(), des_deg.begin(),
                   [](const float &r) { return 180.0 / M_PI * r; });
    std::copy(std::begin(des_deg), std::end(des_deg),
              std::ostream_iterator<float>(std::cout, ", "));

    // Arm straight up
    for (size_t i = 0; i < actuator_count.count(); ++i)
    {
        auto joint_angle = joint_angles->add_joint_angles();
        joint_angle->set_joint_identifier(i);
        joint_angle->set_value(des_deg[i]);
    }

    // Connect to notification action topic
    // (Promise alternative)
    // See cartesian examples for Reference alternative
    std::promise<KApi::Base::ActionEvent> finish_promise;
    auto finish_future = finish_promise.get_future();
    auto promise_notification_handle = base->OnNotificationActionTopic(
        create_event_listener_by_promise(finish_promise),
        KApi::Common::NotificationOptions());

    logger.info("Executing action");
    base->ExecuteAction(action);

    logger.info("Waiting for movement to finish ...");

    // Wait for future value from promise
    // (Promise alternative)
    // See cartesian examples for Reference alternative
    const auto status =
        finish_future.wait_for(settings.TIMEOUT_PROMISE_DURATION);
    base->Unsubscribe(promise_notification_handle);

    if (status != std::future_status::ready)
    {
        logger.error("Timeout on action notification wait");
        return false;
    }

    logger.info("Movement finished");
    return true;
}

bool KortexRobot::MovePredefined(const std::string &name)
{
    require_init();
    if (torque_control_mode)
        return false;
    // Make sure the arm is in Single Level Servoing before executing an
    // Action
    SetServoingMode(KApi::Base::ServoingMode::SINGLE_LEVEL_SERVOING);

    // try clearing faults
    ClearFaults();
    // Move arm to ready position
    logger.info("Moving the arm to position: {}", name);
    auto action_type = KApi::Base::RequestedActionType();
    action_type.set_action_type(KApi::Base::REACH_JOINT_ANGLES);
    auto action_list = base->ReadAllActions(action_type);
    auto action_handle = KApi::Base::ActionHandle();
    action_handle.set_identifier(0);
    for (auto action : action_list.action_list())
    {
        if (action.name() == name)
        {
            action_handle = action.handle();
        }
    }

    if (action_handle.identifier() == 0)
    {
        logger.error("Can't reach {}, exiting", name);
        return false;
    }
    else
    {
        // Connect to notification action topic
        std::promise<KApi::Base::ActionEvent> finish_promise;
        auto finish_future = finish_promise.get_future();
        auto promise_notification_handle = base->OnNotificationActionTopic(
            create_event_listener_by_promise(finish_promise),
            KApi::Common::NotificationOptions());

        // Execute action
        base->ExecuteActionFromReference(action_handle);

        // Wait for future value from promise
        const auto status =
            finish_future.wait_for(settings.TIMEOUT_PROMISE_DURATION);
        base->Unsubscribe(promise_notification_handle);

        if (status != std::future_status::ready)
        {
            logger.error("Timeout on action notification wait");
            return false;
        }
        return true;
    }
    logger.info("Movement finished");
}

bool KortexRobot::MoveCartesian(const std::vector<float> &des)
{
    if (torque_control_mode)
        return false;
    
    auto action = KApi::Base::Action();
    action.set_name("Cartesian action movement");
    action.set_application_data("");

    auto constrained_pose = action.mutable_reach_pose();
    auto pose = constrained_pose->mutable_target_pose();
    auto speed = constrained_pose->mutable_constraint()->mutable_speed();

    assert(des.size() == 6); // (x, y, z, theta_x, theta_y, theta_z)

    speed->set_translation(0.3);  // m/s
    speed->set_orientation(3.5);  // deg/s

    pose->set_x(des[0]);
    pose->set_y(des[1]);
    pose->set_z(des[2]);
    pose->set_theta_x(des[3]);
    pose->set_theta_y(des[4]);
    pose->set_theta_z(des[5]);

    std::promise<KApi::Base::ActionEvent> finish_promise;
    auto finish_future = finish_promise.get_future();
    auto promise_notification_handle = base->OnNotificationActionTopic(
        create_event_listener_by_promise(finish_promise),
        KApi::Common::NotificationOptions()
    );

    logger.info("Executing cartesian action");
    base->ExecuteAction(action);
    logger.info("Waiting for movement to finish...");

    const auto status = 
        finish_future.wait_for(settings.TIMEOUT_PROMISE_DURATION);
    base->Unsubscribe(promise_notification_handle);

    if (status != std::future_status::ready) {
        logger.error("Timeout on action notification wait");
        return false;
    }

    return true;    
}

bool KortexRobot::CloseGripper()
{
    KApi::Base::GripperCommand gripper_command;
    gripper_command.set_mode(KApi::Base::GRIPPER_SPEED);
    auto finger = gripper_command.mutable_gripper()->add_finger();
    finger->set_finger_identifier(0);
    finger->set_value(-0.1f);
    base->SendGripperCommand(gripper_command);

    KApi::Base::Gripper gripper_feedback;
    KApi::Base::GripperRequest gripper_request;
    gripper_request.set_mode(KApi::Base::GRIPPER_SPEED);
    bool grasp_completed = false;

    // avoid false reporting of zero speed
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    while (!grasp_completed)
    {
        float speed = 0.0f;
        gripper_feedback = base->GetMeasuredGripperMovement(gripper_request);
        if (gripper_feedback.finger_size())
            speed = gripper_feedback.finger(0).value();
        if (speed == 0.0f)
            grasp_completed = true;
    }
    return grasp_completed;
}

bool KortexRobot::ResetGripper()
{
    KApi::Base::GripperCommand gripper_command;
    gripper_command.set_mode(KApi::Base::GRIPPER_POSITION);
    auto finger = gripper_command.mutable_gripper()->add_finger();
    finger->set_finger_identifier(0);
    finger->set_value(0.0f);
    base->SendGripperCommand(gripper_command);

    KApi::Base::Gripper gripper_feedback;
    KApi::Base::GripperRequest gripper_request;
    gripper_request.set_mode(KApi::Base::GRIPPER_POSITION);
    bool motion_completed = false;

    while (!motion_completed)
    {
        float position = 0.0f;
        gripper_feedback = base->GetMeasuredGripperMovement(gripper_request);
        if (gripper_feedback.finger_size())
        {
            position = gripper_feedback.finger(0).value();
        }

        if (position < 0.01f)
            motion_completed = true;
    }

    return motion_completed;
}

void KortexRobot::SetAllPositionControlMode()
{
    require_init();
    // If already in position control mode, skip
    if (!torque_control_mode)
        return;
    auto control_mode_message = KApi::ActuatorConfig::ControlModeInformation();
    control_mode_message.set_control_mode(
        KApi::ActuatorConfig::ControlMode::POSITION);
    for (uint i = 0; i < JOINT_NUM; i++)
    {
        actuator_config->SetControlMode(control_mode_message, i + 1);
    }

    torque_control_mode = false;
    logger.warn("All joints reset to position control mode");
}

void KortexRobot::lowlevel_control(
    bool use_position,
    std::function<void(const Measurement &)> send_mearsurement,
    std::function<std::shared_ptr<const ControlMsg>(void)> get_control)
{
    require_init();

    // previous time
    auto prev = std::chrono::high_resolution_clock::now();

    // Clearing faults
    ClearFaults();
    KApi::BaseCyclic::Command base_command;
    base_command.set_frame_id(settings.latency_error);
    KApi::BaseCyclic::Feedback base_feedback;

    // status arrays
    Measurement joint_info(JOINT_NUM);
    joint_info.frame_id = base_command.frame_id();
    // helper function for setting
    const auto set_joint_info = [&joint_info, &base_feedback]() {
        joint_info.frame_id++;
        joint_info.stamp = std::chrono::system_clock::now();
        for (std::size_t i = 0; i < JOINT_NUM; i++)
        {
            joint_info.pos[i] = Utils::wrapToPi(Utils::deg2rad(
                base_feedback.actuators(i).position()));
            joint_info.vel[i] = Utils::deg2rad(
                base_feedback.actuators(i).velocity());
            joint_info.torque[i] = 
                base_feedback.actuators(i).torque();
        }
    };

    uint32_t timer_count = 0;
    try
    {
        SetServoingMode(KApi::Base::ServoingMode::LOW_LEVEL_SERVOING);

        base_feedback = base_cyclic->RefreshFeedback();

        set_joint_info();
        send_mearsurement(joint_info);

        // Initialization
        for (uint i = 0; i < JOINT_NUM; i++)
        {
            // Save the current actuator position, to avoid a following
            // error
            base_command.add_actuators()->set_position(
                base_feedback.actuators(i).position());
        }

        // init gripper if used
        if (settings.use_gripper)
        {
            // get initial position of the gripper
            float gripper_initial_position = base_feedback.interconnect()
                                                 .gripper_feedback()
                                                 .motor()[0]
                                                 .position();
            gripper = std::make_unique<Gripper>(base_command,
                                                gripper_initial_position);
        }

        // Send a first frame
        base_feedback = base_cyclic->Refresh(base_command);

        // Real-time loop
        while (true)
        {
            auto now = std::chrono::high_resolution_clock::now();
            // // NOTE: ensure interval > 1000ms
            // if (std::chrono::duration_cast<std::chrono::microseconds>(now -
            //                                                           prev)
            //         .count() < 1000)
            // {
            //     continue;
            // }

            // record execution ts of this loop
            prev = now;

            if (need_lowlevel_init)
            {
                /* init_lowlevel_control(base_command, base_feedback); */
                servoing_mode.set_servoing_mode(
                    KApi::Base::ServoingMode::LOW_LEVEL_SERVOING);
                base->SetServoingMode(servoing_mode);
                base_feedback = base_cyclic->RefreshFeedback();
                base_command.clear_actuators();
                for (uint i = 0; i < JOINT_NUM; i++)
                {
                    // Save the current actuator position, to avoid a
                    // following error
                    base_command.add_actuators()->set_position(
                        base_feedback.actuators(i).position());
                }
                need_lowlevel_init = false;
            }

            /// control_info_ptr and ref have same life cycle
            auto control_info_ptr = get_control();
            ControlMsg control_info = *control_info_ptr;

            // gripper
            if (settings.use_gripper)
                gripper->update(control_info.is_gripper_open, base_feedback);

            // NOTE: get torque using message callback
            // Joint torque limits must not be exceeded
            if (!use_position && !lowlevel_safety_check(control_info))
                break;

            const int32_t base_frame_id = base_command.frame_id();
            const int32_t control_frame_id = control_info.frame_id;
            const int32_t diff = base_frame_id - control_frame_id;

            send_mearsurement(joint_info);

            // Check if control command is valid
            if (control_frame_id == 0 || control_info.torque.norm() == 0)
            {
                // logger.warn("No valid control command received yet");
                continue;
            }

            /** LATENCY CHECK */
            if (diff > settings.latency_error)
            {
                // Safety precaution. Brake robot.
                if (get_safe())
                {
                    logger.error(
                        "Frame id difference exceeds error thershold {} "
                        "base: {} "
                        "ctrl: {} "
                        "Robot is unsafe.",
                        settings.latency_error, 
                        base_frame_id,
                        control_frame_id);
                    set_unsafe();
                }
            }
            else if (diff > settings.latency_warn)
            {
                logger.warn("frame id doesn't match: {} and {}",
                            base_command.frame_id(), control_info.frame_id);
            }
            else
            {
                if (!get_safe())
                {
                    logger.info(
                        "Packet difference is back within limits, setting "
                        "robot as safe");
                    set_safe();
                }
            }

            /** ACTIONS */
            if (!get_safe())
            {
                SetAllPositionControlMode();
                base_feedback = base_cyclic->RefreshFeedback();
            }
            // torque control mode
            else if (!use_position)
            {
                // set torque control Mode if needed
                SetTorqueControlMode();

                _set_torques(base_command, base_feedback, control_info);

                apply_command_and_get_feedback(base_command, base_feedback);
            }
            // position control mode
            else
            {
                SetAllPositionControlMode();
                for (uint i = 0; i < JOINT_NUM; i++)
                {
                    // apply position
                    if (joint_torque_enb[i])
                    {
                        base_command.mutable_actuators(i)->set_position(
                            control_info.torque[i]);
                    }
                }
                apply_command_and_get_feedback(base_command, base_feedback);
            }

            // Incrementing identifier ensures actuators can reject out of
            // time frames
            base_command.set_frame_id(base_command.frame_id() + 1);

            for (uint idx = 0; idx < JOINT_NUM; idx++)
            {
                base_command.mutable_actuators(idx)->set_command_id(
                    base_command.frame_id());
            }

            /** SEND MEASUREMENT */
            set_joint_info();
            // timer stuff
            timer_count++;
        }

        logger.info("Torque control completed");

        // Set actuator back in position
        SetAllPositionControlMode();

        logger.info("Torque control clean exit");
    }
    catch (Utils::thread_stop &sig_stop)
    {
        // Set actuator back in position
        SetAllPositionControlMode();

        logger.info("Torque control sig stop");
        throw sig_stop;
    }
    catch (KApi::KDetailedException &ex)
    {
        OnError(ex);
    }
    catch (std::runtime_error &ex2)
    {
        logger.error(ex2.what());
    }

    // Set the servoing mode back to Single Level
    SetServoingMode(KApi::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
}

bool KortexRobot::EnableTorqueControl(const std::bitset<JOINT_NUM> &joint_enb)
{
    if (!available())
    {
        logger.error("Robot is currently in torque control mode. Cannot "
                     "change enabled joints.");
        return false;
    }
    logger.info("Torque Control Enabled for joints: {}", joint_enb.to_string());
    joint_torque_enb = joint_enb;
    return true;
}

void KortexRobot::ClearFaults()
{
    try
    {
        base->ClearFaults();
        set_safe();
    }
    catch (...)
    {
        logger.critical(
            "Unable to clear robot faults, try restart the robot to "
            "clear faults");
        exit(-1);
    }
}

bool KortexRobot::lowlevel_safety_check(ControlMsg &control_info)
{
    // Skip if robot is already unsafe
    if (!get_safe())
        return true;
    for (uint8_t i = 0; i < JOINT_NUM; ++i)
    {
        if (abs(control_info.torque[i]) >= settings.TORQUE_LIMITS[i])
        {
            logger.warn("Joint Torque Limit Exceeded for Joint {}, [{}, {}]", 
                i, control_info.torque[i], settings.TORQUE_LIMITS[i]);
            if (control_info.torque[i] > 0)
                control_info.torque[i] = 0.95 * settings.TORQUE_LIMITS[i];
            else
                control_info.torque[i] = -0.95 * settings.TORQUE_LIMITS[i];
            // set_unsafe();
            // base->ApplyEmergencyStop();
            // return false;
        }
    }
    return true;
}

void KortexRobot::SetServoingMode(KApi::Base::ServoingMode mode)
{
    if (servoing_mode.servoing_mode() == mode)
        return;
    if (mode != KApi::Base::ServoingMode::LOW_LEVEL_SERVOING)
        need_lowlevel_init = true;
    else
        need_lowlevel_init = false;
    servoing_mode.set_servoing_mode(mode);
    base->SetServoingMode(servoing_mode);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

void KortexRobot::SetTorqueControlMode()
{
    // If already in torque control mode, skip
    if (torque_control_mode)
        return;
    auto control_mode_message = KApi::ActuatorConfig::ControlModeInformation();
    control_mode_message.set_control_mode(
        KApi::ActuatorConfig::ControlMode::TORQUE);

    for (uint i = 0; i < JOINT_NUM; i++)
    {
        if (joint_torque_enb[i])
            actuator_config->SetControlMode(control_mode_message, i + 1);
    }
    torque_control_mode = true;
    logger.info("Enabled torque control mode");
}

void KortexRobot::OnError(KApi::KDetailedException &ex)
{
    auto error_info = ex.getErrorInfo().getError();
    logger.error("KDetailedoption detected what: {}", ex.what());

    logger.error("KError error_code: {}", error_info.error_code());

    logger.error("KError sub_code: {}", error_info.error_sub_code());
    logger.error("KError sub_string: {}", error_info.error_sub_string());

    // Error codes by themselves are not very verbose if you don't see their
    // corresponding enum value You can use google::protobuf helpers to get
    // the string enum element for every error code and sub-code
    logger.error(
        "Error code string equivalent: {}",
        KApi::ErrorCodes_Name(KApi::ErrorCodes(error_info.error_code())));
    logger.error("Error sub-code string equivalent: {}",
                 KApi::SubErrorCodes_Name(
                     KApi::SubErrorCodes(error_info.error_sub_code())));
    std::this_thread::sleep_for(std::chrono::seconds(10));
}

std::function<void(KApi::Base::ActionNotification)> KortexRobot::
    create_event_listener_by_promise(
        std::promise<KApi::Base::ActionEvent> &finish_promise)
{
    return [&finish_promise](KApi::Base::ActionNotification notification) {
        const auto action_event = notification.action_event();
        switch (action_event)
        {
        case KApi::Base::ActionEvent::ACTION_END:
        case KApi::Base::ActionEvent::ACTION_ABORT:
            finish_promise.set_value(action_event);
            break;
        default:
            break;
        }
    };
}

void KortexRobot::_set_torques(KApi::BaseCyclic::Command &base_command,
                               const KApi::BaseCyclic::Feedback &base_feedback,
                               const ControlMsg &control_info)
{
    for (uint i = 0; i < JOINT_NUM; i++)
    {
        // apply torque
        if (joint_torque_enb[i])
        {
            // Position command to first actuator is set to measured
            // one to avoid following error to trigger Bonus: When
            // doing this instead of disabling the following error,
            // if communication is lost and first
            //        actuator continues to move under torque
            //        command, resulting position error with command
            //        will trigger a following error and switch back
            //        the actuator in position command to hold its
            //        position
            base_command.mutable_actuators(i)->set_position(
                base_feedback.actuators(i).position());
            base_command.mutable_actuators(i)->set_torque_joint(
                control_info.torque[i]);
        }
    }
}

void KortexRobot::apply_command_and_get_feedback(
    const KApi::BaseCyclic::Command &base_command,
    KApi::BaseCyclic::Feedback &base_feedback)
{
    try
    {
        // For some reason, base_cyclic->RefreshCommand() did not work and
        // threw an unsupported protocol error. This would have been ideal
        // to consolidate the code
        base_feedback = base_cyclic->Refresh(base_command);
    }
    catch (KApi::KDetailedException &ex)
    {
        logger.error("Apply Command Error");
        OnError(ex);
    }
    catch (std::runtime_error &ex2)
    {
        logger.error("Runtime Error: {}", ex2.what());
    }
    catch (...)
    {
        logger.error("Unknown error");
    }
}
} // namespace Kortex
} // namespace Roahm
