#include <iostream>
#include <cmath>
#include <string>
#include <vector>

// ROS-related
#include "ros/ros.h"
#include "kinova_test/kinovaMsg.h"
#include "std_msgs/Bool.h"
#include "kinova_test/agvMsg.h"

// Custom functions
#include "kinova_test/KinovaConfig.h"
#include "kinova_test/controller.h"
#include "kinova_test/conio.h"
#include "kinova_test/HumanState.h"
#include "kinova_test/PlatformState.h"
#include "kinova_test/HumanHandPos.h"

bool platform_control(ros::Publisher &platform_cmd_pub, HumanHandPos &oculusState)
{
    cout << "--------- Platform Mode ----------" << endl;
    ros::Rate loop_rate(10);
    bool return_status = true;
    int64_t t_start = GetTickUs(), now = GetTickUs();    // Unit: microsecond
    double exp_time = (double)(now - t_start) / 1000000; // Unit: second
    Matrix<double> Xd0 = oculusState.Xd;
    Matrix<double> Xd = oculusState.Xd - Xd0;
    while (ros::ok() && oculusState.current_mode == lee_ControlMode::Platform)
    {

        if (!(oculusState.stop) && !_kbhit())
        {
            // Calibrating user's initial position
            if (exp_time < 2)
            {
                Xd0 = oculusState.Xd;
                now = GetTickUs();
                exp_time = (double)(now - t_start) / 1000000;
            }
            Xd = oculusState.Xd - Xd0;
            // ROS Publisher
            geometry_msgs::Twist twist;
            humanPos2platformVel(Xd, twist);
            platform_cmd_pub.publish(twist);
            ros::spinOnce();
            loop_rate.sleep();
        }
        else if (oculusState.stop || (char)_getch() == 's')
        {
            return_status = false;
            break;
        }
    }
    return return_status;
}

bool torque_control(k_api::Base::BaseClient *base, k_api::BaseCyclic::BaseCyclicClient *base_cyclic, k_api::ActuatorConfig::ActuatorConfigClient *actuator_config, ros::Publisher &kinova_pub, ros::Publisher &platform_cmd_pub, ros::Publisher &platform_state_pub, HumanHandPos &oculusState, PlatformState &platformState)
{
    cout << "--------- WholeBody Mode ----------" << endl;
    kinova_test::kinovaMsg kinovaInfo; // ROS-related
    kinova_test::agvMsg agvInfo; // ROS-related
    double init_tau[7];
    bool return_status = true;

    // Get actuator count
    unsigned int actuator_count = base->GetActuatorCount().count();
    cout << "actuator_counts: " << actuator_count << endl;
    // Clearing faults
    try
    {
        base->ClearFaults();
    }
    catch (...)
    {
        cout << "Unable to clear robot faults" << endl;
        return false;
    }

    k_api::BaseCyclic::Feedback base_feedback;
    k_api::BaseCyclic::Command base_command;

    // Setup the gripper on the end-effector
    k_api::GripperCyclic::MotorCommand *gripper_motor_command;
    gripper_motor_command = base_command.mutable_interconnect()->mutable_gripper_command()->add_motor_cmd();

    vector<float> commands;
    auto servoing_mode = k_api::Base::ServoingModeInformation();

    cout << "Initializing the arm for torque control" << endl;
    try
    {
        // Set the base in low-level servoing mode
        servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
        base->SetServoingMode(servoing_mode);
        base_feedback = base_cyclic->RefreshFeedback();

        // Initialize each actuator to their current position
        for (unsigned int i = 0; i < actuator_count; i++)
        {
            commands.push_back(base_feedback.actuators(i).position());

            // Save the current actuator position, to avoid a following error
            base_command.add_actuators()->set_position(base_feedback.actuators(i).position());
        }

        // Send a first frame
        base_feedback = base_cyclic->Refresh(base_command);

        // Set first actuator in torque mode now that the command is equal to measure
        auto control_mode_message = k_api::ActuatorConfig::ControlModeInformation();
        control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::TORQUE);

        for (int i = 0; i < actuator_count; i++)
        {
            actuator_config->SetControlMode(control_mode_message, i + 1);
            init_tau[i] = -base_feedback.actuators(i).torque();
        }

        /* ------------ Initializing the parameters start ------------ */
        // Joint angle
        Matrix<double> position_curr(7, 1); // -pi~pi
        Matrix<int> round(7, 1);            // 圈數
        double position_curr_p;             // 移動平台當前旋轉方向 -pi~pi
        int round_p = 0;                    // 移動平台旋轉圈數
        Matrix<double> q(7, 1), dq(7, 1);   // -inf~inf
        Matrix<double> q_p(3, 1), dq_p(3, 1); // q_p = [x_p; y_p; phi_p] -inf~inf
        Matrix<double> q_pd(3, 1), dq_pd(3, 1), ddq_pd(3, 1);
        Matrix<double> q_w(10, 1), dq_w(10, 1);
        Matrix<double> cmd_vel_r(2, 1), cmd_vel(2, 1); // twist:states, cmd_vel:refference. [v; w] (mobile platform)
        for (int i = 0; i < 7; i++)
        {
            position_curr[i] = base_feedback.actuators(i).position() * DEG2RAD;
            if (position_curr[i] > M_PI)
                position_curr[i] = -(2 * M_PI - position_curr[i]);
        }
        q = position_curr;
        // read paltform's translation data
        for (unsigned int i = 0; i < 2; i++)
            q_p[i] = platformState.q_p[i];
        // read paltform's rotated data
        position_curr_p = platformState.q_p[2];
        q_p[2] = position_curr_p;

        for (unsigned int i = 0; i < 10; i++)
        {
            if (i < 3)
            {
                q_w[i] = q_p[i];
                dq_w[i] = dq_p[i];
            }
            else
            {
                q_w[i] = q[i - 3];
                dq_w[i] = dq[i - 3];
            }
        }
        // Forward kinematic
        double X_arr[6];
        Matrix<double> X(DOF, 1), dX(DOF, 1);
        exp_WholeBody_FK(q_p[0], q_p[1], position_curr_p, position_curr[0], position_curr[1], position_curr[2], position_curr[3], position_curr[4], position_curr[5], position_curr[6], X_arr);
        X.update_from_matlab(X_arr);
        Matrix<double> X0 = X;

        // Whole-Body Jacobian Matrix
        double Jw_arr[60]; //Jw_inv_arr[60]
        Matrix<double> Jw_6_10(6, 10);
        Matrix<double> Jw(DOF, 10), Jw_inv(10, DOF); //dJw_inv(10, DOF)
        exp_WholeBody_J(position_curr_p, position_curr[0], position_curr[1], position_curr[2], position_curr[3], position_curr[4], position_curr[5], position_curr[6], Jw_arr); // Odom q_p[2] 的範圍
        Jw_6_10.update_from_matlab(Jw_arr);
#if DOF == 3
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 10; j++)
                Jw(i, j) = Jw_6_10(i, j);
        Jw_inv = PINV(Jw);
#else
        Jw = Jw_6_10;
        Jw_inv = PINV(Jw_6_10);
#endif
        // G_w 矩陣
        double G_arr[7];
        Matrix<double> G(7, 1);
        Matrix<double> G_w(10, 1, MatrixType::General, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0});
        kinova_G_gripper(GRAVITY, position_curr[0], position_curr[1], position_curr[2], position_curr[3], position_curr[4], position_curr[5], position_curr[6], G_arr);
        G.update_from_matlab(G_arr);
        for (unsigned int i = 0; i < 7; i++)
            G_w(i + 3, 0) = G(i, 0);

        // Previous data
        Matrix<double> prev_q = q;
        Matrix<double> prev_q_p(3, 1); // -inf~inf
        Matrix<double> prev_Xd(6, 1);
        Matrix<double> prev_dXd(6, 1);

        // Desired trajectory
        Matrix<double> Xd0 = oculusState.Xd;
        Matrix<double> Xd = X0 + oculusState.Xd - Xd0;
        Matrix<double> dXd(6, 1);
        Matrix<double> ddXd(6, 1);
        Matrix<double> error = X - Xd;
        Matrix<double> derror = dX - dXd;
        Matrix<double> error_p = q_pd - q_p;
        Matrix<double> error_p_tf(3, 1);

        // Controller-related
        Matrix<double> psi(10, 1), subtasks(10, 1);
        std::vector<Matrix<double>> dW_hat, W_hat; // RBFNN-related
        for (int i = 0; i < DOF; i++)
        {
            dW_hat.emplace_back(Matrix<double>(NODE, 1));
            W_hat.emplace_back(Matrix<double>(NODE, 1));
        }
        Matrix<double> phi(NODE, 1);
        Matrix<double> sigma(DOF, 1), controller_tau(10, 1);
        double dGamma_lee, Gamma_lee;
        /* ------------ Initializing the parameters end ------------ */

        int64_t t_start = GetTickUs(), now = GetTickUs(), last = now; // Unit: microsecond
        double exp_time = (double)(now - t_start) / 1000000, dt;      // Unit: second

        // Real-time loop
        while (ros::ok() && oculusState.current_mode == lee_ControlMode::WholeBody)
        {
            if (!(oculusState.stop) && !_kbhit())
            {
                now = GetTickUs();
                kinovaInfo.time = exp_time;
                agvInfo.time = exp_time;
                if (now - last > 1000)
                {
                    // Calibrating user's initial position
                    if (exp_time < 3)
                        Xd0 = oculusState.Xd;

                    // Position command to first actuator is set to measured one to avoid following error to trigger
                    // Bonus: When doing this instead of disabling the following error, if communication is lost and first
                    //        actuator continues to move under torque command, resulting position error with command will
                    //        trigger a following error and switch back the actuator in position command to hold its position
                    for (int i = 0; i < actuator_count; i++)
                        base_command.mutable_actuators(i)->set_position(base_feedback.actuators(i).position());

                    // Parameters of controller
                    lee::WholeBody_joint_angle_limit_psi(position_curr, psi);
                    lee::WholeBody_manipulability_psi(position_curr, psi);
                    lee::WholeBody_joint_limit_subtask(position_curr, psi);
                    lee::WholeBody_joint_vel_limit_subtask(position_curr, psi);
                    lee::WholeBody_manipulator_config_psi(position_curr, psi);
                    lee::WholeBody_null_space_subtasks(Jw, Jw_inv, psi, dq_w, subtasks);
                    // RBFNN
                    lee::wholeBody_get_phi(q_p, q, dq_p, dq, dXd, ddXd, phi);
                    lee::wholeBody_get_dW_hat(phi, derror, Gamma_lee, dGamma_lee, W_hat, dW_hat);
                    for (unsigned i = 0; i < DOF; i ++)
                        sigma[i] = (W_hat.at(i).transpose() * phi)[0];
                    
                    // Controller
                    lee::wholeBody_controller(G_w, Jw, error, derror, sigma, subtasks, controller_tau);
                    wholeBody_gravity_compensation(position_curr, init_tau, controller_tau);

                    // Saturation for the torque
                    wholeBody_torque_saturation(controller_tau);
                    // Admittance interface
                    lee::Admittance_interface(dq_pd, ddq_pd, controller_tau);
                    // Platform controller
                    lee::reference_cmd_vel(dq_pd, q_p, round_p, cmd_vel_r);
                    lee::mobile_platform_error_tf(error_p, position_curr_p, error_p_tf);
                    lee::mobile_platform_control_rule(cmd_vel_r, error_p_tf, cmd_vel);
                    // Command torque for the KinovaGen3
                    for (int i = 3; i < 10; i++)
                    {
                        base_command.mutable_actuators(i - 3)->set_torque_joint(controller_tau[i]);
                        kinovaInfo.torque[i - 3] = controller_tau[i];
                    }
                    // Command velocity for the mobile platform
                    // ROS Publisher
                    geometry_msgs::Twist twist;
                    lee::admittance2platformVel(cmd_vel, twist);
                    platform_cmd_pub.publish(twist);
                    // Command input for the gripper
                    gripper_control(gripper_motor_command, oculusState.triggerVal);

                    // Read the joint angles
                    for (int i = 0; i < 7; i++)
                    {
                        kinovaInfo.jointPos[i] = base_feedback.actuators(i).position();
                        position_curr[i] = base_feedback.actuators(i).position() * DEG2RAD;
                        dq[i] = base_feedback.actuators(i).velocity() * DEG2RAD;
                        if (position_curr[i] > M_PI)
                            position_curr[i] = -(2 * M_PI - position_curr[i]);
                    }
                    q2inf(position_curr, prev_q, round, q);

                    // Forward kinematic
                    exp_WholeBody_FK(q_p[0], q_p[1], position_curr_p, position_curr[0], position_curr[1], position_curr[2], position_curr[3], position_curr[4], position_curr[5], position_curr[6], X_arr);
                    X.update_from_matlab(X_arr);
                    kinovaInfo.kinova_X = {X[0], X[1], X[2]};
                    kinovaInfo.kinova_Xd = {Xd[0], Xd[1], Xd[2]};
                    kinovaInfo.kinova_axis = {X[3], X[4], X[5]};
                    kinovaInfo.kinova_axisd = {Xd[3], Xd[4], Xd[5]};

                    // The status of the gripper
                    kinovaInfo.gripperPos = base_feedback.interconnect().gripper_feedback().motor(0).position();

                    // Jacobian matrix
                    exp_WholeBody_J(position_curr_p, position_curr[0], position_curr[1], position_curr[2], position_curr[3], position_curr[4], position_curr[5], position_curr[6], Jw_arr); // Odom q_p[2] 的範圍
                    Jw_6_10.update_from_matlab(Jw_arr);
#if DOF == 3
                    for (int i = 0; i < 3; i++)
                        for (int j = 0; j < 10; j++)
                            Jw(i, j) = Jw_6_10(i, j);
                    Jw_inv = PINV(Jw);
#else
                    Jw = Jw_6_10;
                    Jw_inv = PINV(Jw_6_10);
#endif

                    // Update time, differentiation, and integration
                    now = GetTickUs();
                    exp_time = (double)(now - t_start) / 1000000;
                    dt = (double)(now - last) / 1000000;
                    dXd = (Xd - prev_Xd) / dt;
                    ddXd = (dXd - prev_dXd) / dt;
                    // dq_p = (q_p - prev_q_p) / dt;
                    for (unsigned i = 0; i < DOF; i++)
                        W_hat.at(i) += dW_hat.at(i) * dt;
                    dq_pd += ddq_pd * dt;
                    q_pd += dq_pd * dt;
                    dX = Jw * dq_w;
                    prev_q = q;
                    prev_Xd = Xd;
                    prev_dXd = dXd;
                    prev_q_p = q_p;
                    last = now;

                    // Update the platform's data
                    // for translation data
                    for (unsigned int i = 0; i < 2; i++)
                        q_p[i] = platformState.q_p[i];
                    // for rotated data
                    position_curr_p = platformState.q_p[2];
                    q2inf_p(position_curr_p, prev_q_p, round_p, q_p);
                    lee::mobile_platform_kinematic(position_curr_p, twist, dq_p);

                    // Update the whole-body data
                    for (unsigned int i = 0; i < 10; i++)
                    {
                        if (i < 3)
                        {
                            q_w[i] = q_p[i];
                            dq_w[i] = dq_p[i];
                        }
                        else
                        {
                            q_w[i] = q[i - 3];
                            dq_w[i] = dq[i - 3];
                        }
                    }

                    Xd = X0 + oculusState.Xd - Xd0;
                    // Fixed the orientation to initialize pose, please uncomment the following code
                    // for (int i = 3; i < 6; i++)
                    //     Xd[i] = X0[i];

                    error = X - Xd;
                    derror = dX - dXd;
                    error_p = q_pd - q_p;

                    // Update to ROS
                    agvInfo.agv_axis = {q_p[0], q_p[1], q_p[2]};
                    agvInfo.agv_axisd = {q_pd[0], q_pd[1], q_pd[2]};
                    agvInfo.agv_axisVel = {dq_p[0], dq_p[1], dq_p[2]};
                    agvInfo.agv_axisdVel = {dq_pd[0], dq_pd[1], dq_pd[2]};
                    agvInfo.cmd_vel = {cmd_vel[0], cmd_vel[1]};
                    agvInfo.twist = {twist.linear.x, twist.angular.z};

                    // ROS publisher
                    kinova_pub.publish(kinovaInfo);
                    platform_state_pub.publish(agvInfo);
                    ros::spinOnce();
                    // Incrementing identifier ensures actuators can reject out of time frames
                    base_command.set_frame_id(base_command.frame_id() + 1);
                    if (base_command.frame_id() > 65535)
                        base_command.set_frame_id(0);

                    for (int idx = 0; idx < actuator_count; idx++)
                    {
                        base_command.mutable_actuators(idx)->set_command_id(base_command.frame_id());
                    }

                    try
                    {
                        base_feedback = base_cyclic->Refresh(base_command, 0);
                    }
                    catch (k_api::KDetailedException &ex)
                    {
                        cout << "Kortex exception: " << ex.what() << endl;

                        cout << "Error sub-code: " << k_api::SubErrorCodes_Name(k_api::SubErrorCodes((ex.getErrorInfo().getError().error_sub_code()))) << endl;
                    }
                    catch (runtime_error &ex2)
                    {
                        cout << "runtime error: " << ex2.what() << endl;
                    }
                    catch (...)
                    {
                        cout << "Unknown error." << endl;
                    }
                }
            }
            else if (oculusState.stop || (char)_getch() == 's')
            {
                return_status = false;
                break;
            }
        }

        cout << "Torque control completed" << endl;

        // Set first actuator back in position
        control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::POSITION);
        for (int i = 1; i <= actuator_count; i++)
            actuator_config->SetControlMode(control_mode_message, i);

        cout << "Torque control clean exit" << endl;
    }
    catch (k_api::KDetailedException &ex)
    {
        cout << "API error: " << ex.what() << endl;
        return_status = false;
    }
    catch (std::runtime_error &ex2)
    {
        cout << "Error: " << ex2.what() << endl;
        return_status = false;
    }

    // Set the servoing mode back to Single Level
    servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoing_mode);
    for (int i = 0; i < actuator_count; i++)
        base_command.mutable_actuators(i)->set_position(base_feedback.actuators(i).position());

    // Wait for a bit
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    return return_status;
}

int main(int argc, char **argv)
{
    // HumanState humanState;
    HumanHandPos oculusState;
    PlatformState platformState;
    // ROS-related
    ros::init(argc, argv, "mobileManipulatorDevice"); // rosnode的名稱
    ros::NodeHandle n;
    ros::Publisher platform_cmd_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);    // the name of the rostopic for publishing
    ros::Publisher platform_state_pub = n.advertise<kinova_test::agvMsg>("agv_state", 1);
    ros::Publisher kinova_pub = n.advertise<kinova_test::kinovaMsg>("kinovaInfo", 5); // the name of the rostopic for publishing
    // ros::Subscriber state_sub = n.subscribe("xsens2kinova", 1, &HumanState::updateHumanData, &humanState);
    ros::Subscriber mode_sub = n.subscribe("controlMode", 1, &HumanHandPos::updateControlMode, &oculusState);
    ros::Subscriber trigger_sub = n.subscribe("triggerVal", 1, &HumanHandPos::updateTriggerValue, &oculusState);
    ros::Subscriber stop_sub = n.subscribe("stop", 1, &HumanHandPos::updateStopState, &oculusState);
    ros::Subscriber odm_sub = n.subscribe("odom", 1, &PlatformState::updatePlatformData, &platformState);
    ros::Subscriber oculus_sub = n.subscribe("ref_pos", 1, &HumanHandPos::updateHumanHandData, &oculusState);

    auto parsed_args = ParseExampleArguments(argc, argv);

    // Create API objects
    auto error_callback = [](k_api::KError err)
    { cout << "_________ callback error _________" << err.toString(); };

    cout << "Creating transport objects" << endl;
    auto transport = new k_api::TransportClientTcp();
    auto router = new k_api::RouterClient(transport, error_callback);
    transport->connect(parsed_args.ip_address, PORT);

    cout << "Creating transport real time objects" << endl;
    auto transport_real_time = new k_api::TransportClientUdp();
    auto router_real_time = new k_api::RouterClient(transport_real_time, error_callback);
    transport_real_time->connect(parsed_args.ip_address, PORT_REAL_TIME);

    // Set session data connection information
    auto create_session_info = k_api::Session::CreateSessionInfo();
    create_session_info.set_username("admin");
    create_session_info.set_password("admin");
    create_session_info.set_session_inactivity_timeout(60000);   // (milliseconds)
    create_session_info.set_connection_inactivity_timeout(2000); // (milliseconds)

    // Session manager service wrapper
    cout << "Creating sessions for communication" << endl;
    auto session_manager = new k_api::SessionManager(router);
    session_manager->CreateSession(create_session_info);
    auto session_manager_real_time = new k_api::SessionManager(router_real_time);
    session_manager_real_time->CreateSession(create_session_info);
    cout << "Sessions created" << endl;

    // Create services
    auto base = new k_api::Base::BaseClient(router);
    auto base_cyclic = new k_api::BaseCyclic::BaseCyclicClient(router_real_time);
    auto actuator_config = new k_api::ActuatorConfig::ActuatorConfigClient(router);

    // Example core
    bool success = true;
    success &= example_move_to_home_position(base);
    while (success)
    {
        if (oculusState.current_mode == lee_ControlMode::Platform)
        {
            success &= move_to_home_position_with_ros(base, kinova_pub);
            success &= platform_control(platform_cmd_pub, oculusState);
        }
        else
        {
            // emergency_stop(platform_pub);
            success &= torque_control(base, base_cyclic, actuator_config, kinova_pub, platform_cmd_pub, platform_state_pub, oculusState, platformState);
        }
    }

    if (!success)
        cout << "There has been an unexpected error." << endl;
    emergency_stop(platform_cmd_pub);

    // Close API session
    session_manager->CloseSession();
    session_manager_real_time->CloseSession();

    // Deactivate the router and cleanly disconnect from the transport object
    router->SetActivationStatus(false);
    transport->disconnect();
    router_real_time->SetActivationStatus(false);
    transport_real_time->disconnect();

    // Destroy the API
    delete base;
    delete base_cyclic;
    delete actuator_config;
    delete session_manager;
    delete session_manager_real_time;
    delete router;
    delete router_real_time;
    delete transport;
    delete transport_real_time;

    return success ? 0 : 1;
}