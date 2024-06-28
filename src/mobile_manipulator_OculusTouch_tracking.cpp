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

bool platform_control(PlatformState &platformState, ros::Publisher &platform_cmd_pub, ros::Publisher &platform_state_pub, HumanHandPos &oculusState)
{
    cout << "--------- Platform Mode ----------" << endl;
    kinova_test::agvMsg agvInfo; // ROS-related
    ros::Rate loop_rate(10);
    bool return_status = true;
    // clock
    int64_t t_start = GetTickUs(), now = GetTickUs(), last = now;    // Unit: microsecond
    double exp_time = (double)(now - t_start) / 1000000, dt; // Unit: second
    
    double position_curr_p; // 移動平台當前旋轉方向 -pi~pi
    int round_p = 0;            // 移動平台旋轉圈數
    Matrix<double> q_p(3, 1), dq_p(3, 1);   // q_p = [x_p; y_p; phi_p] all elements are -inf~inf
    Matrix<double> prev_q_p(3, 1);   // -inf~inf
    Matrix<double> cmd_vel_r(2, 1), cmd_vel(2, 1);

    // read translation data
    for (unsigned int i = 0; i < 2; i++)
        q_p[i] = platformState.q_p[i];
    // read rotated data
    position_curr_p = platformState.q_p[2];
    q_p[2] = position_curr_p;

    // reference trajectory and velocity
    Matrix<double> q_pd0(3, 1);
    Matrix<double> q_pd(3, 1), dq_pd(3, 1);
    
    q_pd0[0] = oculusState.Xd[0];
    q_pd0[1] = oculusState.Xd[1];
    q_pd0[2] = oculusState.Xd[6];

    q_pd[0] = oculusState.Xd[0] - q_pd0[0];
    q_pd[1] = oculusState.Xd[1] - q_pd0[1];
    q_pd[2] = oculusState.Xd[5] - q_pd0[5];

    Matrix<double> prev_q_pd(3, 1);

    Matrix<double> error_p = q_pd - q_p;
    Matrix<double> error_p_tf(3, 1);

    while (ros::ok())
    {
        if (!_kbhit())
        {
            now = GetTickUs();
            agvInfo.time = exp_time;
            if (now - last > 1000)
            {
                if (exp_time < 3)
                {
                    q_pd0[0] = oculusState.Xd[0];
                    q_pd0[1] = oculusState.Xd[1];
                    q_pd0[2] = oculusState.Xd[6];
                }

                lee::reference_cmd_vel(dq_pd, q_p, round_p, cmd_vel_r);
                lee::mobile_platform_error_tf(error_p, position_curr_p, error_p_tf);
                lee::mobile_platform_control_rule(cmd_vel_r, error_p_tf, cmd_vel);
                // Update time, differentiation, and integration
                now = GetTickUs();
                exp_time = (double)(now - t_start) / 1000000;
                dt = (double)(now - last) / 1000000;
                dq_pd = (q_pd - prev_q_pd) / dt;
                prev_q_p = q_p;
                prev_q_pd = q_pd;
                last = now;

                // Update the platform's data
                // for translation data
                for (unsigned int i = 0; i < 2; i++)
                    q_p[i] = platformState.q_p[i];
                // for rotated data
                position_curr_p = platformState.q_p[2];
                q2inf_p(position_curr_p, prev_q_p, round_p, q_p);

                q_pd[0] = oculusState.Xd[0] - q_pd0[0];
                q_pd[1] = oculusState.Xd[1] - q_pd0[1];
                q_pd[2] = oculusState.Xd[5] - q_pd0[5];

                error_p = q_pd - q_p;

                // ROS Publisher
                geometry_msgs::Twist twist;
                lee::admittance2platformVel(cmd_vel, twist);
                lee::mobile_platform_kinematic(position_curr_p, twist, dq_p);
                
                // Update to ROS
                agvInfo.agv_axis = {q_p[0], q_p[1], q_p[2]};
                agvInfo.agv_axisd = {q_pd[0], q_pd[1], q_pd[2]};
                agvInfo.agv_axisVel = {dq_p[0], dq_p[1], dq_p[2]};
                agvInfo.agv_axisdVel = {dq_pd[0], dq_pd[1], dq_pd[2]};
                agvInfo.cmd_vel = {cmd_vel[0], cmd_vel[1]};
                agvInfo.twist = {twist.linear.x, twist.angular.z};

                // debug----------------------------
                cout << "--------------------------" << endl;
                cout << "error_p:";
                for (unsigned int i = 0; i < 3; i++)
                    cout << error_p[i] << ",";
                cout << endl;
                cout << "error_p_tf:";
                for (unsigned int i = 0; i < 3; i++)
                    cout << error_p_tf[i] << ",";
                cout << endl;
                cout << "cmd_vel:";
                for (unsigned int i = 0; i < 2; i++)
                    cout << cmd_vel[i] << ",";
                cout << endl;
                cout << "twist:";
                cout << twist.linear.x << "," << twist.angular.z << ",";
                cout << endl;
                cout << "q_p:";
                for (unsigned int i = 0; i < 3; i++)
                    cout << q_p[i] << ",";
                cout << endl;
                // debug----------------------------
                

                platform_cmd_pub.publish(twist);
                platform_state_pub.publish(agvInfo);
                ros::spinOnce();
                loop_rate.sleep();
            }
        }
        else if ((char)_getch() == 's')
        {
            return_status = false;
            break;
        }
    }
    return return_status;
}

int main(int argc, char **argv)
{
    PlatformState platformState;
    HumanHandPos oculusState;
    // ROS-related
    ros::init(argc, argv, "mobileManipulatorDevice"); // rosnode的名稱
    ros::NodeHandle n;
    ros::Publisher platform_cmd_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);    // the name of the rostopic for publishing
    ros::Publisher platform_state_pub = n.advertise<kinova_test::agvMsg>("agv_state", 1);
    ros::Subscriber odm_sub = n.subscribe("odom", 1, &PlatformState::updatePlatformData, &platformState);
    ros::Subscriber oculus_sub = n.subscribe("ref_pos", 1, &HumanHandPos::updateHumanHandData, &oculusState);

    // Example core
    bool success = true;
    while (success)
        success &= platform_control(platformState, platform_cmd_pub, platform_state_pub, oculusState);

    if (!success)
        cout << "There has been an unexpected error." << endl;

    return success ? 0 : 1;
}