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

bool platform_control(PlatformState &platformState, ros::Publisher &platform_cmd_pub, ros::Publisher &platform_state_pub)
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
    q_pd0[0] = -0.2 * cos(0.1 * 0);
    q_pd0[1] = -0.2 * sin(0.1 * 0);
    q_pd0[2] = -0;

    q_pd[0] = 0.2 * cos(0.1 * exp_time) + q_pd0[0];
    q_pd[1] = 0.2 * sin(0.1 * exp_time) + q_pd0[1];
    q_pd[2] = 0.5 * exp_time + q_pd0[2];

    dq_pd[0] = -0.02 * sin(0.1 * exp_time);
    dq_pd[1] = 0.02 * cos(0.1 * exp_time);
    dq_pd[2] = 0.5;

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
                lee::reference_cmd_vel(dq_pd, q_p, round_p, cmd_vel_r);
                lee::mobile_platform_error_tf(error_p, position_curr_p, error_p_tf);
                lee::mobile_platform_control_rule(cmd_vel_r, error_p_tf, cmd_vel);
                // Update time, differentiation, and integration
                now = GetTickUs();
                exp_time = (double)(now - t_start) / 1000000;
                dt = (double)(now - last) / 1000000;
                dq_p = (q_p - prev_q_p) / dt;
                prev_q_p = q_p;

                // Update the platform's data
                // for translation data
                for (unsigned int i = 0; i < 2; i++)
                    q_p[i] = platformState.q_p[i];
                // for rotated data
                position_curr_p = platformState.q_p[2];
                q2inf_p(position_curr_p, prev_q_p, round_p, q_p);

                if (exp_time < 50)
                {
                    q_pd[0] = 0.2 * cos(0.1 * exp_time) + q_pd0[0];
                    q_pd[1] = 0.2 * sin(0.1 * exp_time) + q_pd0[1];
                    q_pd[2] = 0.5 * exp_time + q_pd0[2];

                    dq_pd[0] = -0.02 * sin(0.1 * exp_time);
                    dq_pd[1] = 0.02 * cos(0.1 * exp_time);
                    dq_pd[2] = 0.5;
                }
                else
                {
                    q_pd[0] = 0.2 * cos(0.1 * 50) + q_pd0[0];
                    q_pd[1] = 0.2 * sin(0.1 * 50) + q_pd0[1];
                    q_pd[2] = 0.5 * 50 + q_pd0[2];

                    dq_pd[0] = 0;
                    dq_pd[1] = 0;
                    dq_pd[2] = 0;
                }

                error_p = q_pd - q_p;

                // Update to ROS
                agvInfo.agv_axis = {q_p[0], q_p[1], q_p[2]};
                agvInfo.agv_axisd = {q_pd[0], q_pd[1], q_pd[2]};
                agvInfo.agv_axisVel = {dq_p[0], dq_p[1], dq_p[2]};
                agvInfo.agv_axisdVel = {dq_pd[0], dq_pd[1], dq_pd[2]};

                // debug----------------------------
                cout << "--------------------------" << endl;
                cout << "error_p:";
                for (unsigned int i = 0; i < 3; i++)
                    cout << error_p[i] << ",";
                cout << endl;
                // debug----------------------------
                
                // ROS Publisher
                geometry_msgs::Twist twist;
                lee::admittance2platformVel(cmd_vel, twist);
                platform_cmd_pub.publish(twist);
                platform_state_pub.publish(agvInfo);

                // Update to ROS
                agvInfo.cmd_vel = {cmd_vel[0], cmd_vel[1]};
                agvInfo.twist = {twist.linear.x, twist.angular.z};

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
    // ROS-related
    ros::init(argc, argv, "mobileManipulatorDevice"); // rosnode的名稱
    ros::NodeHandle n;
    ros::Publisher platform_cmd_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);    // the name of the rostopic for publishing
    ros::Publisher platform_state_pub = n.advertise<kinova_test::agvMsg>("agv_state", 1);
    ros::Subscriber odm_sub = n.subscribe("odom", 1, &PlatformState::updatePlatformData, &platformState);

    // Example core
    bool success = true;
    while (success)
        success &= platform_control(platformState, platform_cmd_pub, platform_state_pub);

    if (!success)
        cout << "There has been an unexpected error." << endl;

    return success ? 0 : 1;
}