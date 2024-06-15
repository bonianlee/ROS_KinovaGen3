#include <iostream>
#include <cmath>
#include <string>
#include <vector>

// ROS-related
#include "ros/ros.h"
#include "kinova_test/kinovaMsg.h"
#include "std_msgs/Bool.h"

// Custom functions
#include "kinova_test/KinovaConfig.h"
#include "kinova_test/controller.h"
#include "kinova_test/conio.h"
#include "kinova_test/HumanState.h"
#include "kinova_test/PlatformState.h"

bool platform_control(PlatformState &platformState, ros::Publisher &platform_pub)
{
    cout << "--------- Platform Mode ----------" << endl;
    ros::Rate loop_rate(10);
    bool return_status = true;
    int64_t t_start = GetTickUs(), now = GetTickUs();    // Unit: microsecond
    double exp_time = (double)(now - t_start) / 1000000; // Unit: second
    double position_curr_p; // 移動平台當前旋轉方向 -pi~pi
    int round_p;            // 移動平台旋轉圈數
    Matrix<double> q_p(3, 1);   // q_p = [x_p; y_p; phi_p] all elements are -inf~inf
    Matrix<double> prev_q_p(3, 1), prev_dq_p(3, 1);   // -inf~inf
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

    Matrix<double> error_p = q_p - q_pd;

    while (ros::ok())
    {
        if (!_kbhit())
        {
            lee::reference_cmd_vel(dq_pd, q_p, round_p, cmd_vel_r);
            lee::mobile_platform_error_tf(q_pd, q_p, position_curr_p, error_p);
            lee::mobile_platform_control_rule(q_pd, error_p, cmd_vel);
            // Update time, differentiation, and integration
            prev_q_p = q_p;

            // Updat the platform's data
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

            error_p = q_p - q_pd;

            // ROS Publisher
            geometry_msgs::Twist twist;
            lee::admittance2platformVel(cmd_vel, twist);
            platform_pub.publish(twist);
            ros::spinOnce();
            loop_rate.sleep();
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
    ros::Publisher platform_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);    // the name of the rostopic for publishing
    ros::Subscriber odm_sub = n.subscribe("odom", 1, &PlatformState::updatePlatformData, &platformState);

    // Example core
    bool success = true;
    while (success)
        success &= platform_control(platformState, platform_pub);

    if (!success)
        cout << "There has been an unexpected error." << endl;

    return success ? 0 : 1;
}