#include "kinova_test/PlatformState.h"
#include "kinova_test/controller.h"

PlatformState::PlatformState() : q_p(3, 1), dq_p(3, 1), twist(2, 1) {}

struct Quaternion
{
    double x, y, z, w;
};


void PlatformState::updatePlatformData(const nav_msgs::Odometry &msg)
{
    // position states
    Quaternion platformQuaternion;
    platformQuaternion.x = msg.pose.pose.orientation.x;
    platformQuaternion.y = msg.pose.pose.orientation.y;
    platformQuaternion.z = msg.pose.pose.orientation.z;
    platformQuaternion.w = msg.pose.pose.orientation.w;
    // platform's Euler angle:yaw (quaternion to Euler angle by x-y-z rotated order)
    double platform_yaw = atan2(2 * (platformQuaternion.w * platformQuaternion.z + platformQuaternion.x * platformQuaternion.y), 1 - 2 * (platformQuaternion.y * platformQuaternion.y + platformQuaternion.z * platformQuaternion.z));
    q_p[0] = msg.pose.pose.position.x;
    q_p[1] = msg.pose.pose.position.y;
    q_p[2] = platform_yaw;
    
    // velocity states
    dq_p[0] = msg.twist.twist.linear.x;
    dq_p[1] = msg.twist.twist.linear.y;
    dq_p[2] = msg.twist.twist.angular.z;

    // twist states
    twist[0] = msg.twist.twist.linear.x / cos(platform_yaw); // v
    twist[1] = msg.twist.twist.angular.z;
}