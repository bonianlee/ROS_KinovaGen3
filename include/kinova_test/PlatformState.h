#ifndef _PLATFORMSTATE_H_
#define _PLATFORMSTATE_H_

#include "kinova_test/Matrix.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "nav_msgs/Odometry.h"

class PlatformState
{
public:
    PlatformState();
    Matrix<double> q_p;
    Matrix<double> dq_p;
    void updatePlatformData(const nav_msgs::Odometry &msg);
};

#endif