#ifndef _HUMANSTATE_H_
#define _HUMANSTATE_H_

#include "xsens_mtw_driver/xsens2kinova.h"
#include "kinova_test/Matrix.h"
#include "std_msgs/Bool.h"

enum class ControlMode
{
    Platform,
    Manipulator
};

class HumanState
{
public:
    HumanState();
    Matrix<double> Xd;
    Matrix<double> dXd;
    double finger_pitch;
    ControlMode current_mode;
    void updateHumanData(const xsens_mtw_driver::xsens2kinova &msg);
    void updateControlMode(const std_msgs::Bool &msg);

private:
    void manipulator_mapping();
};

#endif