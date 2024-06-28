#ifndef _HUMANHANDPOS_H_
#define _HUMANHANDPOS_H_

#include "kinova_test/Matrix.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "kinova_test/HandPosMsg.h"

#define lee_X_MAPPING 2.5
#define lee_Z_MAPPING 1.5

enum class lee_ControlMode
{
    Platform,
    WholeBody
};

class HumanHandPos
{
public:
    HumanHandPos();
    Matrix<double> Xd;
    float triggerVal;
    lee_ControlMode current_mode;
    bool stop;
    void updateHumanHandData(const kinova_test::HandPosMsg &msg);
    void updateControlMode(const std_msgs::Bool &msg);
    void updateTriggerValue(const std_msgs::Float32 &msg);
    void updateStopState(const std_msgs::Bool &msg);

private:
    void manipulator_mapping();
};

#endif