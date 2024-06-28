#include "kinova_test/HumanHandPos.h"
#include "kinova_test/controller.h"

HumanHandPos::HumanHandPos() : Xd(6, 1), current_mode(lee_ControlMode::Platform), triggerVal(0), stop(false) {}

void HumanHandPos::updateHumanHandData(const kinova_test::HandPosMsg &msg)
{
    for (unsigned int i = 0; i < 3; i++)
    {
        Xd[i] = msg.hand_pos[i];
        Xd[i + 3] = msg.hand_rot[i];
    }
    if (current_mode == lee_ControlMode::WholeBody)
        manipulator_mapping();
}

void HumanHandPos::updateTriggerValue(const std_msgs::Float32 &msg)
{
    triggerVal = msg.data;
}

void HumanHandPos::updateStopState(const std_msgs::Bool &msg)
{
    stop = msg.data;
}

void HumanHandPos::updateControlMode(const std_msgs::Bool &msg)
{
    current_mode = (lee_ControlMode)msg.data;
}

void HumanHandPos::manipulator_mapping()
{
    Xd[0] *= lee_X_MAPPING;
    Xd[2] *= lee_Z_MAPPING;
}