#include "kinova_test/HumanHandPos.h"
#include "kinova_test/controller.h"

HumanHandPos::HumanHandPos() : Xd(6, 1) {}

void HumanHandPos::updateHumanHandData(const kinova_test::HandPosMsg &msg)
{
    for (unsigned int i = 0; i < 3; i++)
    {
        Xd[i] = msg.hand_pos[i];
        Xd[i + 3] = msg.hand_rot[i];
    }
}