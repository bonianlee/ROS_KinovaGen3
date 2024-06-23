#ifndef _HUMANHANDPOS_H_
#define _HUMANHANDPOS_H_

#include "kinova_test/Matrix.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "kinova_test/HandPosMsg.h"

class HumanHandPos
{
public:
    HumanHandPos();
    Matrix<double> Xd;
    void updateHumanHandData(const kinova_test::HandPosMsg &msg);
};

#endif