#include "kinova_test/KinovaGen3Model.h"
#include <cmath>

Matrix<double> forward_kinematic_3dof(const Matrix<double> &q)
{
    Matrix<double> Ab0(4, 4), A01(4, 4), A12(4, 4), A23(4, 4), A34(4, 4), A45(4, 4), A56(4, 4), A67(4, 4);
    Ab0(0, 0) = 1;
    Ab0(1, 1) = -1;
    Ab0(2, 2) = -1;
    Ab0(2, 3) = TOMETER(KinovaParams::d0);
    Ab0(3, 3) = 1;
    A01(0, 0) = cos(q[0]);
    A01(0, 2) = sin(q[0]);
    A01(0, 3) = -TOMETER(KinovaParams::d2) * sin(q[0]);
    A01(1, 0) = sin(q[0]);
    A01(1, 2) = -cos(q[0]);
    A01(1, 3) = TOMETER(KinovaParams::d2) * cos(q[0]);
    A01(2, 1) = 1;
    A01(2, 3) = -TOMETER(KinovaParams::d1);
    A01(3, 3) = 1;
    A12(0, 0) = cos(q[1]);
    A12(0, 2) = -sin(q[1]);
    A12(0, 3) = TOMETER(KinovaParams::d3) * sin(q[1]);
    A12(1, 0) = sin(q[1]);
    A12(1, 2) = cos(q[1]);
    A12(1, 3) = -TOMETER(KinovaParams::d3) * cos(q[1]);
    A12(2, 1) = -1;
    A12(2, 3) = -TOMETER(KinovaParams::d4);
    A12(3, 3) = 1;
    A23(0, 0) = cos(q[2]);
    A23(0, 2) = sin(q[2]);
    A23(0, 3) = -TOMETER(KinovaParams::d6) * sin(q[2]);
    A23(1, 0) = sin(q[2]);
    A23(1, 2) = -cos(q[2]);
    A23(1, 3) = TOMETER(KinovaParams::d6) * cos(q[2]);
    A23(2, 1) = 1;
    A23(2, 3) = -TOMETER(KinovaParams::d5);
    A23(3, 3) = 1;
    A34(0, 0) = cos(q[3]);
    A34(0, 2) = -sin(q[3]);
    A34(0, 3) = TOMETER(KinovaParams::d7) * sin(q[3]);
    A34(1, 0) = sin(q[3]);
    A34(1, 2) = cos(q[3]);
    A34(1, 3) = -TOMETER(KinovaParams::d7) * cos(q[3]);
    A34(2, 1) = -1;
    A34(2, 3) = -TOMETER(KinovaParams::d8);
    A34(3, 3) = 1;
    A45(0, 0) = cos(q[4]);
    A45(0, 2) = sin(q[4]);
    A45(1, 0) = sin(q[4]);
    A45(1, 2) = -cos(q[4]);
    A45(2, 1) = 1;
    A45(2, 3) = -TOMETER(KinovaParams::d9);
    A45(3, 3) = 1;
    A56(0, 0) = cos(q[5]);
    A56(0, 2) = -sin(q[5]);
    A56(0, 3) = TOMETER(KinovaParams::d10) * sin(q[5]);
    A56(1, 0) = sin(q[5]);
    A56(1, 2) = cos(q[5]);
    A56(1, 3) = -TOMETER(KinovaParams::d10) * cos(q[5]);
    A56(2, 1) = -1;
    A56(3, 3) = 1;
    A67(0, 0) = cos(q[6]);
    A67(0, 1) = sin(q[6]);
    A67(1, 0) = sin(q[6]);
    A67(1, 1) = -cos(q[6]);
    A67(2, 2) = -1;
    A67(2, 3) = -TOMETER(KinovaParams::d11) - TOMETER(KinovaParams::d12);
    A67(3, 3) = 1;
    Matrix<double> effector(4, 1), pos(3, 1);
    effector[3] = 1;
    effector = Ab0 * A01 * A12 * A23 * A34 * A45 * A56 * A67 * effector;
    for (int i = 0; i < 3; i++)
        pos[i] = effector[i];
    return pos;
}

Matrix<double> forward_kinematic_6dof(const Matrix<double> &q)
{
    double X_arr[7];
    Matrix<double> X(6, 1);
    kinova_FK(q[0], q[1], q[2], q[3], q[4], q[5], q[6], X_arr);
    X.update_from_matlab(X_arr);
    return X;
}