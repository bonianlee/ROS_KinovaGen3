#include "kinova_test/controller.h"

void contrller_params(const Matrix<double> &J, const Matrix<double> &Jinv, const Matrix<double> &dJinv, const Matrix<double> &e, const Matrix<double> &de, const Matrix<double> &dq, const Matrix<double> &subtasks, const Matrix<double> &dsubtasks, Matrix<double> &s, Matrix<double> &v, Matrix<double> &a, Matrix<double> &r)
{
    s = -Jinv * kLambda * e + dq - subtasks;
    v = Jinv * kLambda * e + subtasks;
    a = dJinv * kLambda * e + Jinv * kLambda * de + dsubtasks;
    r = J * s;
}

void get_phi(const Matrix<double> &v, const Matrix<double> &a, const Matrix<double> &q, const Matrix<double> &dq, Matrix<double> &phi)
{
    double distance_square, cj, X;
    for (unsigned i = 0; i < NODE; i++)
    {
        for (unsigned j = 0; j < 7; j++)
        {
            distance_square = 0;
            for (unsigned k = 0; k < 4; k++)
            {
                if (k == 0)
                {
                    X = v[j];
                    cj = kCj_v[0] + (kCj_v[1] - kCj_v[0]) * i / (NODE - 1);
                }
                else if (k == 1)
                {
                    X = a[j];
                    cj = kCj_a[0] + (kCj_a[1] - kCj_a[0]) * i / (NODE - 1);
                }
                else if (k == 2)
                {
                    X = q[j];
                    cj = kCj_q[0] + (kCj_q[1] - kCj_q[0]) * i / (NODE - 1);
                }
                else
                {
                    X = dq[j];
                    cj = kCj_dq[0] + (kCj_dq[1] - kCj_dq[0]) * i / (NODE - 1);
                }
                distance_square += (X - cj) * (X - cj);
            }
            phi(i, j) = exp(-distance_square / (kBj * kBj));
        }
    }
}

void get_dW_hat(const Matrix<double> &phi, const Matrix<double> &s, Matrix<double> &dW_hat)
{
    dW_hat = -kEta * phi * s;
}

void controller(const Matrix<double> &J, const Matrix<double> &de, const Matrix<double> &s, const Matrix<double> &r, const Matrix<double> &phi, const Matrix<double> &W_hat, Matrix<double> &tau)
{
    Matrix<double> tau_bar = kKr * r - kKj * de;
    tau = phi.transpose() * W_hat - kK * s - J.transpose() * tau_bar;
}

void joint_angle_limit(const Matrix<double> &q, Matrix<double> &psi)
{
    double psi_arr[7];
    Matrix<double> psi_tmp(7, 1);
    kinova_psi_jointAngleLimits(q[0], q[1], q[2], q[3], q[4], q[5], q[6], q_max[0], q_max[1], q_max[2], q_max[3], q_max[4], q_max[5], q_max[6], q_min[0], q_min[1], q_min[2], q_min[3], q_min[4], q_min[5], q_min[6], psi_arr);
    for (int i = 0; i < 7; i++)
        if ((q_max[i] > 0 && q_min[i] < 0) || (q_max[i] < 0 && q_min[i] > 0))
            psi[i] = -psi[i];
    for (int i = 0; i < 7; i += 2)
        psi_arr[i] = 0;
    psi_tmp.update_from_matlab(psi_arr);
    psi += -0.8 * psi_tmp; // 全部的 qmax 跟 qmin 反向
}

void manipulability(const Matrix<double> &q, Matrix<double> &psi)
{
    double psi_arr[7];
    Matrix<double> psi_tmp(7, 1);
    psi_tmp.update_from_matlab(psi_arr);
    psi += psi_tmp;
}

void null_space_subtasks(Matrix<double> &J, Matrix<double> &Jinv, Matrix<double> &psi, Matrix<double> &subtasks)
{
    Matrix<double> eye(7, 7);
    for (unsigned i = 0; i < 7; i++)
        eye(i, i) = 1;
    subtasks = (eye - Jinv * J) * psi;
    for (unsigned i = 0; i < 7; i++)
        psi[i] = 0;
}
