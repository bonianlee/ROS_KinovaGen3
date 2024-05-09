#include "kinova_test/controller.h"

void contrller_params(const Matrix<double> &J, const Matrix<double> &Jinv, const Matrix<double> &dJinv, const Matrix<double> &e, const Matrix<double> &de, const Matrix<double> &dq, const Matrix<double> &subtasks, const Matrix<double> &dsubtasks, Matrix<double> &s, Matrix<double> &v, Matrix<double> &a, Matrix<double> &r)
{
    Matrix<double> lambda(DOF, DOF, MatrixType::Diagonal, LAMBDA_INITLIST);
    s = -Jinv * lambda * e + dq - subtasks;
    v = Jinv * lambda * e + subtasks;
    a = dJinv * lambda * e + Jinv * lambda * de + dsubtasks;
    r = J * s;
}

namespace chang
{
    void get_phi(const Matrix<double> &v, const Matrix<double> &a, const Matrix<double> &q, const Matrix<double> &dq, Matrix<double> &phi, unsigned joint)
    {
        Matrix<double> X(4, 1, MatrixType::General, {v[joint], a[joint], q[joint], dq[joint]});
        for (unsigned i = 0; i < NODE; i++)
        {
            Matrix<double> cj(4, 1);
            // v
            cj[0] = Cj_v_LOW + ((Cj_v_UP - Cj_v_LOW) / (NODE - 1)) * i;

            // a
            cj[1] = Cj_a_LOW + ((Cj_a_UP - Cj_a_LOW) / (NODE - 1)) * i;

            // q
            if (joint == 1) // joint 2
                cj[2] = (-128.9 * DEG2RAD) + ((257.8 * DEG2RAD) / (NODE - 1)) * i;
            else if (joint == 3) // joint 4
                cj[2] = (-147.8 * DEG2RAD) + ((295.6 * DEG2RAD) / (NODE - 1)) * i;
            else if (joint == 5) // joint 5
                cj[2] = (-120.3 * DEG2RAD) + ((240.6 * DEG2RAD) / (NODE - 1)) * i;
            else // joint 1 3 5 7
                cj[2] = (-2 * M_PI) + ((4 * M_PI) / (NODE - 1)) * i;

            // dq
            if (joint < 4) // joint 1-4
                cj[3] = (-1.39) + (2.78 / (NODE - 1)) * i;
            else // joint 5-7
                cj[3] = (-1.22) + (2.44 / (NODE - 1)) * i;

            double norm = (X - cj).vec_norm2();
            phi[i] = exp(-(norm * norm) / (Bj * Bj));
        }
    }

    void get_dW_hat(const Matrix<double> &phi, const Matrix<double> &s, Matrix<double> &dW_hat, unsigned joint)
    {
        dW_hat = -Gamma * phi * s[joint];
    }

    void controller(const Matrix<double> &J, const Matrix<double> &dx, const Matrix<double> &dxd, const Matrix<double> &s, const Matrix<double> &r, const Matrix<double> &sigma, Matrix<double> &tau)
    {
        Matrix<double> K(7, 7, MatrixType::Diagonal, K_INITLIST);
        Matrix<double> tau_bar = Kr * r - Kj * (dxd - dx) + PINV(r.transpose()) * dx.transpose() * Kj * dxd;
        tau = sigma - K * s - J.transpose() * tau_bar;
    }
}

namespace lee
{
    void get_phi(const Matrix<double> &q, const Matrix<double> &dq, const Matrix<double> &dxd, const Matrix<double> &ddxd, Matrix<double> &phi)
    {
        Matrix<double> X(14 + (2 * DOF) + 1, 1);
        for (unsigned i = 0; i < (14 + 2 * DOF); i++)
        {
            if (i < 7) 
                X[i] = q[i];
            else if (6 < i && i < 14)
                X[i] = dq[i - 7];
            else if (13 < i && i < (14 + DOF))
                X[i] = dxd[i - 14];
            else if ((13 + DOF) < i && i < (14 + 2 * DOF))
                X[i] = ddxd[i - (14 + DOF)];
            else
                X[i] = 1;
        }
        for (unsigned i = 0; i < NODE; i++)
        {
            Matrix<double> cj(14 + (2 * DOF) + 1, 1);
            // q1 ~ q7
            cj[0] = (-2 * M_PI) + ((4 * M_PI) / (NODE - 1)) * i;
            cj[1] = (q2_MIN) + ((q2_MAX-q2_MIN) / (NODE - 1)) * i;
            cj[2] = (-2 * M_PI) + ((4 * M_PI) / (NODE - 1)) * i;
            cj[3] = (q4_MIN) + ((q4_MAX-q4_MIN) / (NODE - 1)) * i;
            cj[4] = (-2 * M_PI) + ((4 * M_PI) / (NODE - 1)) * i;
            cj[5] = (q6_MIN) + ((q6_MAX-q6_MIN) / (NODE - 1)) * i;
            cj[6] = (-2 * M_PI) + ((4 * M_PI) / (NODE - 1)) * i;

            // dq1 ~ dq7
            for (unsigned j = 7; j < 11; j++) // joint 1-4
                cj[j] = (-1.39) + (2.78 / (NODE - 1)) * i;
            for (unsigned j = 11; j < 14; j++) // joint 5-7
                cj[j] = (-1.22) + (2.44 / (NODE - 1)) * i;
            
            // dxd
            for (unsigned j = 14; j < (14 + DOF); j++)
                cj[j] = Cj_dxd_LOW + ((Cj_dxd_UP - Cj_dxd_LOW) / (NODE - 1)) * i;
            
            // ddxd
            for (unsigned j = 14 + DOF; j < (14 + 2 * DOF); j++)
                cj[j] = Cj_ddxd_LOW + ((Cj_ddxd_UP - Cj_ddxd_LOW) / (NODE - 1)) * i;

            // for adaptive F_ext
            for (unsigned j = 14 + 2 * DOF; j < (14 + 2 * DOF + 1); j++)
                cj[j] = Cj_Fext_LOW + ((Cj_Fext_UP - Cj_Fext_LOW) / (NODE - 1)) * i;

            double norm = (X - cj).vec_norm2();
            phi[i] = exp(-(norm * norm) / (Bj * Bj));
        }
    }

    void get_dW_hat(const Matrix<double> &phi, const Matrix<double> &derror, const double Gamma_lee, double dGamma_lee, std::vector<Matrix<double>> &W_hat, std::vector<Matrix<double>> &dW_hat)
    {
        dGamma_lee = -Alpha1 * Gamma_lee + Alpha2 * (phi.transpose() * phi)[0];
        for (int i = 0; i < DOF; i++)
            dW_hat.at(i) = -Lambda1_lee * derror[i] * phi;
        for (int i = 0; i < DOF; i++)
            dW_hat.at(i) -= Lambda2_lee * abs(dGamma_lee) * W_hat.at(i);
    }

    void controller(const Matrix<double> &G, const Matrix<double> &J, const Matrix<double> &error, const Matrix<double> &derror, const Matrix<double> &sigma, const Matrix<double> &subtasks, Matrix<double> &tau)
    {
        Matrix<double> Dd(DOF, DOF, MatrixType::Diagonal, Dd_INITLIST);
        Matrix<double> Kd(DOF, DOF, MatrixType::Diagonal, Kd_INITLIST);
        Matrix<double> G0 = 0.9 * G;
        // tau = J.transpose() * (Mx * ddxd + Cx * dxd + Gx - Dd * derror - Kd * error - PINV(derror.transpose()) * K1 * (error.transpose() * error));
        tau = G0 + J.transpose() * (sigma - Dd * derror - Kd * error - vPINV(derror.transpose()) * K1 * (error.transpose() * error)) + subtasks;
    }
    
    void joint_angle_limit_psi(const Matrix<double> &q, Matrix<double> &psi)
    {
        double psi_arr[7];
        Matrix<double> psi_tmp(7, 1);
    #ifdef JML_JOINT_ALL
        const double q_max[7] = {q1_MAX, q2_MAX, q3_MAX, q4_MAX, q5_MAX, q6_MAX, q7_MAX};
        const double q_min[7] = {q1_MIN, q2_MIN, q3_MIN, q4_MIN, q5_MIN, q6_MIN, q7_MIN};
        kinova_psi_jointAngleLimits_all(q[0], q[1], q[2], q[3], q[4], q[5], q[6], q_max[0], q_max[1], q_max[2], q_max[3], q_max[4], q_max[5], q_max[6], q_min[0], q_min[1], q_min[2], q_min[3], q_min[4], q_min[5], q_min[6], psi_arr);
        psi_tmp.update_from_matlab(psi_arr);
        for (unsigned int i = 0, k = 0; i < 7; i++, k++)
    #elif defined(JML_JOINT_246)
        const double q_max[3] = {q2_MAX, q4_MAX, q6_MAX};
        const double q_min[3] = {q2_MIN, q4_MIN, q6_MIN};
        kinova_psi_jointAngleLimits_246(q[1], q[3], q[5], q_max[0], q_max[1], q_max[2], q_min[0], q_min[1], q_min[2], psi_arr);
        psi_tmp.update_from_matlab(psi_arr);
        for (unsigned int i = 0, k = 1; i < 3; i++, k += 2)
    #endif
            if ((q_max[i] > 0 && q_min[i] < 0) || (q_max[i] < 0 && q_min[i] > 0))
                psi_tmp[k] = -psi_tmp[k];
        psi += Ks_JOINT_LIMIT * psi_tmp;
    }

    void manipulability_psi(const Matrix<double> &q, Matrix<double> &psi)
    {
        double psi_arr[7];
        Matrix<double> psi_tmp(7, 1);
        kinova_psi_manipulability(q[0], q[1], q[2], q[3], q[4], q[5], psi_arr);
        psi_tmp.update_from_matlab(psi_arr);
        psi += Ks_MANIPULABILITY * psi_tmp;
    }

    void joint_limit_subtask(const Matrix<double> &q, Matrix<double> &psi)
    {
        double psi_arr[7] = {0, 0, 0, 0, 0, 0, 0};
        Matrix<double> psi_tmp(7, 1);
        const double tol[7] = {10 * M_PI / 180, 10 * M_PI / 180, 10 * M_PI / 180, 10 * M_PI / 180, 10 * M_PI / 180, 10 * M_PI / 180, 10 * M_PI / 180};
        double qmin_tol[7], qmax_tol[7];
        // Matrix<double> tol(7, 1, MatrixType::General, {10*M_PI/180, 10*M_PI/180, 10*M_PI/180, 10*M_PI/180, 10*M_PI/180, 10*M_PI/180, 10*M_PI/180});
    #ifdef JML_JOINT_ALL
        const double q_max[7] = {q1_MAX, q2_MAX, q3_MAX, q4_MAX, q5_MAX, q6_MAX, q7_MAX};
        const double q_min[7] = {q1_MIN, q2_MIN, q3_MIN, q4_MIN, q5_MIN, q6_MIN, q7_MIN};
        for (unsigned int i = 0; i < 7; i++)
        {
            qmin_tol[i] = q_min[i] + tol[i];
            qmax_tol[i] = q_max[i] - tol[i];
        }
        for (unsigned int i = 0; i < 7; i++)
    #elif defined(JML_JOINT_246)
        const double q_max[7] = {0, q2_MAX, 0, q4_MAX, 0, q6_MAX, 0};
        const double q_min[7] = {0, q2_MIN, 0, q4_MIN, 0, q6_MIN, 0};
        for (unsigned int i = 0; i < 7; i++)
        {
            qmin_tol[i] = q_min[i] + tol[i];
            qmax_tol[i] = q_max[i] - tol[i];
        }
        for (unsigned int i = 1; i < 7; i + 2)
    #endif
        {
            if ((q[i] > q_min[i]) && (q[i] < qmin_tol[i]))
                psi_arr[i] = -(2 * (q[i] - qmin_tol[i]) * (qmin_tol[i] - q_min[i])) / pow((q[i] - q_min[i]), 3);
            else if ((q[i] > qmax_tol[i]) && (q[i] < q_max[i]))
                psi_arr[i] = -(2 * (q[i] - qmax_tol[i]) * (qmax_tol[i] - q_max[i])) / pow((q[i] - q_max[i]), 3);
            else
                psi_arr[i] = 0;
        }
        psi_tmp.update_from_matlab(psi_arr);
        for (unsigned int i = 0; i < 7; i++)
            if ((q_max[i] > 0 && q_min[i] < 0) || (q_max[i] < 0 && q_min[i] > 0))
                psi_tmp[i] = -psi_tmp[i];
        psi += Ks_BARRIER_JOINT_LIMIT * psi_tmp;
    }

    void joint_vel_limit_subtask(const Matrix<double> &dq, Matrix<double> &psi)
    {
        double psi_arr[7] = {0, 0, 0, 0, 0, 0, 0};
        Matrix<double> psi_tmp(7, 1);
        const double tol[7] = {M_PI / 8, M_PI / 8, M_PI / 8, M_PI / 8, M_PI / 8, M_PI / 8, M_PI / 8};
        const double dq_min[7] = {-1.39, -1.39, -1.39, -1.39, -1.22, -1.22, -1.22};
        const double dq_max[7] = {1.39, 1.39, 1.39, 1.39, 1.22, 1.22, 1.22};
        double dqmin_tol[7], dqmax_tol[7];
        for (unsigned int i = 0; i < 7; i++)
        {
            dqmin_tol[i] = dq_min[i] + tol[i];
            dqmax_tol[i] = dq_max[i] - tol[i];
        }
        for (unsigned int i = 0; i < 7; i++)
        {
            if ((dq[i] > dq_min[i]) && (dq[i] < dqmin_tol[i]))
                psi_arr[i] = -(2 * (dq[i] - dqmin_tol[i]) * (dqmin_tol[i] - dq_min[i])) / pow((dq[i] - dq_min[i]), 3);
            else if ((dq[i] > dqmax_tol[i]) && (dq[i] < dq_max[i]))
                psi_arr[i] = -(2 * (dq[i] - dqmax_tol[i]) * (dqmax_tol[i] - dq_max[i])) / pow((dq[i] - dq_max[i]), 3);
            else
                psi_arr[i] = 0;
        }
        psi_tmp.update_from_matlab(psi_arr);
        for (unsigned int i = 0; i < 7; i++)
            if ((dq_max[i] > 0 && dq_min[i] < 0) || (dq_max[i] < 0 && dq_min[i] > 0))
                psi_tmp[i] = -psi_tmp[i];
        psi += Ks_BARRIER_JOINT_VEL_LIMIT * psi_tmp;
    }

    void manipulator_config_psi(const Matrix<double> &q, Matrix<double> &psi)
    {
        Matrix<double> qH(7, 1, MatrixType::General, qH_INITLIST);
        Matrix<double> psi_tmp(7, 1);
        psi_tmp = qH - q;
        psi += Ks_MANIPULATOR_CONFIG * psi_tmp;
    }

    void null_space_subtasks(Matrix<double> &J, Matrix<double> &Jinv, Matrix<double> &psi, const Matrix<double> &dq, Matrix<double> &subtasks)
    {
        Matrix<double> eye(7, 7, MatrixType::Diagonal, {1, 1, 1, 1, 1, 1, 1});
        Matrix<double> Ksd(7, 7, MatrixType::Diagonal, Ksd_INITLIST);
        subtasks = (eye - Jinv * J) * (psi - Ksd * dq);
        psi.zeros();
    }
}

void joint_angle_limit_psi(const Matrix<double> &q, Matrix<double> &psi)
{
    double psi_arr[7];
    Matrix<double> psi_tmp(7, 1);
#ifdef JML_JOINT_ALL
    const double q_max[7] = {q1_MAX, q2_MAX, q3_MAX, q4_MAX, q5_MAX, q6_MAX, q7_MAX};
    const double q_min[7] = {q1_MIN, q2_MIN, q3_MIN, q4_MIN, q5_MIN, q6_MIN, q7_MIN};
    kinova_psi_jointAngleLimits_all(q[0], q[1], q[2], q[3], q[4], q[5], q[6], q_max[0], q_max[1], q_max[2], q_max[3], q_max[4], q_max[5], q_max[6], q_min[0], q_min[1], q_min[2], q_min[3], q_min[4], q_min[5], q_min[6], psi_arr);
    psi_tmp.update_from_matlab(psi_arr);
    for (unsigned int i = 0, k = 0; i < 7; i++, k++)
#elif defined(JML_JOINT_246)
    const double q_max[3] = {q2_MAX, q4_MAX, q6_MAX};
    const double q_min[3] = {q2_MIN, q4_MIN, q6_MIN};
    kinova_psi_jointAngleLimits_246(q[1], q[3], q[5], q_max[0], q_max[1], q_max[2], q_min[0], q_min[1], q_min[2], psi_arr);
    psi_tmp.update_from_matlab(psi_arr);
    for (unsigned int i = 0, k = 1; i < 3; i++, k += 2)
#endif
        if ((q_max[i] > 0 && q_min[i] < 0) || (q_max[i] < 0 && q_min[i] > 0))
            psi_tmp[k] = -psi_tmp[k];
    psi += Ks_JOINT_LIMIT * psi_tmp;
}

void manipulability_psi(const Matrix<double> &q, Matrix<double> &psi)
{
    double psi_arr[7];
    Matrix<double> psi_tmp(7, 1);
    kinova_psi_manipulability(q[0], q[1], q[2], q[3], q[4], q[5], psi_arr);
    psi_tmp.update_from_matlab(psi_arr);
    psi += Ks_MANIPULABILITY * psi_tmp;
}

void null_space_subtasks(Matrix<double> &J, Matrix<double> &Jinv, Matrix<double> &psi, Matrix<double> &subtasks)
{
    Matrix<double> eye(7, 7, MatrixType::Diagonal, {1, 1, 1, 1, 1, 1, 1});
    subtasks = (eye - Jinv * J) * psi;
    psi.zeros();
}

void humanPos2platformVel(Matrix<double> &Xd, geometry_msgs::Twist &twist)
{
    // 前進後退
    if (Xd[2] > USER_pZ_MIN)
        twist.linear.x = PLATFORM_pLINEAR_MAX * (Xd[2] - USER_pZ_MAX) / (USER_pZ_MAX - USER_pZ_MIN) + PLATFORM_pLINEAR_MAX;
    else if (Xd[2] < USER_nZ_MIN)
        twist.linear.x = PLATFORM_nLINEAR_MAX * (Xd[2] - USER_nZ_MAX) / (USER_nZ_MAX - USER_nZ_MIN) + PLATFORM_nLINEAR_MAX;
    else
        twist.linear.x = 0;

    // 旋轉
    if (Xd[1] > USER_pY_MIN)
        twist.angular.z = PLATFORM_pANGULAR_MAX * (Xd[1] - USER_pY_MAX) / (USER_pY_MAX - USER_pY_MIN) + PLATFORM_pANGULAR_MAX;
    else if (Xd[1] < USER_nY_MIN)
        twist.angular.z = PLATFORM_nANGULAR_MAX * (Xd[1] - USER_nY_MAX) / (USER_nY_MAX - USER_nY_MIN) + PLATFORM_nANGULAR_MAX;
    else
        twist.angular.z = 0;
}

void emergency_stop(ros::Publisher &platform_pub)
{
    geometry_msgs::Twist twist;
    twist.linear.x = 0;
    twist.linear.y = 0;
    twist.linear.z = 0;
    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = 0;
    platform_pub.publish(twist);
}