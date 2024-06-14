#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_

#include <cmath>
#include "kinova_test/Matrix.h"
#include "kinova_test/KinovaGen3Model.h"
#include "kinova_test/HumanState.h"
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

// Controller Parameters
#define DOF 6U
#define NODE 25U

// lee's Controller Parameters
#if  (DOF == 3)
#define Dd_INITLIST \
    {               \
        20, 20, 20     \
    }
#define Kd_INITLIST \
    {               \
        100, 100, 100     \
    }
#define K1 0.01
#else
#define Dd_INITLIST \
    {               \
        20, 20, 20, 20, 20, 20    \
    }
#define Kd_INITLIST \
    {               \
        100, 100, 100, 100, 100, 100     \
    }
#define K1 0.01
#endif
// Admittance interface
#define Madm_INITLIST \
    {                 \
        2, 2, 2     \
    }
#define Dadm_INITLIST \
    {                 \
        0.2, 0.2, 0.2     \
    }

// non-holonomic pd
#define P_INITLIST \
    {              \
        2, 2     \
    }
#define D_INITLIST \
    {              \
        2, 2     \
    }

// lee's RBFNN Parameters
#define Cj_dxd_UP 3
#define Cj_dxd_LOW (-3)
#define Cj_ddxd_UP 5
#define Cj_ddxd_LOW (-5)
#define Cj_Fext_UP (10)
#define Cj_Fext_LOW (-10)
#define Bj 20
#define Lambda1_lee 10
#define Lambda2_lee 10
#define Alpha1 1
#define Alpha2 1

// lee's Subtasks Parameters
#define Ks_MANIPULABILITY 0
#define Ks_JOINT_LIMIT 0
#define Ks_BARRIER_JOINT_LIMIT 0.05
#define Ks_BARRIER_JOINT_VEL_LIMIT 0.05
#define Ks_MANIPULATOR_CONFIG 10
#define Ksd_INITLIST \
    {               \
        0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05    \
    }
#define qH_INITLIST \
    {               \
        0, 0.2618, 3.1416, -2.2689, 0, 0.9599, 1.5708   \
    }

// chang's Controller Parameters
#if (DOF == 3)
#define LAMBDA_INITLIST \
    {                   \
        15, 15, 25      \
    }
#else
#define LAMBDA_INITLIST     \
    {                       \
        10, 10, 12, 8, 8, 8 \
    }
#endif

#define K_INITLIST           \
    {                        \
        5, 10, 5, 8, 5, 8, 5 \
    }
#define Kj 0.5
#define Kr 25U
#define Gamma 2.5

// RBFNN Parameters
#define Cj_v_UP 40
#define Cj_v_LOW (-40)
#define Cj_a_UP 80
#define Cj_a_LOW (-80)
#define Bj 20

// all: JML_JOINT_ALL, Only even joints (2,4,6): JML_JOINT_246
#define JML_JOINT_246
#define q1_MAX (100 * DEG2RAD)
#define q1_MIN (-100 * DEG2RAD)
// #define q2_MAX (110 * DEG2RAD)
// #define q2_MIN (-110 * DEG2RAD)
#define q2_MAX 1.1345
#define q2_MIN -1.1345
#define q3_MAX (100 * DEG2RAD)
#define q3_MIN (-100 * DEG2RAD)
// #define q4_MAX (147.8 * DEG2RAD)
// #define q4_MIN (-147.8 * DEG2RAD)
#define q4_MAX 0.5376
#define q4_MIN -2.6
#define q5_MAX (100 * DEG2RAD)
#define q5_MIN (-100 * DEG2RAD)
// #define q6_MAX (120.3 * DEG2RAD)
// #define q6_MIN (-120.3 * DEG2RAD)
#define q6_MAX 1.4888
#define q6_MIN -1.4888
#define q7_MAX (100 * DEG2RAD)
#define q7_MIN (-100 * DEG2RAD)

#define PLATFORM_pLINEAR_MAX 1
#define PLATFORM_nLINEAR_MAX (-1)
#define USER_pZ_MAX 0.6
#define USER_pZ_MIN 0.2
#define USER_nZ_MAX (-0.6)
#define USER_nZ_MIN (-0.2)
#define PLATFORM_pANGULAR_MAX 0.5
#define PLATFORM_nANGULAR_MAX (-0.5)
#define USER_pY_MAX (0.45)
#define USER_pY_MIN (0.05)
#define USER_nY_MAX (-0.5)
#define USER_nY_MIN (-0.1)

// functions
namespace chang
{
    void get_phi(const Matrix<double> &v, const Matrix<double> &a, const Matrix<double> &q, const Matrix<double> &dq, Matrix<double> &phi, unsigned joint);
    void get_dW_hat(const Matrix<double> &phi, const Matrix<double> &s, Matrix<double> &dW_hat, unsigned joint);
    void controller(const Matrix<double> &J, const Matrix<double> &dx, const Matrix<double> &dxd, const Matrix<double> &s, const Matrix<double> &r, const Matrix<double> &sigma, Matrix<double> &tau);
}
namespace lee
{
    // Manipulator Controller
    void get_phi(const Matrix<double> &q, const Matrix<double> &dq, const Matrix<double> &dxd, const Matrix<double> &ddxd, Matrix<double> &phi);
    void get_dW_hat(const Matrix<double> &phi, const Matrix<double> &derror, const double Gamma_lee, double dGamma_lee, std::vector<Matrix<double>> &W_hat, std::vector<Matrix<double>> &dW_hat);
    void controller(const Matrix<double> &G, const Matrix<double> &J, const Matrix<double> &error, const Matrix<double> &derror, const Matrix<double> &sigma, const Matrix<double> &subtasks, Matrix<double> &tau);
    void joint_angle_limit_psi(const Matrix<double> &q, Matrix<double> &psi);
    void manipulability_psi(const Matrix<double> &q, Matrix<double> &psi);
    void joint_limit_subtask(const Matrix<double> &q, Matrix<double> &psi);
    void joint_vel_limit_subtask(const Matrix<double> &dq, Matrix<double> &psi);
    void manipulator_config_psi(const Matrix<double> &q, Matrix<double> &psi);
    void null_space_subtasks(Matrix<double> &J, Matrix<double> &Jinv, Matrix<double> &psi, const Matrix<double> &dq, Matrix<double> &subtasks);
    // Whole-body Controller
    void wholeBody_controller(const Matrix<double> &G_w, const Matrix<double> &J_w, const Matrix<double> &error, const Matrix<double> &derror, const Matrix<double> &sigma, const Matrix<double> &subtasks, Matrix<double> &tau_w);
    void Admittance_interface(Matrix<double> &dq_p, Matrix<double> &ddq_p, Matrix<double> &tau_w);
    void calculate_Jp(Matrix<double> &q_p, Matrix<double> &Jp, Matrix<double> &Jp_inv);
    void non_holonomic_pd(Matrix<double> &dq_p_error, Matrix<double> &ddq_p_error, Matrix<double> &Jp_inv, Matrix<double> &dJp_inv, Matrix<double> &cmd_vel);
    void admittance2platformVel(Matrix<double> &cmd_vel, geometry_msgs::Twist &twist);
    void wholeBody_get_phi(const Matrix<double> &q_p, const Matrix<double> &q, const Matrix<double> &dq_p, const Matrix<double> &dq, const Matrix<double> &dxd, const Matrix<double> &ddxd, Matrix<double> &phi);
    void wholeBody_get_dW_hat(const Matrix<double> &phi, const Matrix<double> &derror, const double Gamma_lee, double dGamma_lee, std::vector<Matrix<double>> &W_hat, std::vector<Matrix<double>> &dW_hat);
}
void contrller_params(const Matrix<double> &J, const Matrix<double> &Jinv, const Matrix<double> &dJinv, const Matrix<double> &e, const Matrix<double> &de, const Matrix<double> &dq, const Matrix<double> &subtasks, const Matrix<double> &dsubtasks, Matrix<double> &s, Matrix<double> &v, Matrix<double> &a, Matrix<double> &r);
void joint_angle_limit_psi(const Matrix<double> &q, Matrix<double> &psi);
void manipulability_psi(const Matrix<double> &q, Matrix<double> &psi);
void null_space_subtasks(Matrix<double> &J, Matrix<double> &Jinv, Matrix<double> &psi, Matrix<double> &subtasks);
void humanPos2platformVel(Matrix<double> &Xd, geometry_msgs::Twist &cmd_vel);
void emergency_stop(ros::Publisher &platform_pub);
#endif