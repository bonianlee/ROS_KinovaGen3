#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_

#include "Matrix.h"
#include <cmath>

// Parameters
#define NODE 100
const double kLambda = 1;
const double kK = 2;
const double kKj = 5;
const double kKr = 8;
const double kEta = 2;
const double kBj = 1;
const double kCj_v[2] = {-10, 10};
const double kCj_a[2] = {-60, 60};
const double kCj_q[2] = {-2 * M_PI, 2 * M_PI};
const double kCj_dq[2] = {-3, 3};

// functions
void contrller_params(const Matrix<double> &J, const Matrix<double> &Jinv, const Matrix<double> &dJinv, const Matrix<double> &e, const Matrix<double> &de, const Matrix<double> &dq, const Matrix<double> &subtasks, const Matrix<double> &dsubtasks, Matrix<double> &s, Matrix<double> &v, Matrix<double> &a, Matrix<double> &r);
Matrix<double> get_phi(const Matrix<double> &s, const Matrix<double> &v, const Matrix<double> &a, const Matrix<double> &q, const Matrix<double> &dq);
Matrix<double> get_dW_hat(const Matrix<double> &phi, const Matrix<double> &s);
Matrix<double> controller(const Matrix<double> &J, const Matrix<double> &de, const Matrix<double> &s, const Matrix<double> &r, const Matrix<double> &phi, const Matrix<double> &W_hat);

#endif