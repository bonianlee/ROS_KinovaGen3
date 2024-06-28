#ifndef _KINOVACONFIG_H_
#define _KINOVACONFIG_H_

#include <iomanip>
#include <cmath>

#include <KDetailedException.h>
#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <ActuatorConfigClientRpc.h>
#include <SessionClientRpc.h>
#include <SessionManager.h>
#include <RouterClient.h>
#include <TransportClientUdp.h>
#include <TransportClientTcp.h>
#include <google/protobuf/util/json_util.h>

#include "utilities.h"
#include "kinova_test/Matrix.h"
#include "kinova_test/KinovaGen3Model.h"

#include "ros/ros.h"

#if defined(_MSC_VER)
#include <Windows.h>
#else
#include <unistd.h>
#endif
#include <time.h>

#define PORT 10000
#define PORT_REAL_TIME 10001
#define WAITING_TIME 20

namespace k_api = Kinova::Api;

int64_t GetTickUs();
bool example_move_to_home_position(k_api::Base::BaseClient *base);

/*
 * 回歸 Home 姿態同時傳送各關節角度。
 *
 * @param kinova_pub: 傳送 kinovaMsg 的 Publisher
 *
 */
bool move_to_home_position_with_ros(k_api::Base::BaseClient *base, ros::Publisher &kinova_pub);

/*
 * 輸出扭矩飽和器。
 *
 * @param tau: 7*1 的輸出扭矩向量
 *
 */
void torque_saturation(Matrix<double> &tau);

/*
 * 全身控制之輸出扭矩飽和器。
 *
 * @param tau: 7*1 的輸出扭矩向量
 *
 */
void wholeBody_torque_saturation(Matrix<double> &tau_w);

/*
 * 重力補償。
 *
 * @param q: 7*1 關節角度
 * @param init_tau: 7*1 初始扭矩
 * @param tau: 7*1 的輸出扭矩向量
 */
void gravity_compensation(const Matrix<double> &q, const double init_tau[7], Matrix<double> &tau);

/*
 * 全身控制之機械手臂重力補償。
 *
 * @param q_w: 10*1 關節角度
 * @param init_tau: 7*1 初始扭矩(機械手臂)
 * @param tau_w: 10*1 的輸出扭矩向量
 */
void wholeBody_gravity_compensation(const Matrix<double> &q_w, const double init_tau[7], Matrix<double> &tau_w);

/*
 * 將關節角度轉到 -inf 到 inf。
 *
 * @param curr_pos: 7*1 當前關節角度 (-pi~pi)
 * @param prev_q: 7*1 先前關節角度 (-inf~inf)
 * @param round: 7*1 各關節圈數
 * @param q: 7*1 當前關節角度 (-inf~inf)
 */
void q2inf(const Matrix<double> &curr_pos, const Matrix<double> &prev_q, Matrix<int> &round, Matrix<double> &q);

/*
 * 將移動平台旋轉角度轉到 -inf 到 inf。
 *
 * @param curr_pos_p: 當前旋轉角度 (-pi~pi)
 * @param prev_q_p: 先前旋轉角度 (-inf~inf)
 * @param round_p: 旋轉圈數
 * @param q_p: 當前旋轉角度 (-inf~inf)
 */
void q2inf_p(const double &curr_pos_p, const Matrix<double> &prev_q_p, int &round_p, Matrix<double> &q_p);

/*
 * Kinova 末端夾爪開闔。
 *
 * @param gripper_motor_command: k_api::GripperCyclic::MotorCommand 指標型態
 * @param triggerVal: 操作員左手控制器的 trigger button value
 */
void gripper_control(k_api::GripperCyclic::MotorCommand *gripper_motor_command, float triggerVal);
#endif