//
// Created by hitlfh on 2024/8/1.
//

#ifndef SRC_BASEBAMULTIDIM_H
#define SRC_BASEBAMULTIDIM_H

#include "qm_compliant/CompliantBase.h"
#include <std_msgs/Float64.h>
namespace qm{
using namespace ocs2;

class BaseBAMultiDim : public CompliantBase {
public:
    BaseBAMultiDim(const PinocchioInterface &pinocchioInterface, CentroidalModelInfo info,
                      const PinocchioEndEffectorKinematics &armEeKinematics, ros::NodeHandle &controller_nh);
    vector_t update(const vector_t &rbdStateMeasured, scalar_t time, scalar_t period);
    void initParam();
    void setTorqueMax(ocs2::scalar_t tau_max1, ocs2::scalar_t tau_max2);
    void setProxyParam(scalar_t Mx1, scalar_t Mx2, scalar_t Dx1,  scalar_t Dx2, scalar_t Kx1, scalar_t Kx2);
    void setScalarSlidingModeParam(scalar_t Lambda, scalar_t K_1);
    void setMatrixSlidingModeParam(matrix2_t SM_Lambda, matrix2_t SM_K_1);
    void setPDParam(matrix2_t K_p, matrix2_t K_d);
    void setIDControllerParam(matrix2_t M_a,  matrix2_t C_a, vector2_t G_a);  // 设置逆动力学控制器项
    void setPIDControllerParam(matrix2_t M, matrix2_t K, matrix2_t B, matrix2_t L);
    void setDesired(vector2_t ddot_q0_, vector2_t dot_q0_, vector2_t q0_);
    void setTorqueDesired(vector2_t torque) { torque_desired = torque; };
    void setTorqueExternal(vector2_t torque) { tau_ext = torque;};
    void setTorqueFeedback(vector2_t torque) { torque_feedback = torque; };
    vector_t getJoint1ProxyState();
    vector_t getJoint2ProxyState();  
private:
    vector2_t projectionFunction(const vector2_t& x);

/* // 一维param
    ros::Publisher qx_pub_, torque_pub_, tau_pub_, torque_ext_pub_;
*/

    size_t generalizedCoordinatesNum_{};
    // flag
    bool init_flag_, idx_flag_, param_flag_, tau_flag_;
    // Bounded Multi_Admittance params   目前考虑二维
    scalar_t T_;
    scalar_t lambda, k_1; // 标量形式滑模项参数
    Eigen::Matrix2d Lambda, K_1; // 矩阵形式滑模项参数
    Eigen::Matrix2d Mx, Dx, Kx, M, B, B_hat, Khat, K, C, Mat_1, Mat_2, Kp_, Kd_;  // M为当前惯性项
    Eigen::Matrix2d M_pid, K_pid, B_pid, L_pid;  // PID控制器参数
    Eigen::Vector2d a, a_pre;
    Eigen::Vector2d q0, dot_q0, ddot_q0;  // 期望状态
    Eigen::Vector2d qx, qx_pre, q, q_pre, ax;
    Eigen::Vector2d ux, ux_pre;
    Eigen::Vector2d tau, tau_star, tau_max;
    Eigen::Vector2d phi_a, phi_b;
    Eigen::Vector2d q1_star, ux_star, qx_star;
    Eigen::Vector2d n;  // 当前非线性项
    Eigen::Vector2d G;  // 当前重力项
    Eigen::Vector2d tau_ext; //外力矩
    Eigen::Vector2d torque_desired;
    Eigen::Vector2d torque_feedback;
    Eigen::Vector2d tau_ext_temp; //设为0


    // publisher
    ros::Publisher qx1_pub_, torque1_limit_pub_, tau1_pub_, torque1_ext_pub_;
    ros::Publisher qx2_pub_, torque2_limit_pub_, tau2_pub_, torque2_ext_pub_;
    ros::Publisher tau_star1_pub_, tau_star2_pub_;
    ros::Publisher Multi_gravity1_pub_, Multi_gravity2_pub_;
};

}

#endif //SRC_BASEBAMULTIDIM_H
