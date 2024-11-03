//
// Created by lfh on 2024/10/26.
//

#ifndef SRC_STA_H
#define SRC_STA_H

#include <ros/ros.h>
#include <iostream>
#include <ocs2_core/Types.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_centroidal_model/PinocchioCentroidalDynamics.h>
#include <dynamic_reconfigure/server.h>

#include <dynamic_reconfigure/server.h>
#include <qm_force_observer/ObserverConfig.h>

using vector3_t = Eigen::Matrix<ocs2::scalar_t, 3, 1>;
using vector6_t = Eigen::Matrix<ocs2::scalar_t, 6, 1>;
using matrix3_t = Eigen::Matrix<ocs2::scalar_t, 3, 3>;
using matrix2_t = Eigen::Matrix<ocs2::scalar_t, 2, 2>;
using matrix6_t = Eigen::Matrix<ocs2::scalar_t, 6, 6>;
using matrix12_t = Eigen::Matrix<ocs2::scalar_t, 12, 12>;
using vector2_t = Eigen::Matrix<ocs2::scalar_t, 2, 1>;


namespace qm{
using namespace ocs2;
class STA{

public:
    STA(const PinocchioInterface& pinocchioInterface, CentroidalModelInfo info,
                  const PinocchioEndEffectorKinematics& armEeKinematics, ros::NodeHandle &controller_nh);
    vector_t getExternalTorque(const vector_t& rbdStateMeasured, scalar_t time, scalar_t period);
    void setParam(scalar_t K1_ang, scalar_t K2_ang, scalar_t K3_ang, scalar_t K4_ang,
                    scalar_t K1_lin, scalar_t K2_lin,scalar_t K3_lin,scalar_t K4_lin,
                    scalar_t K1_leg, scalar_t K2_leg,scalar_t K3_leg,scalar_t K4_leg,
                    scalar_t K1_arm, scalar_t K2_arm, scalar_t K3_arm, scalar_t K4_arm);   // 为base的姿态， base的位置， 腿部关节， 机械臂关节分别设置STA观测器增益
    void initParam();

private:
    PinocchioInterface pinocchioInterface_;
    CentroidalModelInfo info_;
    CentroidalModelPinocchioMapping mapping_;

    std::unique_ptr<PinocchioEndEffectorKinematics> armEeKinematics_;

    matrix3_t rotationEeMeasuredToWorld_;

    size_t generalizedCoordinatesNum_;
    size_t actuatedDofNum_;

    vector_t qMeasured_, vMeasured_;
    matrix_t arm_j_, arm_dj_;   // 机械臂末端雅可比矩阵
    matrix_t j_, dj_;  // 足端雅可比矩阵
    matrix_t Arm_J_PseudoInverse;  // 机械臂末端雅可比矩阵的伪逆 
    matrix_t J_T_PseudoInverse;  // ETH论文中拼接后大转置矩阵的伪逆
    vector_t hat_ExternalWrenchs;

    matrix_t M;
    matrix_t C;
    vector_t G;

    vector_t jointTorque;  // 实际各关节力矩  dim = info_.actuatedDofNum
    vector_t generalizedTorque;  // 广义关节力矩（包含了base的零分量） dim =  info_.generalizedCoordinatedNum


    vector_t p;  // 机器人实际动量
    vector_t dot_p; 
    vector_t p_hat;  // 机器人估计动量
    vector_t dot_p_hat;  
    vector_t r;
    vector_t torque_ext_hat;
    vector_t base_torque_ext_hat;
    vector_t force_ext_hat;
    //vector_t force_ext_true;
    scalar_t norm_force_ext_hat;
    //scalar_t norm_force_ext_true;
    vector_t dot_r;  
    
    matrix3_t Angle_K1; //base 姿态的观测器增益
    matrix3_t Angle_K2;
    matrix3_t Angle_K3;
    matrix3_t Angle_K4;

    matrix3_t Linear_K1; //base 位置的观测器增益
    matrix3_t Linear_K2;
    matrix3_t Linear_K3;
    matrix3_t Linear_K4;

    matrix12_t Leg_K1; //leg的观测器增益
    matrix12_t Leg_K2;
    matrix12_t Leg_K3;
    matrix12_t Leg_K4;

    matrix6_t Arm_K1; //arm 姿态的观测器增益
    matrix6_t Arm_K2;
    matrix6_t Arm_K3;
    matrix6_t Arm_K4;

    matrix_t Gain_K1;
    matrix_t Gain_K2;
    matrix_t Gain_K3;
    matrix_t Gain_K4;

    ros::Publisher armTau2hat_pub_;
    ros::Publisher armTau3hat_pub_;
    ros::Publisher baseXhat_pub_;
    ros::Publisher baseYhat_pub_;
    ros::Publisher extForcehatAbs_pub_;
    ros::Publisher extForceAbs_pub_;

    scalar_t T;

    matrix_t selectMatrix;  // ETH论文中的S矩阵

    // 动态传参
    void dynamicCallback(qm_force_observer::ObserverConfig& config, uint32_t /*level*/);
    std::shared_ptr<dynamic_reconfigure::Server<qm_force_observer::ObserverConfig>> dynamic_srv_{};

};
}



#endif //SRC_STA_H
