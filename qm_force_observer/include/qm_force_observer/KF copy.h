//
// Created by lfh on 2024/11/25.
//

#ifndef SRC_KF_H
#define SRC_KF_H

#include <ros/ros.h>
#include <iostream>
#include <random>
#include <ocs2_core/Types.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_centroidal_model/PinocchioCentroidalDynamics.h>
#include <dynamic_reconfigure/server.h>
#include <unsupported/Eigen/MatrixFunctions>

#include <dynamic_reconfigure/server.h>
#include <qm_force_observer/Momentum_KFConfig.h>
#include <Eigen/Sparse>

using vector3_t = Eigen::Matrix<ocs2::scalar_t, 3, 1>;
using vector6_t = Eigen::Matrix<ocs2::scalar_t, 6, 1>;
using matrix3_t = Eigen::Matrix<ocs2::scalar_t, 3, 3>;
using matrix2_t = Eigen::Matrix<ocs2::scalar_t, 2, 2>;
using vector2_t = Eigen::Matrix<ocs2::scalar_t, 2, 1>;


namespace qm{
using namespace ocs2;
class KF{

public:
    KF(const PinocchioInterface& pinocchioInterface, CentroidalModelInfo info,
                  const PinocchioEndEffectorKinematics& armEeKinematics, ros::NodeHandle &controller_nh);
    vector_t getExternalTorque(const vector_t& rbdStateMeasured, scalar_t time, scalar_t period);
    void setParam(scalar_t QcpAng, scalar_t QcpLin, scalar_t QcpLeg, scalar_t QcpArm, scalar_t Qc_f, scalar_t RcpAng, scalar_t RcpLin, scalar_t RcpLeg, scalar_t RcpArm, scalar_t Af);   
    void initParam();

private:
    PinocchioInterface pinocchioInterface_;
    CentroidalModelInfo info_;
    CentroidalModelPinocchioMapping mapping_;

    std::unique_ptr<PinocchioEndEffectorKinematics> armEeKinematics_;

    matrix3_t rotationEeMeasuredToWorld_;

    size_t generalizedCoordinatesNum_;
    size_t actuatedDofNum_;

    vector_t qMeasured_, vMeasured_, vMeasured_noise;
    matrix_t arm_j_, arm_dj_;   // 机械臂末端雅可比矩阵
    matrix_t j_, dj_;  // 足端雅可比矩阵
    matrix_t base_j_, base_dj_; // base 雅可比矩阵
    matrix_t Arm_J_PseudoInverse;  // 机械臂末端雅可比矩阵的伪逆 
    matrix_t J_T_PseudoInverse_EE;  // ETH论文中拼接后大转置矩阵的伪逆
    matrix_t J_T_PseudoInverse_base;
    vector_t hat_ExternalWrenchs_EE;
    vector_t hat_ExternalWrenchs_base;

    // 判断碰撞发生位置
    double th_joint2;    // 判断机械臂joint2是否发生碰撞的阈值
    double th_joint3;    // 判断机械臂joint3是否发生碰撞的阈值
    double th_baseX;     
    double th_baseY;
    double th_collision;  // 判断是否发生了碰撞
    double force_isolation; //1 代表碰撞发生在机械臂末端，-1代表发生在base上, 0代表未发生碰撞

    // 标志位 ：是否含有噪声以及模型不确定性
    int flag_noise ;
    int flag_uncertainty_M;
    int flag_uncertainty_C;
    int flag_uncertainty_G;

    matrix_t M;
    matrix_t C;
    vector_t G;

    vector_t jointTorque;  // 实际各关节力矩  dim = info_.actuatedDofNum
    vector_t generalizedTorque;  // 广义关节力矩（包含了base的零分量） dim =  info_.generalizedCoordinatedNum


    vector_t p;  // 机器人实际动量
    vector_t dot_p; 
    vector_t p_hat;  // 机器人估计动量
    vector_t dot_p_hat;  
    vector_t torque_ext_hat;
    vector_t base_torque_ext_hat;   // 力施加到末端力传感器上得出的base部分外力矩分量
    vector_t baseforce_torque_ext_hat;  // 力施加到base上得出的base部分外力矩分量
    vector_t force_ext_hat_EE;
    vector_t force_ext_hat_base;
    //vector_t force_ext_true;
    scalar_t norm_force_ext_hat_EE;
    scalar_t norm_force_ext_hat_base;
    //scalar_t norm_force_ext_true;
    

    // KF部分的参数
    matrix3_t Qcp_ang;  //base 姿态的动量过程噪声协方差矩阵
    matrix3_t Qcp_Linear;  //base 位置的动量过程噪声协方差矩阵
    matrix_t Qcp_Leg; //leg的动量过程噪声协方差矩阵
    matrix_t Qcp_Arm; //arm 动量过程噪声协方差矩阵
    matrix_t Qcp;
    matrix_t Qcf;
    matrix_t Qc;  // 总的过程噪声协方差矩阵
    matrix3_t Rc_ang;  //base 姿态的动量测量噪声协方差矩阵
    matrix3_t Rc_Linear;  //base 位置的动量测量噪声协方差矩阵
    matrix_t Rc_Leg; //leg的动量测量噪声协方差矩阵
    matrix_t Rc_Arm; //arm 动量测量噪声协方差矩阵
    matrix_t Rc;  // 总的测量噪声协方差矩阵
    int dim_p = 24; // 动量的维度
    int dim_f = 18; // 外力的维度
    matrix_t A_f;
    vector_t x_pred;
    vector_t x_hat_KF;
    matrix_t P_pred;
    matrix_t P_KF;
    vector_t tau_bar;
    matrix_t K_KF;

    //连续时间系统矩阵
    matrix_t Ac;
    matrix_t Bc;
    matrix_t Cc;
    matrix_t mat_zero1;
    matrix_t mat_zero2;
    matrix_t mat_zero3;
    matrix_t mat_zero4;
    matrix_t mat_zero5;
    matrix_t mat_indentity1;
    matrix_t mat_indentity2;
    matrix_t mat_indentity3;
    matrix_t mat_indentity4;

    //Eigen::SparseMatrix<double> sparseMatrix(rows, cols);
    // 离散化需要用到的矩阵
    matrix_t Ad;
    matrix_t Bd;
    matrix_t Cd;
    
    matrix_t Rd;
    matrix_t Qd;

    matrix_t Mat_d_1;
    matrix_t Mat_c_1;
    matrix_t Mat_d_2;
    matrix_t Mat_c_2;
    matrix_t M_11;
    matrix_t M_12;

    matrix_t H;

    ros::Publisher armTau2hat_pub_;
    ros::Publisher armTau3hat_pub_;
    ros::Publisher baseXhat_pub_;
    ros::Publisher baseYhat_pub_;
    ros::Publisher extForcehatAbs_EE_pub_;
    ros::Publisher extForcehatAbs_base_pub_;
    ros::Publisher extForceAbs_pub_;
    ros::Publisher force_isolation_pub_;

    ros::Publisher baseforceXhat_pub_;   //  发布力施加在base上时base的外力矩估计
    ros::Publisher baseforceYhat_pub_;

    ros::Publisher tau_ext_baseXhat_pub_; //  发布估计到的外力矩的base分量(直接从估计到的外力矩中发布，不经过多余处理)
    ros::Publisher tau_ext_baseYhat_pub_;

    scalar_t T;

    matrix_t selectMatrix;  // ETH论文中的S矩阵

    // 动态传参
    void dynamicCallback(qm_force_observer::Momentum_KFConfig& config, uint32_t /*level*/);
    std::shared_ptr<dynamic_reconfigure::Server<qm_force_observer::Momentum_KFConfig>> dynamic_srv_{};

};
}



#endif //SRC_KF_H
