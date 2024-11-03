//
// Created by skywoodsz on 2023/4/22.
//

#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/cholesky.hpp>


#include <ocs2_robotic_tools/common/AngularVelocityMapping.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>

//#include "qm_compliant/CompliantBase.h"
#include "qm_force_observer/STA.h"
#include <std_msgs/Float64.h>

namespace qm{
using namespace ocs2;
STA::STA(const PinocchioInterface &pinocchioInterface, CentroidalModelInfo info,
                             const PinocchioEndEffectorKinematics &armEeKinematics, ros::NodeHandle &controller_nh)
     : pinocchioInterface_(pinocchioInterface),
       info_(std::move(info)),
       mapping_(info_),
       armEeKinematics_(armEeKinematics.clone())
{
    qMeasured_ = vector_t(info_.generalizedCoordinatesNum);
    vMeasured_ = vector_t(info_.generalizedCoordinatesNum);
    arm_j_ = matrix_t(6, info_.generalizedCoordinatesNum);
    arm_dj_ = matrix_t(6, info_.generalizedCoordinatesNum);
    G = vector_t(info_.generalizedCoordinatesNum);

    generalizedCoordinatesNum_ = info_.generalizedCoordinatesNum;
    actuatedDofNum_ = info_.actuatedDofNum;
    initParam();

    ROS_INFO_STREAM("\033[32m STA initialized\033[0m");

    // dynamic reconfigure
    ros::NodeHandle nh_weight = ros::NodeHandle(controller_nh,"observer");
    dynamic_srv_ = std::make_shared<dynamic_reconfigure::Server<qm_force_observer::ObserverConfig>>(nh_weight);
    dynamic_reconfigure::Server<qm_force_observer::ObserverConfig>::CallbackType cb = [this](auto&& PH1, auto&& PH2) {
        dynamicCallback(std::forward<decltype(PH1)>(PH1), std::forward<decltype(PH2)>(PH2));
    };
    dynamic_srv_->setCallback(cb);
}


// 初始化各种参数
void STA::initParam(){
    Angle_K1.setZero();
    Angle_K2.setZero();
    Angle_K3.setZero();
    Angle_K4.setZero();

    Linear_K1.setZero();
    Linear_K2.setZero();
    Linear_K3.setZero();
    Linear_K4.setZero();
    
    Leg_K1.setZero();
    Leg_K2.setZero();
    Leg_K3.setZero();
    Leg_K4.setZero();

    Arm_K1.setZero();
    Arm_K2.setZero();
    Arm_K3.setZero();
    Arm_K4.setZero();
    

    int totalRows = Angle_K1.rows() + Linear_K1.rows() + Leg_K1.rows() + Arm_K1.rows();
    dot_r.resize(totalRows);
    dot_r.setZero();
    r.resize(totalRows);
    r.setZero(totalRows);
    p.resize(totalRows);
    p.setZero();
    dot_p.resize(totalRows);
    dot_p.setZero();
    p_hat.resize(totalRows);
    p_hat.setZero();
    dot_p_hat.resize(totalRows);
    dot_p_hat.setZero();
    torque_ext_hat.setZero();
    // force_ext_true.resize(6);
    // force_ext_true.setZero();
    Gain_K1.resize(totalRows, totalRows);
    Gain_K2.resize(totalRows, totalRows);
    Gain_K3.resize(totalRows, totalRows);
    Gain_K4.resize(totalRows, totalRows);
    Gain_K1.setZero();
    Gain_K2.setZero();
    Gain_K3.setZero();
    Gain_K4.setZero();

    //ROS_INFO_STREAM("\033[32m sizeMBOgainRows:"<< totalRows << "\033[0m");
    // 初始化 S 矩阵  
    selectMatrix.resize(info_.actuatedDofNum, info_.generalizedCoordinatesNum);  // 初始化选择矩阵的维度
    matrix_t I = matrix_t::Identity(info_.actuatedDofNum, info_.actuatedDofNum);
    matrix_t zeroMatrix  = matrix_t::Zero(info_.actuatedDofNum, 6);
    selectMatrix << zeroMatrix, I;

    ros::NodeHandle nh;
    armTau2hat_pub_= nh.advertise<std_msgs::Float64>("/STAestimation/armjoint2" , 1);
    armTau3hat_pub_= nh.advertise<std_msgs::Float64>("/STAestimation/armjoint3" , 1);
    baseXhat_pub_= nh.advertise<std_msgs::Float64>("/STAestimation/baseX" , 1);
    baseYhat_pub_= nh.advertise<std_msgs::Float64>("/STAestimation/baseY" , 1);
    extForcehatAbs_pub_ = nh.advertise<std_msgs::Float64>("/STAestimation/extForceAbs_hat" , 1);
    //extForceAbs_pub_ = nh.advertise<std_msgs::Float64>("/MBOestimation/extForceAbs_true" , 1);
}

void STA::dynamicCallback(qm_force_observer::ObserverConfig &config, uint32_t) {
    scalar_t Angle_K1;
    Angle_K1 = config.STA_ang_k1;
    scalar_t Angle_K2;
    Angle_K2 = config.STA_ang_k2;
    scalar_t Angle_K3;
    Angle_K3 = config.STA_ang_k3;
    scalar_t Angle_K4;
    Angle_K4 = config.STA_ang_k4;
    
    scalar_t Linear_K1;
    Linear_K1 = config.STA_lin_k1;
    scalar_t Linear_K2;
    Linear_K2 = config.STA_lin_k2;
    scalar_t Linear_K3;
    Linear_K3 = config.STA_lin_k3;
    scalar_t Linear_K4;
    Linear_K4 = config.STA_lin_k4;

    scalar_t Leg_K1;
    Leg_K1 = config.STA_leg_k1;
    scalar_t Leg_K2;
    Leg_K2 = config.STA_leg_k2;
    scalar_t Leg_K3;
    Leg_K3 = config.STA_leg_k3;
    scalar_t Leg_K4;
    Leg_K4 = config.STA_leg_k4;

    scalar_t Arm_K1;
    Arm_K1 = config.STA_arm_k1;
    scalar_t Arm_K2;
    Arm_K2 = config.STA_arm_k2;
    scalar_t Arm_K3;
    Arm_K3 = config.STA_arm_k3;
    scalar_t Arm_K4;
    Arm_K4 = config.STA_arm_k4;


    setParam(Angle_K1, Angle_K2, Angle_K3, Angle_K4, Linear_K1, Linear_K2, Linear_K3, Linear_K4, Leg_K1, Leg_K2, Leg_K3, Leg_K4, Arm_K1, Arm_K2, Arm_K3, Arm_K4);
    ROS_INFO_STREAM("\033[32m Update the STA Force Observer Param. \033[0m");
}

vector_t STA::getExternalTorque(const vector_t& rbdStateMeasured, scalar_t time, scalar_t period){
    // 得到四足机械臂当前的测量状态

    //ROS_INFO_STREAM("\033[32m STA running. \033[0m");

    qMeasured_.setZero();
    vMeasured_.setZero();

    qMeasured_.head<3>() = rbdStateMeasured.segment<3>(3);
    qMeasured_.segment<3>(3) = rbdStateMeasured.head<3>();  // base欧拉角  
    qMeasured_.tail(info_.actuatedDofNum) = rbdStateMeasured.segment(6, info_.actuatedDofNum);
    vMeasured_.head<3>() = rbdStateMeasured.segment<3>(info_.generalizedCoordinatesNum + 3);
    vMeasured_.segment<3>(3) = getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<scalar_t>(
            qMeasured_.segment<3>(3), rbdStateMeasured.segment<3>(info_.generalizedCoordinatesNum));   // 这里是base欧拉角的导数而不是角速度
    vMeasured_.tail(info_.actuatedDofNum) = rbdStateMeasured.segment(info_.generalizedCoordinatesNum + 6, info_.actuatedDofNum);



    // 取出四足机械臂的各项动力学矩阵
    const auto& model = pinocchioInterface_.getModel();
    auto& data = pinocchioInterface_.getData();

    pinocchio::forwardKinematics(model, data, qMeasured_, vMeasured_);
    pinocchio::computeJointJacobians(model, data);
    pinocchio::updateFramePlacements(model, data);
    pinocchio::crba(model, data, qMeasured_);
    pinocchio::computeJointJacobiansTimeVariation(model, data, qMeasured_, vMeasured_);

    data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();

    pinocchio::nonLinearEffects(model, data, qMeasured_, vMeasured_);
    pinocchio::computeCoriolisMatrix(model, data, qMeasured_, vMeasured_);

    pinocchio::computeGeneralizedGravity(model, data, qMeasured_);

    M = data.M;
    C = data.C;
    G = data.g;

    // 得到机械臂末端操作空间雅可比矩阵
    arm_j_.setZero();
    arm_dj_.setZero(); // must
    size_t armEeFrameIdx = model.getBodyId(armEeKinematics_->getIds()[0]);
    pinocchio::getFrameJacobian(model, data, armEeFrameIdx, pinocchio::LOCAL_WORLD_ALIGNED, arm_j_);
    pinocchio::getFrameJacobianTimeVariation(model, data, armEeFrameIdx, pinocchio::LOCAL_WORLD_ALIGNED, arm_dj_);

    // 求arm_j_的伪逆
    Arm_J_PseudoInverse = (arm_j_.transpose()).completeOrthogonalDecomposition().pseudoInverse();

    // 得到足端的雅可比矩阵
    j_ = matrix_t(3 * info_.numThreeDofContacts, info_.generalizedCoordinatesNum);
    for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
        Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> jac;
        jac.setZero(6, info_.generalizedCoordinatesNum);
        pinocchio::getFrameJacobian(model, data, info_.endEffectorFrameIndices[i], pinocchio::LOCAL_WORLD_ALIGNED, jac);
        j_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) = jac.template topRows<3>();     //取jac的线速度部分
    }

    // ETH论文公式（3）
    matrix_t j_transposed(info_.generalizedCoordinatesNum, info_.numThreeDofContacts * 3);
    // 将每个足底的雅可比矩阵转置并平铺
    for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
        // 获取每个足底的雅可比矩阵的转置
        Eigen::Matrix<scalar_t, Eigen::Dynamic, Eigen::Dynamic> jac_transposed = j_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum).transpose();
        
        // 平铺到新矩阵中
        j_transposed.block(0, i * 3, info_.generalizedCoordinatesNum, 3) = jac_transposed;
    }
    matrix_t final_j_transposed(info_.generalizedCoordinatesNum, info_.numThreeDofContacts * 3 + 6);
    final_j_transposed.block(0, 0, info_.generalizedCoordinatesNum, info_.numThreeDofContacts * 3) = j_transposed;
    final_j_transposed.block(0, info_.numThreeDofContacts * 3, info_.generalizedCoordinatesNum, 6) = arm_j_.transpose();
    // 求拼接后的大转置矩阵的伪逆
    J_T_PseudoInverse = final_j_transposed.completeOrthogonalDecomposition().pseudoInverse();



    matrix_t Jea_T;
    matrix_t Jeb_T;
    vector6_t force_ext_hat;
    Jea_T = arm_j_.transpose().bottomRows(6);
    Jeb_T = arm_j_.transpose().topRows(6);
    //得到当前当前各关节力矩  refer to 《Collision detection and identification for a legged manipulator》
    jointTorque = rbdStateMeasured.segment(2 * generalizedCoordinatesNum_ + 7 + 3, info_.actuatedDofNum);  // 实际测量关节力矩
    generalizedTorque = selectMatrix.transpose() * jointTorque;


    T = period;
    p = M * qMeasured_;  // 实际动量
    // // MBO观测器
    // //p = pinocchio::cholesky::Mv(model, data, qMeasured_); // 利用惯性矩阵的稀疏性性质加速计算动量
    // dot_p_hat =  generalizedTorque + r + C.transpose() * vMeasured_ - G;

    // p_hat = p_hat + T * dot_p_hat;
    // //r = gainMBO * (p - p_hat);
    // torque_ext_hat = r;

    // STA 观测器
    dot_r = Gain_K3 * (p - p_hat)/(p - p_hat).norm() + Gain_K4 * (p - p_hat);
    r = r + T * dot_r;
    dot_p_hat = generalizedTorque + C.transpose() * vMeasured_ - G + Gain_K1 * (p - p_hat)/sqrt((p - p_hat).norm()) + Gain_K2 * (p - p_hat) + r;
    p_hat = p_hat + T * dot_p_hat;
    torque_ext_hat = r;
    
    // force_ext_hat = Arm_J_PseudoInverse * torque_ext_hat;
    // norm_force_ext_hat = force_ext_hat.head<3>().norm();
    // ROS_INFO_STREAM("\033[32m norm of external force:"<< norm_force_ext_hat << "\033[0m");

    // ETH论文公式（3）
    hat_ExternalWrenchs = J_T_PseudoInverse * torque_ext_hat;
    force_ext_hat = hat_ExternalWrenchs.block(info_.numThreeDofContacts * 3, 0, 6, 1);
    norm_force_ext_hat = force_ext_hat.head<3>().norm();
    //ROS_INFO_STREAM("\033[32m norm of external force:"<< norm_force_ext_hat << "\033[0m");
    std_msgs::Float64 absForce_msg1;
    absForce_msg1.data = norm_force_ext_hat;
    extForcehatAbs_pub_.publish(absForce_msg1);

    // // groundtruth abs external force
    // force_ext_true.head(3) = rbdStateMeasured.segment<3>(2 * info_.generalizedCoordinatesNum + 7);
    // norm_force_ext_true = force_ext_true.head<3>().norm();
    // std_msgs::Float64 absForce_msg2;
    // absForce_msg2.data = norm_force_ext_true;
    // extForceAbs_pub_.publish(absForce_msg2);

    // 利用计算得到的外力得到base方向上的外力矩
    base_torque_ext_hat = Jeb_T * force_ext_hat;

    //ROS_INFO_STREAM("\033[32m STA get torque_ext_hat. \033[0m");

    std_msgs::Float64 tau_msg1;
    tau_msg1.data = torque_ext_hat(19);
    armTau2hat_pub_.publish(tau_msg1);

    std_msgs::Float64 tau_msg2;
    tau_msg2.data = torque_ext_hat(20);
    armTau3hat_pub_.publish(tau_msg2);

    std_msgs::Float64 tau_msg3;
    //tau_msg3.data = torque_ext_hat(0);
    tau_msg3.data = base_torque_ext_hat(0);


    baseXhat_pub_.publish(tau_msg3);

    std_msgs::Float64 tau_msg4;
    //tau_msg4.data = torque_ext_hat(1);
    tau_msg4.data = base_torque_ext_hat(1);

    baseYhat_pub_.publish(tau_msg4);

    return torque_ext_hat;
}


void STA::setParam(scalar_t K1_ang, scalar_t K2_ang, scalar_t K3_ang, scalar_t K4_ang,
                    scalar_t K1_lin, scalar_t K2_lin,scalar_t K3_lin,scalar_t K4_lin,
                    scalar_t K1_leg, scalar_t K2_leg,scalar_t K3_leg,scalar_t K4_leg,
                    scalar_t K1_arm, scalar_t K2_arm, scalar_t K3_arm, scalar_t K4_arm){
    Angle_K1.diagonal() << K1_ang, K1_ang, K1_ang;
    Angle_K2.diagonal() << K2_ang, K2_ang, K2_ang;
    Angle_K3.diagonal() << K3_ang, K3_ang, K3_ang;
    Angle_K4.diagonal() << K4_ang, K4_ang, K4_ang;

    Linear_K1.diagonal() << K1_lin, K1_lin, K1_lin;
    Linear_K2.diagonal() << K2_lin, K2_lin, K2_lin;
    Linear_K3.diagonal() << K3_lin, K3_lin, K3_lin;
    Linear_K4.diagonal() << K4_lin, K4_lin, K4_lin;

    Leg_K1.diagonal() << K1_leg, K1_leg, K1_leg, K1_leg, K1_leg, K1_leg, K1_leg, K1_leg, K1_leg, K1_leg, K1_leg, K1_leg;
    Leg_K2.diagonal() << K2_leg, K2_leg, K2_leg, K2_leg, K2_leg, K2_leg, K2_leg, K2_leg, K2_leg, K2_leg, K2_leg, K2_leg;
    Leg_K3.diagonal() << K3_leg, K3_leg, K3_leg, K3_leg, K3_leg, K3_leg, K3_leg, K3_leg, K3_leg, K3_leg, K3_leg, K3_leg;
    Leg_K4.diagonal() << K4_leg, K4_leg, K4_leg, K4_leg, K4_leg, K4_leg, K4_leg, K4_leg, K4_leg, K4_leg, K4_leg, K4_leg;

    Arm_K1.diagonal() << K1_arm, K1_arm, K1_arm, K1_arm, K1_arm, K1_arm;
    Arm_K2.diagonal() << K2_arm, K2_arm, K2_arm, K2_arm, K2_arm, K2_arm;
    Arm_K3.diagonal() << K3_arm, K3_arm, K3_arm, K3_arm, K3_arm, K3_arm;
    Arm_K4.diagonal() << K4_arm, K4_arm, K4_arm, K4_arm, K4_arm, K4_arm;

    // 组合为最终的增益矩阵
    Gain_K1.block(0, 0, Angle_K1.rows(), Angle_K1.cols()) = Angle_K1;
    Gain_K1.block(Angle_K1.rows(), Angle_K1.cols(), Linear_K1.rows(), Linear_K1.cols()) = Linear_K1;
    Gain_K1.block(Angle_K1.rows() + Linear_K1.rows(), Angle_K1.cols() + Linear_K1.cols(), Leg_K1.rows(), Leg_K1.cols()) = Leg_K1;
    Gain_K1.block(Angle_K1.rows() + Linear_K1.rows() + Leg_K1.rows(), Angle_K1.cols() + Linear_K1.cols() + Leg_K1.cols(), Arm_K1.rows(), Arm_K1.cols()) = Arm_K1;

    Gain_K2.block(0, 0, Angle_K1.rows(), Angle_K1.cols()) = Angle_K2;
    Gain_K2.block(Angle_K1.rows(), Angle_K1.cols(), Linear_K1.rows(), Linear_K1.cols()) = Linear_K2;
    Gain_K2.block(Angle_K1.rows() + Linear_K1.rows(), Angle_K1.cols() + Linear_K1.cols(), Leg_K1.rows(), Leg_K1.cols()) = Leg_K2;
    Gain_K2.block(Angle_K1.rows() + Linear_K1.rows() + Leg_K1.rows(), Angle_K1.cols() + Linear_K1.cols() + Leg_K1.cols(), Arm_K1.rows(), Arm_K1.cols()) = Arm_K2;

    Gain_K3.block(0, 0, Angle_K1.rows(), Angle_K1.cols()) = Angle_K3;
    Gain_K3.block(Angle_K1.rows(), Angle_K1.cols(), Linear_K1.rows(), Linear_K1.cols()) = Linear_K3;
    Gain_K3.block(Angle_K1.rows() + Linear_K1.rows(), Angle_K1.cols() + Linear_K1.cols(), Leg_K1.rows(), Leg_K1.cols()) = Leg_K3;
    Gain_K3.block(Angle_K1.rows() + Linear_K1.rows() + Leg_K1.rows(), Angle_K1.cols() + Linear_K1.cols() + Leg_K1.cols(), Arm_K1.rows(), Arm_K1.cols()) = Arm_K3;

    Gain_K4.block(0, 0, Angle_K1.rows(), Angle_K1.cols()) = Angle_K4;
    Gain_K4.block(Angle_K1.rows(), Angle_K1.cols(), Linear_K1.rows(), Linear_K1.cols()) = Linear_K4;
    Gain_K4.block(Angle_K1.rows() + Linear_K1.rows(), Angle_K1.cols() + Linear_K1.cols(), Leg_K1.rows(), Leg_K1.cols()) = Leg_K4;
    Gain_K4.block(Angle_K1.rows() + Linear_K1.rows() + Leg_K1.rows(), Angle_K1.cols() + Linear_K1.cols() + Leg_K1.cols(), Arm_K1.rows(), Arm_K1.cols()) = Arm_K4;
}



}
