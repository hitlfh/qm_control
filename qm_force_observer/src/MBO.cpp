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
#include "qm_force_observer/MBO.h"
#include <std_msgs/Float64.h>

namespace qm{
using namespace ocs2;
MBO::MBO(const PinocchioInterface &pinocchioInterface, CentroidalModelInfo info,
                             const PinocchioEndEffectorKinematics &armEeKinematics, ros::NodeHandle &controller_nh)
     : pinocchioInterface_(pinocchioInterface),
       info_(std::move(info)),
       mapping_(info_),
       armEeKinematics_(armEeKinematics.clone())
{
    qMeasured_ = vector_t(info_.generalizedCoordinatesNum);
    vMeasured_ = vector_t(info_.generalizedCoordinatesNum);
    vMeasured_noise = vector_t(info_.generalizedCoordinatesNum);
    arm_j_ = matrix_t(6, info_.generalizedCoordinatesNum);
    arm_dj_ = matrix_t(6, info_.generalizedCoordinatesNum);
    base_j_ = matrix_t(6, info_.generalizedCoordinatesNum);
    base_dj_ = matrix_t(6, info_.generalizedCoordinatesNum);
    G = vector_t(info_.generalizedCoordinatesNum);

    generalizedCoordinatesNum_ = info_.generalizedCoordinatesNum;
    actuatedDofNum_ = info_.actuatedDofNum;
    initParam();

    ROS_INFO_STREAM("\033[32m MBO initialized\033[0m");

    // dynamic reconfigure
    ros::NodeHandle nh_weight = ros::NodeHandle(controller_nh,"observer");
    dynamic_srv_ = std::make_shared<dynamic_reconfigure::Server<qm_force_observer::ObserverConfig>>(nh_weight);
    dynamic_reconfigure::Server<qm_force_observer::ObserverConfig>::CallbackType cb = [this](auto&& PH1, auto&& PH2) {
        dynamicCallback(std::forward<decltype(PH1)>(PH1), std::forward<decltype(PH2)>(PH2));
    };
    dynamic_srv_->setCallback(cb);
}


// 初始化各种参数
void MBO::initParam(){
    gainAngle.resize(3, 3);
    gainAngle.setZero();
    gainLinear.resize(3, 3);
    gainLinear.setZero();
    gainLeg.resize(12, 12);
    gainLeg.setZero();
    gainArm.resize(6, 6);
    gainArm.setZero();
    
    dot_r.setZero();
    int totalRows = gainAngle.rows() + gainLinear.rows() + gainLeg.rows() + gainArm.rows();
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
    gainMBO.resize(totalRows, totalRows);
    gainMBO.setZero();
    //ROS_INFO_STREAM("\033[32m sizeMBOgainRows:"<< totalRows << "\033[0m");
    // 初始化 S 矩阵  
    selectMatrix.resize(info_.actuatedDofNum, info_.generalizedCoordinatesNum);  // 初始化选择矩阵的维度
    matrix_t I = matrix_t::Identity(info_.actuatedDofNum, info_.actuatedDofNum);
    matrix_t zeroMatrix  = matrix_t::Zero(info_.actuatedDofNum, 6);
    selectMatrix << zeroMatrix, I;

    // 初始化碰撞判断阈值
    th_joint2 = 0;
    th_joint3 = 0;
    th_baseX = 0;
    th_baseY = 0;
    th_collision = 0;
    force_isolation = 0;

    // 初始化标志位
    flag_noise = 0;
    flag_uncertainty_M = 0;
    flag_uncertainty_C = 0;
    flag_uncertainty_G = 0;

    ros::NodeHandle nh;
    armTau2hat_pub_= nh.advertise<std_msgs::Float64>("/MBOestimation/armjoint2" , 1);
    armTau3hat_pub_= nh.advertise<std_msgs::Float64>("/MBOestimation/armjoint3" , 1);
    baseXhat_pub_= nh.advertise<std_msgs::Float64>("/MBOestimation/baseX" , 1);
    baseYhat_pub_= nh.advertise<std_msgs::Float64>("/MBOestimation/baseY" , 1);
    extForcehatAbs_EE_pub_ = nh.advertise<std_msgs::Float64>("/MBOestimation/extForceAbs_EE_hat" , 1);
    extForcehatAbs_base_pub_ = nh.advertise<std_msgs::Float64>("/MBOestimation/extForceAbs_base_hat" , 1);
    force_isolation_pub_ = nh.advertise<std_msgs::Float64>("/MBOestimation/force_isolation" , 1);
    //extForceAbs_pub_ = nh.advertise<std_msgs::Float64>("/MBOestimation/extForceAbs_true" , 1);
}

void MBO::dynamicCallback(qm_force_observer::ObserverConfig &config, uint32_t) {
    scalar_t Ko_ang;
    scalar_t Ko_lin;
    scalar_t Ko_leg;
    scalar_t Ko_arm;
    Ko_ang = config.MBO_ang;
    Ko_lin = config.MBO_lin;
    Ko_leg = config.MBO_leg;
    Ko_arm = config.MBO_arm;
    setParam(Ko_ang, Ko_lin, Ko_leg, Ko_arm);

    flag_noise = config.MBO_noise_flag;
    flag_uncertainty_M = config.MBO_uncertainty_M_flag;
    flag_uncertainty_C = config.MBO_uncertainty_C_flag;
    flag_uncertainty_G = config.MBO_uncertainty_G_flag;
    ROS_INFO_STREAM("\033[32m Update the MBO Force Observer Param. \033[0m");
}

vector_t MBO::getExternalTorque(const vector_t& rbdStateMeasured, scalar_t time, scalar_t period){
    // 得到四足机械臂当前的测量状态

    //ROS_INFO_STREAM("\033[32m MBO running. \033[0m");

    qMeasured_.setZero();
    vMeasured_.setZero();
    vMeasured_noise.setZero();

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

    // 得到base的雅可比矩阵
    base_j_.setZero();
    base_dj_.setZero(); // must
    pinocchio::getFrameJacobian(model, data, model.getBodyId("base"), pinocchio::LOCAL_WORLD_ALIGNED, base_j_);
    pinocchio::getFrameJacobianTimeVariation(model, data, model.getBodyId("base"), pinocchio::LOCAL_WORLD_ALIGNED, base_dj_);

    // 得到足端的雅可比矩阵
    j_ = matrix_t(3 * info_.numThreeDofContacts, info_.generalizedCoordinatesNum);
    for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
        Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> jac;
        jac.setZero(6, info_.generalizedCoordinatesNum);
        pinocchio::getFrameJacobian(model, data, info_.endEffectorFrameIndices[i], pinocchio::LOCAL_WORLD_ALIGNED, jac);
        j_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) = jac.template topRows<3>();     //取jac的线速度部分
    }

    // ETH论文公式（3）  _EE后缀代表施力连杆是机械臂末端，_base后缀代表施加力连杆是base
    matrix_t j_transposed(info_.generalizedCoordinatesNum, info_.numThreeDofContacts * 3);
    // 将每个足底的雅可比矩阵转置并平铺
    for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
        // 获取每个足底的雅可比矩阵的转置
        Eigen::Matrix<scalar_t, Eigen::Dynamic, Eigen::Dynamic> jac_transposed = j_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum).transpose();
        
        // 平铺到新矩阵中
        j_transposed.block(0, i * 3, info_.generalizedCoordinatesNum, 3) = jac_transposed;
    }
    // EE部分
    matrix_t final_j_transposed_EE(info_.generalizedCoordinatesNum, info_.numThreeDofContacts * 3 + 6);
    final_j_transposed_EE.block(0, 0, info_.generalizedCoordinatesNum, info_.numThreeDofContacts * 3) = j_transposed;
    final_j_transposed_EE.block(0, info_.numThreeDofContacts * 3, info_.generalizedCoordinatesNum, 6) = arm_j_.transpose();
    // base 部分
    matrix_t final_j_transposed_base(info_.generalizedCoordinatesNum, info_.numThreeDofContacts * 3 + 6);
    final_j_transposed_base.block(0, 0, info_.generalizedCoordinatesNum, info_.numThreeDofContacts * 3) = j_transposed;
    final_j_transposed_base.block(0, info_.numThreeDofContacts * 3, info_.generalizedCoordinatesNum, 6) = base_j_.transpose();
    // 求拼接后的大转置矩阵的伪逆
    J_T_PseudoInverse_EE = final_j_transposed_EE.completeOrthogonalDecomposition().pseudoInverse();
    J_T_PseudoInverse_base = final_j_transposed_base.completeOrthogonalDecomposition().pseudoInverse();

    //ROS_INFO_STREAM("\033[32m MBO inverse gotten \033[0m");

    matrix_t Jea_T;
    matrix_t Jeb_T;
    Jea_T = arm_j_.transpose().bottomRows(6);
    Jeb_T = arm_j_.transpose().topRows(6);
    //得到当前当前各关节力矩  refer to 《Collision detection and identification for a legged manipulator》
    jointTorque = rbdStateMeasured.segment(2 * generalizedCoordinatesNum_ + 7 + 3, info_.actuatedDofNum);  // 实际测量关节力矩
    generalizedTorque = selectMatrix.transpose() * jointTorque;

    // // debug
    // size_t sizeJointTorque = jointTorque.rows();
    // size_t sizegeneralizedTorque = generalizedTorque.rows();
    // size_t sizevMeasured_ = vMeasured_.rows();
    // size_t sizeG = G.rows();
    // size_t sizeCrows = C.rows();
    // size_t sizeCcols = C.cols();

    // ROS_INFO_STREAM("\033[32m sizeJointTorque:"<< sizeJointTorque << "\033[0m");
    // ROS_INFO_STREAM("\033[32m sizegeneralizedTorque:"<< sizegeneralizedTorque << "\033[0m");
    // ROS_INFO_STREAM("\033[32m sizevMeasured_:"<< sizevMeasured_ << "\033[0m");
    // ROS_INFO_STREAM("\033[32m sizeG:"<< sizeG << "\033[0m");
    // ROS_INFO_STREAM("\033[32m sizeCrows:"<< sizeCrows << "\033[0m");
    // ROS_INFO_STREAM("\033[32m sizeCrows:"<< sizeCcols << "\033[0m");

    // ROS_INFO_STREAM("\033[32m MBO get generalizedTorque. \033[0m");
    //rbdState_.segment(2 * generalizedCoordinatesNum_ + 7 + 3, info_.actuatedDofNum) = jointTor;
    // MBO观测器
    T = period;
    //p = M * vMeasured_;  // 实际动量

    // 根据标志位决定动力学矩阵是否需要包含不确定性
    //M = M + flag_uncertainty_M * 0.7 * M;
    M = M + flag_uncertainty_M * 2 * M;
    C = C + flag_uncertainty_C * 1 * C;
    G = G + flag_uncertainty_G * 0.2 * G;
    //ROS_INFO_STREAM("\033[32m MBO M flag" << flag_uncertainty_M << "\033[0m");

    std::random_device rd;  // 随机数种子
    std::mt19937 gen(rd());  // 使用梅森旋转算法作为随机数生成器
    std::normal_distribution<> d(0.0, 0.01);  // 均值为0，标准差为0.01的高斯分布
    for (int i = 0; i < vMeasured_.size(); ++i) {
        vMeasured_noise[i] = vMeasured_[i] +  d(gen);  // 为每个元素加上噪声
    }
    //p = M * vMeasured_noise;  // 实际动量（含有噪声）
    vMeasured_ = (1-flag_noise) * vMeasured_+ flag_noise * vMeasured_noise;
    p = M * vMeasured_; // 实际动量

    // ROS_INFO_STREAM("\033[32m MBO get actual momentum. \033[0m");

    // p = pinocchio::cholesky::Mv(model, data, qMeasured_); // 利用惯性矩阵的稀疏性性质加速计算动量
    dot_p_hat = generalizedTorque + r + C.transpose() * vMeasured_ - G;

    // ROS_INFO_STREAM("\033[32m MBO get actual dot_p_hat. \033[0m");

    p_hat = p_hat + T * dot_p_hat;
    r = gainMBO * (p - p_hat);
    torque_ext_hat = r;
    

    // ETH论文公式（3）
    // 力施加在EE上的情况
    hat_ExternalWrenchs_EE = J_T_PseudoInverse_EE * torque_ext_hat;
    force_ext_hat_EE = hat_ExternalWrenchs_EE.block(info_.numThreeDofContacts * 3, 0, 6, 1);
    norm_force_ext_hat_EE = force_ext_hat_EE.head<3>().norm();
    //ROS_INFO_STREAM("\033[32m norm of external force:"<< norm_force_ext_hat_EE << "\033[0m");
    std_msgs::Float64 absForce_msg1;
    absForce_msg1.data = norm_force_ext_hat_EE;
    extForcehatAbs_EE_pub_.publish(absForce_msg1);

    // 利用计算得到的机械臂末端外力得到base方向上的外力矩
    base_torque_ext_hat = Jeb_T * force_ext_hat_EE;
    
    // 力施加在base上的情况
    hat_ExternalWrenchs_base = J_T_PseudoInverse_base * torque_ext_hat;
    force_ext_hat_base = hat_ExternalWrenchs_base.block(info_.numThreeDofContacts * 3, 0, 6, 1);
    norm_force_ext_hat_base = force_ext_hat_base.head<3>().norm();
    //ROS_INFO_STREAM("\033[32m norm of external force:"<< norm_force_ext_hat_base << "\033[0m");
    std_msgs::Float64 absForce_msg2;
    absForce_msg2.data = norm_force_ext_hat_base;
    extForcehatAbs_base_pub_.publish(absForce_msg2);
    
    th_joint2 = 0.5;
    th_joint3 = 0.5;
    th_baseX = 3;
    th_baseY = 3;
    th_collision = 5;


    // 力施加位置判断（施加在机械臂末端还是base上（为arm的joint力矩设置阈值，阈值设置需要考虑到有噪声的情况），发布消息：EE=1，base = -1）
    if(norm_force_ext_hat_EE > th_collision || norm_force_ext_hat_base > th_collision)
    {
        //发生了碰撞

        // if (torque_ext_hat(19) > th_joint2 || torque_ext_hat(20) > th_joint3)
        // {
        //     force_isolation = 1;
        // }
        // else if (base_torque_ext_hat(0) > th_baseX || base_torque_ext_hat(0) > th_baseY)
        // {
        //     force_isolation = -1;
        // }
        // else
        // {
        //     force_isolation =0;
        // }
        if (torque_ext_hat(19) < th_joint2 && torque_ext_hat(20) < th_joint3 && (base_torque_ext_hat(0) > th_baseX || base_torque_ext_hat(0) > th_baseY))
        {
            force_isolation = -1;
        }
        else if (torque_ext_hat(19) > th_joint2 || torque_ext_hat(20) > th_joint3)
        {
            force_isolation = 1;
        }
        else
        {
            force_isolation = 0;
        }
    }
    else
    {
        force_isolation = 0;
    }

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

    std_msgs::Float64 tau_msg5;
    tau_msg5.data = force_isolation;
    force_isolation_pub_.publish(tau_msg5);

    return torque_ext_hat;
}


void MBO::setParam(scalar_t Ko_ang, scalar_t Ko_lin, scalar_t Ko_leg, scalar_t Ko_arm){
    gainAngle.diagonal() << Ko_ang, Ko_ang, Ko_ang;
    gainLinear.diagonal() << Ko_lin, Ko_lin, Ko_lin;
    gainLeg.diagonal() << Ko_leg, Ko_leg, Ko_leg, Ko_leg, Ko_leg, Ko_leg, Ko_leg, Ko_leg, Ko_leg, Ko_leg, Ko_leg, Ko_leg;
    gainArm.diagonal() << Ko_arm, Ko_arm, Ko_arm, Ko_arm, Ko_arm, Ko_arm;

    // 组合为最终的增益矩阵Ko
    gainMBO.block(0, 0, gainAngle.rows(), gainAngle.cols()) = gainAngle;
    gainMBO.block(gainAngle.rows(), gainAngle.cols(), gainLinear.rows(), gainLinear.cols()) = gainLinear;
    gainMBO.block(gainAngle.rows() + gainLinear.rows(), gainAngle.cols() + gainLinear.cols(), gainLeg.rows(), gainLeg.cols()) = gainLeg;
    gainMBO.block(gainAngle.rows() + gainLinear.rows() + gainLeg.rows(), gainAngle.cols() + gainLinear.cols() + gainLeg.cols(), gainArm.rows(), gainArm.cols()) = gainArm;
}



}
