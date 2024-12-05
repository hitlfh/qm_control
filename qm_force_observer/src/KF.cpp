//
// Created by lfh on 2024/11/25.
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
#include "qm_force_observer/KF.h"
#include <std_msgs/Float64.h>
#include <chrono>  //计时功能头文件

namespace qm{
using namespace ocs2;
KF::KF(const PinocchioInterface &pinocchioInterface, CentroidalModelInfo info,
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

    ROS_INFO_STREAM("\033[32m KF initialized\033[0m");

    // dynamic reconfigure
    ros::NodeHandle nh_weight = ros::NodeHandle(controller_nh,"KF_observer");
    dynamic_srv_ = std::make_shared<dynamic_reconfigure::Server<qm_force_observer::Momentum_KFConfig>>(nh_weight);
    dynamic_reconfigure::Server<qm_force_observer::Momentum_KFConfig>::CallbackType cb = [this](auto&& PH1, auto&& PH2) {
        dynamicCallback(std::forward<decltype(PH1)>(PH1), std::forward<decltype(PH2)>(PH2));
    };
    dynamic_srv_->setCallback(cb);
}


// 初始化各种参数
void KF::initParam(){
    Qcp_ang.resize(3,3);
    Qcp_ang.setZero();
    Qcp_Linear.resize(3,3);
    Qcp_Linear.setZero();
    Qcp_Leg.resize(12,12);
    Qcp_Leg.setZero();
    Qcp_Arm.resize(6,6);
    Qcp_Arm.setZero();
    Qcp.resize(24,24);
    Qcp.setZero();
    Qcf.resize(24,24);
    Qcf.setZero();
    Qc.resize(48,48);
    Qc.setZero();

    Rc_ang.resize(3,3);
    Rc_ang.setZero();
    Rc_Linear.resize(3,3);
    Rc_Linear.setZero();
    Rc_Leg.resize(12,12);
    Rc_Leg.setZero();
    Rc_Arm.resize(6,6);
    Rc_Arm.setZero();
    Rc.resize(24,24);
    Rc.setZero();

    A_f.resize(24,24);
    A_f.setZero();

    Ac.resize(48, 48);
    Ac.setZero();
    Ad.resize(48, 48);
    Ad.setZero();
    Bc.resize(48, 24);
    Bc.setZero();
    Bd.resize(48, 24);
    Bd.setZero();
    Cc.resize(24, 48);
    Cc.setZero();
    Cd.resize(24, 48);
    Cd.setZero();

    mat_zero1.resize(24, 24);
    mat_zero1.setZero();
    mat_zero2.resize(24, 24);
    mat_zero2.setZero();
    mat_zero3.resize(24, 24);
    mat_zero3.setZero();
    mat_zero4.resize(24, 48);
    mat_zero4.setZero();
    mat_zero5.resize(48, 48);
    mat_zero5.setZero();
    mat_indentity1.resize(24, 24);
    mat_indentity1.setIdentity();
    mat_indentity2.resize(48, 48);
    mat_indentity2.setIdentity();
    mat_indentity3.resize(72, 72);
    mat_indentity3.setIdentity();
    mat_indentity4.resize(96, 96);
    mat_indentity4.setIdentity();

    Rd.resize(24, 24);
    Rd.setZero();
    Qd.resize(48, 48);
    Qd.setZero();

    Mat_d_1.resize(72, 72);
    Mat_d_1.setZero();
    Mat_c_1.resize(72, 72);
    Mat_c_1.setZero();
    Mat_d_2.resize(96, 96);
    Mat_d_2.setZero();
    Mat_c_2.resize(96, 96);
    Mat_c_2.setZero();
    M_11.resize(48, 48);
    M_11.setZero();
    M_12.resize(48, 48);
    M_12.setZero();
    H.resize(96, 96);
    H.setZero();


    x_pred.resize(48);
    x_pred.setZero();
    x_hat_KF.resize(48);
    x_hat_KF.setZero();
    P_pred.resize(48,48);
    P_pred.setIdentity();  // 初始化协方差矩阵
    P_pred = 0.1 * P_pred;
    P_KF.resize(48,48);
    P_KF.setIdentity();
    P_KF = 0.1 * P_KF;
    tau_bar.resize(24);
    K_KF.resize(48, 24);
    K_KF.setZero();

    force_ext_hat_EE.resize(6);
    force_ext_hat_EE.setZero();


    p.resize(24);
    p.setZero();
    dot_p.resize(24);
    dot_p.setZero();
    p_hat.resize(24);
    p_hat.setZero();
    dot_p_hat.resize(24);
    dot_p_hat.setZero();
    torque_ext_hat.setZero();
    torque_dis_hat.setZero();
    // force_ext_true.resize(6);
    // force_ext_true.setZero();
    //ROS_INFO_STREAM("\033[32m sizeMBOgainRows:"<< 24 << "\033[0m");
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
    armTau2hat_pub_= nh.advertise<std_msgs::Float64>("/KFestimation/armjoint2" , 1);
    armTau3hat_pub_= nh.advertise<std_msgs::Float64>("/KFestimation/armjoint3" , 1);
    baseXhat_pub_= nh.advertise<std_msgs::Float64>("/KFestimation/baseX" , 1);
    baseYhat_pub_= nh.advertise<std_msgs::Float64>("/KFestimation/baseY" , 1);
    extForcehatAbs_EE_pub_ = nh.advertise<std_msgs::Float64>("/KFestimation/extForceAbs_EE_hat" , 1);
    extForcehatAbs_base_pub_ = nh.advertise<std_msgs::Float64>("/KFestimation/extForceAbs_base_hat" , 1);
    force_isolation_pub_ = nh.advertise<std_msgs::Float64>("/KFestimation/force_isolation" , 1);

    tau_ext_baseXhat_pub_ = nh.advertise<std_msgs::Float64>("/KFestimation/tau_ext_baseX", 1);
    tau_ext_baseYhat_pub_ = nh.advertise<std_msgs::Float64>("/KFestimation/tau_ext_baseY", 1);  // 发布直接从估计到的外力矩中提取的base外力矩分量

    baseforceXhat_pub_ = nh.advertise<std_msgs::Float64>("/KFestimation/force_onBase_baseX", 1); 
    baseforceYhat_pub_ = nh.advertise<std_msgs::Float64>("/KFestimation/force_onBase_baseY", 1);
}

void KF::dynamicCallback(qm_force_observer::Momentum_KFConfig &config, uint32_t) {
    scalar_t Qcp_angle_;
    scalar_t Qcp_lin_;
    scalar_t Qcp_leg_;
    scalar_t Qcp_arm_;
    scalar_t Qcf_;
    scalar_t Rcp_angle_;
    scalar_t Rcp_lin_;
    scalar_t Rcp_leg_;
    scalar_t Rcp_arm_;
    scalar_t A_f_;
    Qcp_angle_ = config.Qcp_ang;
    Qcp_lin_ = config.Qcp_lin;
    Qcp_leg_ = config.Qcp_leg;
    Qcp_arm_ = config.Qcp_arm;
    Qcf_ = config.Qcf;
    Rcp_angle_ = config.Rcp_ang;
    Rcp_lin_ = config.Rcp_lin;
    Rcp_leg_ = config.Rcp_leg;
    Rcp_arm_ = config.Rcp_arm;
    A_f_ = config.A_f;

    setParam(Qcp_angle_, Qcp_lin_, Qcp_leg_, Qcp_arm_, Qcf_, Rcp_angle_, Rcp_lin_, Rcp_leg_, Rcp_arm_, A_f_);

    flag_noise = config.KF_noise_flag;
    flag_uncertainty_M = config.KF_uncertainty_M_flag;
    flag_uncertainty_C = config.KF_uncertainty_C_flag;
    flag_uncertainty_G = config.KF_uncertainty_G_flag;
    ROS_INFO_STREAM("\033[32m Update the force KF Param. \033[0m");
}

vector_t KF::getExternalTorque(const vector_t& rbdStateMeasured, scalar_t time, scalar_t period){
    // 得到四足机械臂当前的测量状态

    //ROS_INFO_STREAM("\033[32m MBO running. \033[0m");
    // 开始计时
    auto start = std::chrono::high_resolution_clock::now();

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
    matrix_t Jb_T;
    Jea_T = arm_j_.transpose().bottomRows(6);
    Jeb_T = arm_j_.transpose().topRows(6);
    Jb_T = base_j_.transpose();
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

    //ROS_INFO_STREAM("\033[32m KF get generalizedTorque. \033[0m");
    //rbdState_.segment(2 * generalizedCoordinatesNum_ + 7 + 3, info_.actuatedDofNum) = jointTor;
    T = period;
    //p = M * vMeasured_;  // 实际动量

    // 根据标志位决定动力学矩阵是否需要包含不确定性
    //M = M + flag_uncertainty_M * 0.7 * M;
    M = M + flag_uncertainty_M * 2 * M;
    C = C + flag_uncertainty_C * 1 * C;
    G = G + flag_uncertainty_G * 0.1 * G;
    //ROS_INFO_STREAM("\033[32m MBO M flag" << flag_uncertainty_M << "\033[0m");

    std::random_device rd;  // 随机数种子
    std::mt19937 gen(rd());  // 使用梅森旋转算法作为随机数生成器
    //std::normal_distribution<> d(0.0, 0.01);  // 均值为0，标准差为0.01的高斯分布
    std::normal_distribution<> d(0.0, 0.01);  // 均值为0，标准差为0.005的高斯分布
    for (int i = 0; i < vMeasured_.size(); ++i) {
        vMeasured_noise[i] = vMeasured_[i] +  d(gen);  // 为每个元素加上噪声
    }
    vMeasured_ = (1-flag_noise) * vMeasured_+ flag_noise * vMeasured_noise;
    p = M * vMeasured_; // 实际动量

    

    // 连续时间系统矩阵
    Ac.block(0, 0, 24, 24) = mat_zero1; // 左上角
    Ac.block(0, 24, 24, 24) = mat_indentity1; // 右上角
    Ac.block(24, 0, 24, 24) = mat_zero2; // 左下角
    Ac.block(24, 24, 24, 24) = A_f; // 右下角
    Bc.block(0, 0, 24, 24) = mat_indentity1;
    Bc.block(24, 0, 24, 24) = mat_zero2;
    Cc.block(0, 0, 24, 24) = mat_indentity1;
    Cc.block(0, 24, 24, 24) = mat_zero3;
    // 系统离散化
    Mat_c_1.block(0, 0, 48, 48) = Ac;
    Mat_c_1.block(0, 48, 48, 24) = Bc;
    Mat_c_1.block(48, 0, 24, 48) = mat_zero4;
    Mat_c_1.block(48, 48, 24, 24) = mat_zero1;
    
    //Mat_d_1 = (Mat_c_1* T).exp();
    Mat_d_1 = mat_indentity3 + Mat_c_1* T + 1/2 * T * T * Mat_c_1 * Mat_c_1 +  1/6 * T * T * T * Mat_c_1 * Mat_c_1 * Mat_c_1;
    Ad = Mat_d_1.block(0, 0, 48, 48);
    Bd = Mat_d_1.block(0, 48, 48, 24);
    Cd = Cc;

    // 协方差矩阵离散化
    Rd = Rc / T;
    H.block(0, 0, 48, 48) = Ac; // 左上角  常数矩阵
    H.block(0, 48, 48, 48) = Qc; // 右上角
    H.block(48, 0, 48, 48) = mat_zero5; // 左下角
    H.block(48, 48, 48, 48) = -Ac.transpose(); // 右下角
    
    
    //Mat_d_2 = (H* T).exp();
    Mat_d_2 = mat_indentity4 +  H* T + 1/2 * T * T * H * H +  1/6 * T * T * T * H * H * H;
    

    
    M_11 = Mat_d_2.block(0,0,48,48);
    M_12 = Mat_d_2.block(0,48,48,48);
    Qd = M_12 * M_11.transpose();
    tau_bar = generalizedTorque + C.transpose() * vMeasured_ - G;

    

    // 卡尔曼滤波更新
    // 预测阶段
    x_pred = Ad * x_hat_KF + Bd * tau_bar;
    P_pred = Ad * P_KF * Ad.transpose() + Qd;

    
    // 计算卡尔曼增益
    //K_KF = P_pred * Cd.transpose() * (Cd * P_pred * Cd.transpose() + Rd).completeOrthogonalDecomposition().pseudoInverse();
    
    K_KF = P_pred * Cd.transpose() * (Cd * P_pred * Cd.transpose() + Rd).inverse();


    // 校正阶段
    x_hat_KF = x_pred + K_KF * (p - Cd * x_pred);
    P_KF = (mat_indentity2 - K_KF * Cd) * P_pred;

    

    // 取出估计得到的外力矩(施加在机械臂末端的情况)
    torque_ext_hat = x_hat_KF.block(24,0,24,1);
    //ROS_INFO_STREAM("\033[32m KF finish one turn. \033[0m");

    // ETH论文公式（3）
    // 力施加在EE上的情况
    hat_ExternalWrenchs_EE = J_T_PseudoInverse_EE * torque_ext_hat;
    force_ext_hat_EE = hat_ExternalWrenchs_EE.block(info_.numThreeDofContacts * 3, 0, 6, 1);
    norm_force_ext_hat_EE = force_ext_hat_EE.head<3>().norm();

    // // 对外力的幅值进行一阶低通滤波
    // norm_force_ext_hat_EE = alpha_ee * norm_force_ext_hat_EE + (1 - alpha_ee) * pre_norm_force_ext_hat_EE;
    // // 保存当前值，供下次滤波使用
    // pre_norm_force_ext_hat_EE = norm_force_ext_hat_EE;


    //ROS_INFO_STREAM("\033[32m norm of external force:"<< norm_force_ext_hat_EE << "\033[0m");
    std_msgs::Float64 absForce_msg1;
    absForce_msg1.data = norm_force_ext_hat_EE;
    extForcehatAbs_EE_pub_.publish(absForce_msg1);

    // 利用计算得到的机械臂末端外力得到机器人受到的外力矩
    //torque_ext_hat = arm_j_.transpose() * force_ext_hat_EE;
    base_torque_ext_hat = Jeb_T * force_ext_hat_EE;  //base部分的分量

    // 力施加在base上的情况
    hat_ExternalWrenchs_base = J_T_PseudoInverse_base * torque_ext_hat;
    force_ext_hat_base = hat_ExternalWrenchs_base.block(info_.numThreeDofContacts * 3, 0, 6, 1);
    norm_force_ext_hat_base = force_ext_hat_base.head<3>().norm();

    // // 对外力的幅值进行一阶低通滤波
    // norm_force_ext_hat_base = alpha_base * norm_force_ext_hat_base + (1 - alpha_base) * pre_norm_force_ext_hat_base;
    // // 保存当前值，供下次滤波使用
    // pre_norm_force_ext_hat_base = norm_force_ext_hat_base;


    //ROS_INFO_STREAM("\033[32m norm of external force:"<< norm_force_ext_hat_base << "\033[0m");
    std_msgs::Float64 absForce_msg2;
    absForce_msg2.data = norm_force_ext_hat_base;
    extForcehatAbs_base_pub_.publish(absForce_msg2);

    // 利用计算得到的base上的外力得到base方向上的外力矩
    baseforce_torque_ext_hat = Jb_T * force_ext_hat_base;

    // 结束计时
    auto end = std::chrono::high_resolution_clock::now();

    // 计算时间差
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    //ROS_INFO_STREAM("KF computation time: " << duration.count() << " us");
    
    th_joint2 = 0.5;
    th_joint3 = 0.5;
    th_baseX = 3;
    th_baseY = 3;
    th_collision = 5;


    // // 力施加位置判断（施加在机械臂末端还是base上（为arm的joint力矩设置阈值，阈值设置需要考虑到有噪声的情况），发布消息：EE=1，base = -1）
    // if(norm_force_ext_hat_EE > th_collision || norm_force_ext_hat_base > th_collision)
    // {
    //     //发生了碰撞

    //     // if (torque_ext_hat(19) > th_joint2 || torque_ext_hat(20) > th_joint3)
    //     // {
    //     //     force_isolation = 1;
    //     // }
    //     // else if (base_torque_ext_hat(0) > th_baseX || base_torque_ext_hat(0) > th_baseY)
    //     // {
    //     //     force_isolation = -1;
    //     // }
    //     // else
    //     // {
    //     //     force_isolation =0;
    //     // }
    //     if (torque_ext_hat(19) < th_joint2 && torque_ext_hat(20) < th_joint3 && (base_torque_ext_hat(0) > th_baseX || base_torque_ext_hat(0) > th_baseY))
    //     {
    //         force_isolation = -1;
    //     }
    //     else if (torque_ext_hat(19) > th_joint2 || torque_ext_hat(20) > th_joint3)
    //     {
    //         force_isolation = 1;
    //     }
    //     else
    //     {
    //         force_isolation = 0;
    //     }
    // }
    // else
    // {
    //     force_isolation = 0;
    // }
    //torque_ext_hat = arm_j_.transpose() * force_ext_hat_EE;

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

    // std_msgs::Float64 tau_msg5;
    // tau_msg5.data = force_isolation;
    // force_isolation_pub_.publish(tau_msg5);

    std_msgs::Float64 tau_msg6;
    tau_msg6.data = torque_ext_hat(0);
    tau_ext_baseXhat_pub_.publish(tau_msg6);

    std_msgs::Float64 tau_msg7;
    tau_msg7.data = torque_ext_hat(1);
    tau_ext_baseYhat_pub_.publish(tau_msg7);

    std_msgs::Float64 tau_msg8;
    tau_msg8.data = baseforce_torque_ext_hat(0);
    baseforceXhat_pub_.publish(tau_msg8);

    std_msgs::Float64 tau_msg9;
    tau_msg9.data = baseforce_torque_ext_hat(1);
    baseforceYhat_pub_.publish(tau_msg9);

    return torque_ext_hat;
}


void KF::setParam(scalar_t QcpAng, scalar_t QcpLin, scalar_t QcpLeg, scalar_t QcpArm, scalar_t Qc_f, scalar_t RcpAng, scalar_t RcpLin, scalar_t RcpLeg, scalar_t RcpArm, scalar_t Af){
    // gainAngle.diagonal() << Ko_ang, Ko_ang, Ko_ang;
    // gainLinear.diagonal() << Ko_lin, Ko_lin, Ko_lin;
    // gainLeg.diagonal() << Ko_leg, Ko_leg, Ko_leg, Ko_leg, Ko_leg, Ko_leg, Ko_leg, Ko_leg, Ko_leg, Ko_leg, Ko_leg, Ko_leg;
    // gainArm.diagonal() << Ko_arm, Ko_arm, Ko_arm, Ko_arm, Ko_arm, Ko_arm;

    // // 组合为最终的增益矩阵Ko
    // gainMBO.block(0, 0, gainAngle.rows(), gainAngle.cols()) = gainAngle;
    // gainMBO.block(gainAngle.rows(), gainAngle.cols(), gainLinear.rows(), gainLinear.cols()) = gainLinear;
    // gainMBO.block(gainAngle.rows() + gainLinear.rows(), gainAngle.cols() + gainLinear.cols(), gainLeg.rows(), gainLeg.cols()) = gainLeg;
    // gainMBO.block(gainAngle.rows() + gainLinear.rows() + gainLeg.rows(), gainAngle.cols() + gainLinear.cols() + gainLeg.cols(), gainArm.rows(), gainArm.cols()) = gainArm;
    Qcp_ang.diagonal() << QcpAng, QcpAng, QcpAng;
    Qcp_Linear.diagonal() << QcpLin, QcpLin, QcpLin;
    Qcp_Leg.diagonal() << QcpLeg, QcpLeg, QcpLeg, QcpLeg, QcpLeg, QcpLeg, QcpLeg, QcpLeg, QcpLeg, QcpLeg, QcpLeg, QcpLeg;
    Qcp_Arm.diagonal() << QcpArm, QcpArm, QcpArm, QcpArm, QcpArm, QcpArm;
    Qcp.block(0, 0, Qcp_ang.rows(), Qcp_ang.cols()) = Qcp_ang;
    Qcp.block(Qcp_ang.rows(), Qcp_ang.cols(), Qcp_Linear.rows(), Qcp_Linear.cols()) = Qcp_Linear;
    Qcp.block(Qcp_ang.rows() + Qcp_Linear.rows(), Qcp_ang.cols() + Qcp_Linear.cols(), Qcp_Leg.rows(), Qcp_Leg.cols()) = Qcp_Leg;
    Qcp.block(Qcp_ang.rows() + Qcp_Linear.rows() + Qcp_Leg.rows(), Qcp_ang.cols() + Qcp_Linear.cols() + Qcp_Leg.cols(), Qcp_Arm.rows(), Qcp_Arm.cols()) = Qcp_Arm;
    Qcf.diagonal() << Qc_f, Qc_f, Qc_f, Qc_f, Qc_f, Qc_f, Qc_f, Qc_f, Qc_f, Qc_f, Qc_f, Qc_f, Qc_f, Qc_f, Qc_f, Qc_f, Qc_f, Qc_f, Qc_f, Qc_f, Qc_f, Qc_f, Qc_f, Qc_f;
    Qc.block(0, 0, Qcp.rows(), Qcp.cols()) = Qcp;
    Qc.block(Qcp.rows(), Qcp.cols(), Qcf.rows(), Qcf.cols()) = Qcf;

    Rc_ang.diagonal() << RcpAng, RcpAng, RcpAng;
    Rc_Linear.diagonal() << RcpLin, RcpLin, RcpLin;
    Rc_Leg.diagonal() << RcpLeg, RcpLeg, RcpLeg, RcpLeg, RcpLeg, RcpLeg, RcpLeg, RcpLeg, RcpLeg, RcpLeg, RcpLeg, RcpLeg;
    Rc_Arm.diagonal() << RcpArm, RcpArm, RcpArm, RcpArm, RcpArm, RcpArm;
    Rc.block(0, 0, Rc_ang.rows(), Rc_ang.cols()) = Rc_ang;
    Rc.block(Rc_ang.rows(), Rc_ang.cols(), Rc_Linear.rows(), Rc_Linear.cols()) = Rc_Linear;
    Rc.block(Rc_ang.rows() + Rc_Linear.rows(), Rc_ang.cols() + Rc_Linear.cols(), Rc_Leg.rows(), Rc_Leg.cols()) = Rc_Leg;
    Rc.block(Rc_ang.rows() + Rc_Linear.rows() + Rc_Leg.rows(), Rc_ang.cols() + Rc_Leg.cols() + Rc_Leg.cols(), Rc_Arm.rows(), Rc_Arm.cols()) = Rc_Arm;

    A_f.diagonal() << -Af, -Af, -Af, -Af, -Af, -Af, -Af, -Af, -Af, -Af, -Af, -Af, -Af, -Af, -Af, -Af, -Af, -Af, -Af, -Af, -Af, -Af, -Af, -Af;

}



}
