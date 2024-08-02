//
// Created by hitlfh on 2023/6/24.
//
#include "qm_compliant/AdmittanceMultiDim.h"
#include "qm_compliant/Numerics.h"

namespace qm{
using namespace ocs2;

AdmittanceMultiDim::AdmittanceMultiDim(const ocs2::PinocchioInterface &pinocchioInterface, ocs2::CentroidalModelInfo info,
                                               const ocs2::PinocchioEndEffectorKinematics &armEeKinematics, ros::NodeHandle &controller_nh) 
                                               :CompliantBase(pinocchioInterface, info, armEeKinematics, controller_nh)
{
    generalizedCoordinatesNum_ = info.generalizedCoordinatesNum;
    initParam();
}

void AdmittanceMultiDim::initParam() {

    // multi dimensional initialization
    lambda = 0;
    k_1 = 0;
    T_ = 0;
    Mx.setZero();
    Dx.setZero();
    Kx.setZero();
    B.setZero();
    Khat.setZero();
    K.setZero();

    q0.setZero();
    dot_q0.setZero();
    ddot_q0.setZero();
    qx.setZero();
    qx_pre.setZero();
    q.setZero();
    q_pre.setZero();
    
    ux.setZero();
    ux_pre.setZero();
    tau.setZero();
    tau_star.setZero();
    phi_a.setZero();
    phi_b.setZero();
    ax.setZero();

    torque_desired.setZero();
    tau_ext.setZero();

    tau_ext_temp.setZero();

    q1_star.setZero();
    n.setZero();
    init_flag_ = false;
    tau_flag_ = false;
    param_flag_ = false;
}


void AdmittanceMultiDim::setTorqueMax(ocs2::scalar_t tau_max1, ocs2::scalar_t tau_max2) {
    tau_max[0] = tau_max1;
    tau_max[1] = tau_max2;
    tau_flag_ = true;
    //multi
}

void AdmittanceMultiDim::setProxyParam(scalar_t Mx1, scalar_t Mx2, scalar_t Dx1,  scalar_t Dx2, scalar_t Kx1, scalar_t Kx2) {

    //对角线赋值
    Mx.diagonal() << Mx1 , Mx2;  // 设置对角线元素
    Dx.diagonal() << Dx1 , Dx2;  // 设置对角线元素
    Kx.diagonal() << Kx1 , Kx2;
    param_flag_ = true;

    // multi
}
void AdmittanceMultiDim::setScalarSlidingModeParam(scalar_t Lambda, scalar_t K_1){
    lambda= Lambda;
    k_1 = K_1;
}
void AdmittanceMultiDim::setMatrixSlidingModeParam(matrix2_t SM_Lambda, matrix2_t SM_K_1){
    Lambda = SM_Lambda;
    K_1 = SM_K_1;
}
void AdmittanceMultiDim::setPDParam(matrix2_t K_p, matrix2_t K_d){
    Kp_ = K_p;
    Kd_ = K_d;
}
void AdmittanceMultiDim::setIDControllerParam(matrix2_t M_a,  matrix2_t C_a, vector2_t G_a){  
    M = M_a;
    G = G_a;
    C = C_a;
}

void AdmittanceMultiDim::setDesired(vector2_t ddot_q0_, vector2_t dot_q0_, vector2_t q0_) {
    ddot_q0 = ddot_q0;
    dot_q0 = dot_q0;
    q0 = q0_;
    //multi
}

// 二维投影函数
vector2_t AdmittanceMultiDim::projectionFunction(const vector2_t& tau) {  
    vector2_t x = tau;
    if(almost_ge(x[0], tau_max[0]))
        x[0] = tau_max[0];
    if(almost_le(x[0], -tau_max[0]))
        x[0] = -tau_max[0];
    if(almost_ge(x[1], tau_max[1]))
        x[1] = tau_max[1];
    if(almost_le(x[1], -tau_max[1]))
        x[1] = -tau_max[1];  
    return x;
    //multi
}

vector_t AdmittanceMultiDim::getJoint1ProxyState() {
    vector_t state(3); // qx, qx_dot, qx_ddot

    state(0) = qx[0];
    state(1) = ux[0];
    state(2) = ax[0];
    return state;
}

vector_t AdmittanceMultiDim::getJoint2ProxyState() {
    vector_t state(3); // qx, qx_dot, qx_ddot

    state(0) = qx[1];
    state(1) = ux[1];
    state(2) = ax[1];
    return state;
}

vector_t AdmittanceMultiDim::update(const ocs2::vector_t &rbdStateMeasured, ocs2::scalar_t time, ocs2::scalar_t period) {
    CompliantBase::update(rbdStateMeasured, time, period);
    //获取当前关节实际位置和速度
    vector_t arm_joint_pos = rbdStateMeasured.segment(generalizedCoordinatesNum_-6, 6);
    vector_t arm_joint_vel = rbdStateMeasured.segment(2*generalizedCoordinatesNum_-6, 6);
    vector2_t joint_pos = arm_joint_pos.block(1, 0, 2, 1);    // 获取当前第2、3关节的角度值
    vector2_t joint_vel = arm_joint_vel.block(1, 0, 2, 1);    // 获取当前第2、3关节的角速度值


    if(!init_flag_)
    {
        init_flag_ = true;
        ros::NodeHandle nh;
        
        //发布各类话题
        qx1_pub_ = nh.advertise<std_msgs::Float64>("/AdmittanceMultiDim/qx1" , 1);
        torque1_limit_pub_ = nh.advertise<std_msgs::Float64>("/AdmittanceMultiDim/torque_limit1" , 1);
        tau1_pub_ = nh.advertise<std_msgs::Float64>("/AdmittanceMultiDim/tau_cmd1", 1);
        torque1_ext_pub_ = nh.advertise<std_msgs::Float64>("/AdmittanceMultiDim/torque_ext1", 1);

        qx2_pub_ = nh.advertise<std_msgs::Float64>("/AdmittanceMultiDim/qx2" , 1);
        torque2_limit_pub_ = nh.advertise<std_msgs::Float64>("/AdmittanceMultiDim/torque_limit2" , 1);
        tau2_pub_ = nh.advertise<std_msgs::Float64>("/AdmittanceMultiDim/tau_cmd2", 1);
        torque2_ext_pub_ = nh.advertise<std_msgs::Float64>("/AdmittanceMultiDim/torque_ext2", 1);

        tau_star1_pub_ = nh.advertise<std_msgs::Float64>("/AdmittanceMultiDim/tau_star1", 1);
        tau_star2_pub_ = nh.advertise<std_msgs::Float64>("/AdmittanceMultiDim/tau_star2", 1);

        qx = joint_pos;
        qx_pre = joint_pos;
        q = joint_pos;
        q_pre = joint_pos;

        ux = joint_vel;
        ux_pre = joint_vel;
    }

    T_ = period;
    q = joint_pos;   
    
    // //第二版PDF推导
    // ux_star = ((Mx + Dx * T_+Kx * T_*T_).inverse())*(Mx * ux_pre + Mx * qx_pre / T_ + Dx * qx_pre + T_ * (Mx * ddot_q0 + Dx * dot_q0 + Kx * q0 + tau_ext));  // 2.27
    // qx_star = T_ * ux_star; // 2.28
    // phi_a = ((M +  C * T_)* q) / (T_ * T_) + B * q_pre / T_ + G; // 2.29
    // phi_b =  M * (qx_pre + T_ * ux_pre) / (T_ * T_) + B_hat * qx_pre / T_; // 2.30
    // q1_star = q + ((M / (T_ * T_) + Khat).inverse()) * (phi_b - phi_a); // 2.31
    // tau_star = (M / (T_ * T_) + Khat) * (qx_star - q1_star);  // 2.32
    // tau = projectionFunction(tau_star); // 2.33
    // qx = q1_star + (M / (T_ * T_) + Khat).inverse() * tau; //2.34
    // ux = (qx - qx_pre) / T_;  // 2.35
    // ax = (ux - ux_pre) / T_;
    
    ax = Mx.inverse() * (-Dx * (ux - dot_q0) - Kx * (qx - q0) + tau_ext) + ddot_q0;
    ux = ux_pre + T_ * ax;
    qx = qx_pre + T_ * ux;
    
    ddot_qr = ax + Lambda * (ux - joint_vel);
    dot_qr = ux + Lambda * (qx - joint_pos);
    S = (ux - joint_vel) + Lambda * (qx - joint_pos);

    tau_star = M * ddot_qr + C * dot_qr + G + K_1 * S;
    tau = projectionFunction(tau_star);
    //把 lambda和k1换成矩阵的离散化
    
    //update
    qx_pre = qx;
    ux_pre = ux;
    q_pre = q;

    vector2_t tau_cmd;
    tau_cmd = tau;
        
    // publish msg

    // 发布tau_star
    std_msgs::Float64 tau_star_msg1, tau_star_msg2;
    tau_star_msg1.data = tau_star[0];
    tau_star_msg2.data = tau_star[1];
    tau_star1_pub_.publish(tau_star_msg1);
    tau_star2_pub_.publish(tau_star_msg2);

    //发布proxy位置
    std_msgs::Float64 proxy_pos_msg1, proxy_pos_msg2;
    proxy_pos_msg1.data = qx[0];
    proxy_pos_msg2.data = qx[1];
    qx1_pub_.publish(proxy_pos_msg1);
    qx2_pub_.publish(proxy_pos_msg2);

    //发布关节力矩极限值
    std_msgs::Float64 flimit_msg1, flimit_msg2;
    flimit_msg1.data = -tau_max[0];
    flimit_msg2.data = -tau_max[1];
    torque1_limit_pub_.publish(flimit_msg1);
    torque2_limit_pub_.publish(flimit_msg2);            

    // 发布输出力矩
    std_msgs::Float64 tau_msg1, tau_msg2;
    tau_msg1.data = tau[0];
    tau_msg2.data = tau[1];
    tau1_pub_.publish(tau_msg1);
    tau2_pub_.publish(tau_msg2);

    //publish外部力映射到关节的外力矩
    std_msgs::Float64 torque_ext_msg1, torque_ext_msg2;
    torque_ext_msg1.data = tau_ext[0];
    torque_ext_msg2.data = tau_ext[1];
    torque1_ext_pub_.publish(torque_ext_msg1);
    torque2_ext_pub_.publish(torque_ext_msg2);

    //发布重力值
    std_msgs::Float64 Gravity_msg1, Gravity_msg2;
    Gravity_msg1.data = G[0];
    Gravity_msg2.data = G[1];
    Multi_gravity1_pub_.publish(Gravity_msg1);
    Multi_gravity2_pub_.publish(Gravity_msg2);
    
    return tau_cmd;
}


}

