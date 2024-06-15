//
// Created by skywoodsz on 2023/5/6.
//

#include "qm_wbc/CompliantWbc.h"
#include "qm_wbc/HoQp.h"

namespace qm{
using namespace ocs2;
CompliantWbc::CompliantWbc(const ocs2::PinocchioInterface &pinocchioInterface, ocs2::CentroidalModelInfo info,
                           const ocs2::PinocchioEndEffectorKinematics &eeKinematics,
                           const ocs2::PinocchioEndEffectorKinematics &armEeKinematics,
                           ros::NodeHandle &controller_nh)
    : WbcBase(pinocchioInterface, info, eeKinematics, armEeKinematics, controller_nh)
{
    tau_max_.setZero();             //机械臂力矩最大值
    imp_desired_.resize(generalizedCoordinatesNum_);
    imp_desired_.setZero();

    // init compliant controller
    impendace_controller_ = std::make_shared<CartesianImpendance>(pinocchioInterface, info, armEeKinematics, controller_nh);
    BoundedAdmittanceInit(pinocchioInterface, info, armEeKinematics, controller_nh);
    BaseBoundedAdmittanceInit(pinocchioInterface, info, armEeKinematics, controller_nh);
    BaseAdmittanceInit(pinocchioInterface, info, armEeKinematics, controller_nh);

    // dynamic reconfigure
    ros::NodeHandle nh_weight = ros::NodeHandle(controller_nh,"compliant");
    dynamic_srv_ = std::make_shared<dynamic_reconfigure::Server<qm_wbc::CompliantConfig>>(nh_weight);
    dynamic_reconfigure::Server<qm_wbc::CompliantConfig>::CallbackType cb = [this](auto&& PH1, auto&& PH2) {
        dynamicCallback(std::forward<decltype(PH1)>(PH1), std::forward<decltype(PH2)>(PH2));
    };
    dynamic_srv_->setCallback(cb);
}
// init bounded admittance of joints
void CompliantWbc::BoundedAdmittanceInit(const ocs2::PinocchioInterface &pinocchioInterface, ocs2::CentroidalModelInfo info,
                                         const ocs2::PinocchioEndEffectorKinematics &armEeKinematics, ros::NodeHandle &controller_nh) {
    ros::NodeHandle nh;
    bounded_admittance_controller2_ = std::make_shared<BoundedAdmittanceNoImp>(pinocchioInterface, info, armEeKinematics, controller_nh);
    bounded_admittance_controller2_->setJointIdx(2);
    tau_cmd1_pub = nh.advertise<std_msgs::Float64>("/BoundedAdmittanceNoImp/tau_cmd_2", 1);

    bounded_admittance_controller1_ = std::make_shared<BoundedAdmittanceNoImp>(pinocchioInterface, info, armEeKinematics, controller_nh);
    bounded_admittance_controller1_->setJointIdx(1);
    tau_cmd2_pub = nh.advertise<std_msgs::Float64>("/BoundedAdmittanceNoImp/tau_cmd_1", 1);
}
/*
void CompliantWbc::BoundedAdmittanceInit(const ocs2::PinocchioInterface &pinocchioInterface, ocs2::CentroidalModelInfo info,
                                         const ocs2::PinocchioEndEffectorKinematics &armEeKinematics, ros::NodeHandle &controller_nh) {
    bounded_admittance_controller2_ = std::make_shared<BoundedAdmittance>(pinocchioInterface, info, armEeKinematics, controller_nh);
    bounded_admittance_controller2_->setJointIdx(2);

    bounded_admittance_controller1_ = std::make_shared<BoundedAdmittance>(pinocchioInterface, info, armEeKinematics, controller_nh);
    bounded_admittance_controller1_->setJointIdx(1);
}
*/
// init bounded admittance of the base
void CompliantWbc::BaseBoundedAdmittanceInit(const ocs2::PinocchioInterface &pinocchioInterface, ocs2::CentroidalModelInfo info,
                                             const ocs2::PinocchioEndEffectorKinematics &armEeKinematics, ros::NodeHandle &controller_nh) {
    bounded_admittance_controller_base_x_ = std::make_shared<BoundedAdmittanceWithK>(pinocchioInterface, info, armEeKinematics, controller_nh);
    bounded_admittance_controller_base_x_->setJointIdx(0);

//    bounded_admittance_controller_base_y_ = std::make_shared<BoundedAdmittanceWithK>(pinocchioInterface, info, armEeKinematics, controller_nh);
//    bounded_admittance_controller_base_y_->setJointIdx(1);
}

void CompliantWbc::BaseAdmittanceInit(const ocs2::PinocchioInterface &pinocchioInterface, ocs2::CentroidalModelInfo info,
                                             const ocs2::PinocchioEndEffectorKinematics &armEeKinematics, ros::NodeHandle &controller_nh) {
    admittance_controller_base_x_ = std::make_shared<BaseBoundedAdmittanceNoImp>(pinocchioInterface, info, armEeKinematics, controller_nh);
    admittance_controller_base_x_->setJointIdx(0);

//    bounded_admittance_controller_base_y_ = std::make_shared<BoundedAdmittanceWithK>(pinocchioInterface, info, armEeKinematics, controller_nh);
//    bounded_admittance_controller_base_y_->setJointIdx(1);
}
// bounded admittance of joints control law   tau_feedback 应该设成0
void CompliantWbc::BoundedAdmittanceUpdate(const ocs2::vector_t &rbdStateMeasured, ocs2::scalar_t time,
                                                ocs2::scalar_t period) {
    vector6_t tau_feedback, tau_desired, tau_addmitance, torque_ext;
    tau_feedback.setZero();
    tau_desired.setZero();
    tau_addmitance.setZero();
    torque_ext.setZero();

    torque_ext = getExternalArmTorque();

    //tau_feedback = rbdStateMeasured.tail(6);
    //tau_desired = imp.tail(6);
    scalar_t q1_d;
    scalar_t q2_d;

    if(referenceManagerPtr_ == nullptr){
        throw std::runtime_error("[CompliantWbc] ReferenceManager pointer cannot be a nullptr!");
    } else{
        const auto& targetTrajectories = referenceManagerPtr_->getTargetTrajectories();
        const auto& timeTrajectory = targetTrajectories.timeTrajectory;
        const auto& stateTrajectory = targetTrajectories.stateTrajectory;   //维度可以参考src/qm_control/qm_controllers/src/QMController.cpp  119行
        const size_t size = timeTrajectory.size();
        q1_d = stateTrajectory[size-1][25];   //MPC算出来的期望的机械臂第2关节角度
        q2_d = stateTrajectory[size-1][26];   //MPC算出来的期望的机械臂第3关节角度
    }

    //只考虑机械臂2，3关节的纯柔顺
    //vector_t tmp;
    bounded_admittance_controller1_->setTorqueMax(tau_max_[1]);
    bounded_admittance_controller1_->setParam(M1_, K1_, D1_, L1_, M1x_, D1x_,K1x_);
    bounded_admittance_controller1_->setForceFeedback(0);
    bounded_admittance_controller1_->setTorqueExternal(torque_ext[1]);
    bounded_admittance_controller1_->setDesired(0, 0, q1_d);    //设置公式里的期望关节加速度，速度，位置
    //bounded_admittance_controller1_->setForceDesired(tau_desired[1]);
    bounded_admittance_controller1_->update(rbdStateMeasured, time, period);
    //tau_addmitance[1] = tmp[1];

    bounded_admittance_controller2_->setTorqueMax(tau_max_[2]);
    bounded_admittance_controller2_->setParam(M2_, K2_, D2_, L2_, M2x_, D2x_,K2x_);
    bounded_admittance_controller2_->setForceFeedback(0);
    bounded_admittance_controller2_->setTorqueExternal(torque_ext[2]);
    bounded_admittance_controller2_->setDesired(0, 0, q2_d);
    //bounded_admittance_controller2_->setForceDesired(tau_desired[2]);
    bounded_admittance_controller2_->update(rbdStateMeasured, time, period);
    //tau_addmitance[2] = tmp[1];


    //return tau_addmitance;
}
// bounded admittance of base control law
vector6_t CompliantWbc::BaseBoundedAdmittanceUpdate(const ocs2::vector_t &rbdStateMeasured, ocs2::scalar_t time,    // 参数force_z是所有接触组底力Z方向的MPC期望值之和
                                                    ocs2::scalar_t period, ocs2::vector_t imp, scalar_t force_z) {
    vector_t tau_desired(2), tau_addmitance(2);
    tau_desired.setZero();
    tau_desired = imp.head(2);

    // get the desired position from the reference
    scalar_t base_xd{};
    if(referenceManagerPtr_ == nullptr){
        throw std::runtime_error("[CompliantWbc] ReferenceManager pointer cannot be a nullptr!");
    } else{
        const auto& targetTrajectories = referenceManagerPtr_->getTargetTrajectories();
        const auto& timeTrajectory = targetTrajectories.timeTrajectory;
        const auto& stateTrajectory = targetTrajectories.stateTrajectory;   //维度可以参考src/qm_control/qm_controllers/src/QMController.cpp  119行
        const size_t size = timeTrajectory.size();
        base_xd = stateTrajectory[size-1][6];
    }

    vector_t tmp;
    bounded_admittance_controller_base_x_->setTorqueMax(force_z * mu_); // 设置base x 方向的力最大值  =  frictionCoeff_ * force_z
    bounded_admittance_controller_base_x_->setParam(Mbasex_, Kbasex_, Bbasex_, Lbasex_, Mbase_px_, Bbase_px_, Kbase_px_);
    bounded_admittance_controller_base_x_->setDesired(0, 0, base_xd);
//    bounded_admittance_controller_base_x_->setDesired(baseAccDesired_[0], vDesired_[0], qDesired_[0]);
    bounded_admittance_controller_base_x_->setForceDesired(tau_desired[0]);
    bounded_admittance_controller_base_x_->setForceFeedback(0);     //base部分没有力传感器，所以直接把base的力反馈设置成0
    tmp = bounded_admittance_controller_base_x_->update(rbdStateMeasured, time, period);
    tau_addmitance[0] = tmp[1];

//    bounded_admittance_controller_base_y_->setTorqueMax(force_z * mu_); // mpc force z * mu
//    bounded_admittance_controller_base_y_->setParam(Mbasex_, Kbasex_, Bbasex_, Lbasex_, Mbase_px_, Bbase_px_, Kbase_px_);
//    bounded_admittance_controller_base_y_->setDesired(0, 0, base_y_);
//    bounded_admittance_controller_base_y_->setForceDesired(tau_desired[1]);
//    bounded_admittance_controller_base_y_->setForceFeedback(0);
//    tmp = bounded_admittance_controller_base_y_->update(rbdStateMeasured, time, period);
//    tau_addmitance[1] = tmp[1];

    return tau_addmitance;
}

void CompliantWbc::BaseAdmittanceUpdate(const vector_t& rbdStateMeasured, scalar_t time, scalar_t period){
    vector_t tau_desired(2), tau_addmitance(2), tau_feedback(2), torque_ext(2);
    tau_feedback.setZero();
    tau_desired.setZero();
    tau_addmitance.setZero();
    torque_ext.setZero();
    torque_ext = getExternalBaseTorque();
    // get the desired position from the reference
    scalar_t base_xd{};
    if(referenceManagerPtr_ == nullptr){
        throw std::runtime_error("[CompliantWbc] ReferenceManager pointer cannot be a nullptr!");
    } else{
        const auto& targetTrajectories = referenceManagerPtr_->getTargetTrajectories();
        const auto& timeTrajectory = targetTrajectories.timeTrajectory;
        const auto& stateTrajectory = targetTrajectories.stateTrajectory;   //维度可以参考src/qm_control/qm_controllers/src/QMController.cpp  119行
        const size_t size = timeTrajectory.size();
        base_xd = stateTrajectory[size-1][6];
    }

    admittance_controller_base_x_->setTorqueMax(1000); // 设置base x 方向的力最大值  =  frictionCoeff_ * force_z
    admittance_controller_base_x_->setParam(Mbasex_, Kbasex_, Bbasex_, Lbasex_, Mbase_px_, Bbase_px_, Kbase_px_);
    admittance_controller_base_x_->setDesired(0, 0, base_xd);
//    bounded_admittance_controller_base_x_->setDesired(baseAccDesired_[0], vDesired_[0], qDesired_[0]);
    //admittance_controller_base_x_->setForceDesired(tau_desired[0]);
    admittance_controller_base_x_->setForceFeedback(0);     //base部分没有力传感器，所以直接把base的力反馈设置成0
    admittance_controller_base_x_->setTorqueExternal(torque_ext[0]);
    admittance_controller_base_x_->update(rbdStateMeasured, time, period);
//    bounded_admittance_controller_base_y_->setTorqueMax(force_z * mu_); // mpc force z * mu
//    bounded_admittance_controller_base_y_->setParam(Mbasex_, Kbasex_, Bbasex_, Lbasex_, Mbase_px_, Bbase_px_, Kbase_px_);
//    bounded_admittance_controller_base_y_->setDesired(0, 0, base_y_);
//    bounded_admittance_controller_base_y_->setForceDesired(tau_desired[1]);
//    bounded_admittance_controller_base_y_->setForceFeedback(0);
//    tmp = bounded_admittance_controller_base_y_->update(rbdStateMeasured, time, period);
//    tau_addmitance[1] = tmp[1];

}
/*  师兄论文方案
// bounded admittance control law
vector_t CompliantWbc::BoundanceAdmittanceControl(const ocs2::vector_t &stateDesired, const ocs2::vector_t &inputDesired,
             const ocs2::vector_t &rbdStateMeasured, size_t mode, ocs2::scalar_t period, ocs2::scalar_t time) {
    // get desire pose and velocity from planner
    vector3_t eeDesiredPosition = WbcBase::getEEPosition();
    vector3_t eeDesiredVelocity = WbcBase::getEEVelocity();

    // impendance control
    impendace_controller_->setDesiredPosition(eeDesiredPosition);
    impendace_controller_->setDesiredVelocity(eeDesiredVelocity);
    vector_t imp = impendace_controller_->update(rbdStateMeasured, time, period);
    imp_desired_ = imp;

    // arm's bounded admittance
    vector_t ba_tau = BoundedAdmittanceUpdate(rbdStateMeasured, time, period, imp);
    vector6_t compliant_tau = imp.tail(6);
    for (int i = 1; i < 3; ++i) {                    // 只对第二三关节采用先阻抗再导纳，另外的关节都只用阻抗。
        compliant_tau[i] = ba_tau[i];
    }

    // base's bounded admittance
    scalar_t force_z = 0;
    for (size_t i = 0; i < 4; ++i) {
        force_z += inputDesired[3*i+2];
    }
    vector_t tau_cmd = BaseBoundedAdmittanceUpdate(rbdStateMeasured, time, period, imp, force_z);
    vector_t proxy_x = bounded_admittance_controller_base_x_->getProxyState();

    // constraint
    Task task0 = formulateFloatingBaseEomWithEEForceTask() + formulateTorqueLimitsTaskWithEEForceTask()
                 + formulateNoContactMotionTask() + formulateFrictionConeTask();
    // compliance
    Task taskBA = formulateManipulatorTorqueTask(compliant_tau.tail(6));
    Task taskBaseProxy = formulateBaseXMotionTrackingTask(proxy_x[0], proxy_x[1], proxy_x[2]);  //proxy_x[0] [1] [2]分别代表proxy的位置 速度 加速度

    Task task1 = formulateBaseHeightMotionTask() + formulateBaseAngularMotionTask() + formulateSwingLegTask() * 100
                 + formulateEeAngularMotionTrackingTask() + taskBA + taskBaseProxy + formulateBaseYLinearMotionTask();

//    Task task3 = formulateContactForceTask(inputDesired);
    Task task3 = formulateZContactForceTask(inputDesired) + formulateYContactForceTask(inputDesired)
            + formulateXContactForceTaskWithCompliant(inputDesired, tau_cmd);     //这里的tau_cmd是阻抗+导纳算出来的base的x方向的合力

    HoQp hoQp(task3, std::make_shared<HoQp>(task1, std::make_shared<HoQp>(task0)));
    vector_t x_optimal = hoQp.getSolutions();
    return WbcBase::updateCmdWithEEForce(x_optimal);
}
*/
/**
 * @brief 纯导纳实现  v1.0   机械臂的2、3关节采用纯导纳，base依旧采用阻抗+导纳  （为了便于调试保留了纯阻抗，通过滑块使能纯导纳关节）
 * 
 * @param stateDesired 
 * @param inputDesired 
 * @param rbdStateMeasured 
 * @param mode 
 * @param period 
 * @param time 
 * @return vector_t 
 */

vector_t CompliantWbc::BoundanceAdmittanceControl(const ocs2::vector_t &stateDesired, const ocs2::vector_t &inputDesired,
             const ocs2::vector_t &rbdStateMeasured, size_t mode, ocs2::scalar_t period, ocs2::scalar_t time) {
    // get desire pose and velocity from planner
    vector3_t eeDesiredPosition = WbcBase::getEEPosition();
    vector3_t eeDesiredVelocity = WbcBase::getEEVelocity();

    // impendance control
    impendace_controller_->setDesiredPosition(eeDesiredPosition);
    impendace_controller_->setDesiredVelocity(eeDesiredVelocity);
    vector_t imp = impendace_controller_->update(rbdStateMeasured, time, period);
    imp_desired_ = imp;

    // arm's bounded admittance with no IMP
    //vector_t ba_tau = BoundedAdmittanceUpdate(rbdStateMeasured, time, period);
    BoundedAdmittanceUpdate(rbdStateMeasured, time, period);
    vector6_t ImpArm_tau  = imp.tail(6);

    // for (int i = begin_; i < end_; ++i) {        //手动使能纯导纳关节           
    //     compliant_tau[i] = ba_tau[i];
    // }

    //提取2、3关节的proxy状态向量（位置 速度 加速度）
    vector_t proxy_1  = bounded_admittance_controller1_->getProxyState();
    vector_t proxy_2  = bounded_admittance_controller2_->getProxyState();

    
    // base's bounded admittance（base部分和师兄一样）
    scalar_t force_z = 0;
    for (size_t i = 0; i < 4; ++i) {
        force_z += inputDesired[3*i+2];
    }
    vector_t tau_cmd = BaseBoundedAdmittanceUpdate(rbdStateMeasured, time, period, imp, force_z);
    //vector_t proxy_x = bounded_admittance_controller_base_x_->getProxyState();
    vector_t proxy_x = admittance_controller_base_x_->getProxyState();
    // constraint
    Task task0 = formulateFloatingBaseEomWithEEForceTask() + formulateTorqueLimitsTaskWithEEForceTask()
                 + formulateNoContactMotionTask() + formulateFrictionConeTask();
    // arm compliance
    Task taskAllImpedance  = formulateManipulatorTorqueTask(ImpArm_tau.tail(6));

    Task taskJoint12Admittance  = ManipulatorTorqueTaskTwoAdmittance(ImpArm_tau.tail(6));
    Task taskJoint1Admittance  = ManipulatorTorqueTaskJointOneAdmittance(ImpArm_tau.tail(6));  //除了joint2都是跟踪阻抗   joint2 留给导纳
    Task taskJoint2Admittance  = ManipulatorTorqueTaskJointTwoAdmittance(ImpArm_tau.tail(6));  //除了joint3都是跟踪阻抗   joint3 留给导纳

    Task trackJoint1Proxy = formulateJoint1ProxyTrackingTask(proxy_1[0],proxy_1[1],proxy_1[2]);
    Task trackJoint2Proxy = formulateJoint2ProxyTrackingTask(proxy_2[0],proxy_2[1],proxy_2[2]);
    Task trackJoint12Proxy = formulateJoint12ProxyTrackingTask(proxy_1[0],proxy_1[1],proxy_1[2],proxy_2[0],proxy_2[1],proxy_2[2]);

    Task taskBA  = taskAllImpedance;
    switch (begin_) {
        case 1:
            if (end_ == 2) {
                taskBA = taskJoint1Admittance + trackJoint1Proxy;
            } 
            else if (end_ == 3) {
                taskBA = taskJoint12Admittance + trackJoint12Proxy;
            }
            break;
        case 2:
            if (end_ == 3) {
                taskBA = taskJoint2Admittance + trackJoint2Proxy;
            }
            break;
        default:
            break;
    }

    // base compliance 
    Task taskBaseProxy = formulateBaseXAdmittanceTrackingTask(proxy_x[0], proxy_x[1], proxy_x[2]);  //proxy_x[0] [1] [2]分别代表proxy的位置 速度 加速度



    Task task1 = formulateBaseHeightMotionTask() + formulateBaseAngularMotionTask() + formulateSwingLegTask() * 100
                 + formulateEeAngularMotionTrackingTask() + taskBA + taskBaseProxy + formulateBaseYLinearMotionTask();

   Task task3 = formulateContactForceTask(inputDesired);
    // Task task3 = formulateZContactForceTask(inputDesired) + formulateYContactForceTask(inputDesired)
    //         + formulateXContactForceTaskWithCompliant(inputDesired, tau_cmd);     //这里的tau_cmd是阻抗+导纳算出来的base的x方向的合力

    HoQp hoQp(task3, std::make_shared<HoQp>(task1, std::make_shared<HoQp>(task0)));
    vector_t x_optimal = hoQp.getSolutions();


    return WbcBase::updateCmdWithEEForce(x_optimal);
}



vector_t CompliantWbc::update(const ocs2::vector_t &stateDesired, const ocs2::vector_t &inputDesired,
                              const ocs2::vector_t &rbdStateMeasured, size_t mode, ocs2::scalar_t period,
                              ocs2::scalar_t time) {
    // wbc
    WbcBase::update(stateDesired, inputDesired, rbdStateMeasured, mode, period, time);
    setManipulatorTorqueLimit(tau_max_);

    if(time < 5)
    {
        Task task0 = formulateFloatingBaseEomWithEEForceTask() + formulateTorqueLimitsTaskWithEEForceTask()
                     + formulateNoContactMotionTask() + formulateFrictionConeTask();
        Task taskInit = formulateArmJointNomalTrackingTask();
        Task task3 = formulateContactForceTask(inputDesired);

        HoQp hoQp(task3, std::make_shared<HoQp>(taskInit, std::make_shared<HoQp>(task0)));
        vector_t x_optimal = hoQp.getSolutions();
        return WbcBase::updateCmdWithEEForce(x_optimal);
    }
    //发布tau_cmd
    // std_msgs::Float64 f_msg;
    // f_msg.data = -tau_max_;
    // torque_pub_.publish(f_msg);  
    vector_t torque = BoundanceAdmittanceControl(stateDesired, inputDesired, rbdStateMeasured, mode, period, time);
    torque = torque.tail(18);
    std_msgs::Float64 torque_msg1,torque_msg2;
    torque_msg1.data = torque[13];
    torque_msg2.data = torque[14];
    tau_cmd1_pub.publish(torque_msg1);
    tau_cmd2_pub.publish(torque_msg2);
    return BoundanceAdmittanceControl(stateDesired, inputDesired, rbdStateMeasured, mode, period, time);
}



void CompliantWbc::dynamicCallback(qm_wbc::CompliantConfig &config, uint32_t) {
    tau_max_ << config.joint1_tau_max, config.joint2_tau_max, config.joint3_tau_max,
            config.joint4_tau_max, config.joint5_tau_max, config.joint6_tau_max;

    // bounde admittance for joint1 & joint2
    M1x_ = config.M1x;
    D1x_ = config.D1x;
    K1x_ = config.K1x;

    M2x_ = config.M2x;
    D2x_ = config.D2x;
    K2x_ = config.K2x;

    // M1_ = config.M1;
    // K1_ = config.K1;
    // D1_ = config.D1;
    // L1_ = config.L1;

    // M2_ = config.M2;
    // K2_ = config.K2;
    // D2_ = config.D2;
    // L2_ = config.L2;  
    // Mbx_ = config.Mbx;
    // Bbx_ = config.Bbx;
    // Mb2_ = config.Mb2;
    // Kb2_ = config.Kb2;
    // Bb2_ = config.Bb2;
    // Lb2_ = config.Lb2;
    // Mb1_ = config.Mb1;
    // Kb1_ = config.Kb1;
    // Bb1_ = config.Bb1;
    // Lb1_ = config.Lb1;

    // bounde admittance for base
    Mbase_px_ = config.Mbase_px;
    Bbase_px_ = config.Bbase_px;
    Kbase_px_ = config.Kbase_px;
    Mbase_py_ = config.Mbase_py;
    Bbase_py_ = config.Bbase_py;
    Kbase_py_ = config.Kbase_py;

    Mbasex_ = config.Mbasex;
    Bbasex_ = config.Bbasex;
    Kbasex_ = config.Kbasex;
    Lbasex_ = config.Lbasex;
    Mbasey_ = config.Mbasey;
    Bbasey_ = config.Bbasey;
    Kbasey_ = config.Kbasey;
    Lbasey_ = config.Lbasey;

    begin_ = config.begin;
    end_   = config.end;

    mu_ = config.mu;

    ROS_INFO_STREAM("\033[32m Update the CompliantWbc param. \033[0m");
}

}
