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
    AdmittanceInit(pinocchioInterface, info, armEeKinematics, controller_nh);
    BaseBoundedAdmittanceInit(pinocchioInterface, info, armEeKinematics, controller_nh);
    BaseAdmittanceInit(pinocchioInterface, info, armEeKinematics, controller_nh);
    BaseBAMultiDimiInit(pinocchioInterface, info, armEeKinematics, controller_nh); // base多维饱和集值控制器初始化
    BaseAdmCMultiDimiInit(pinocchioInterface, info, armEeKinematics, controller_nh); // base多维AdmC初始化
    MultiBoundedAdmittanceInit(pinocchioInterface, info, armEeKinematics, controller_nh);  // 机械臂多维饱和集值控制器初始化
    MultiAdmittanceInit(pinocchioInterface, info, armEeKinematics, controller_nh);  // 机械臂多维饱和常规控制器初始化
    ros::NodeHandle nh;
    BaseXforce_pub = nh.advertise<std_msgs::Float64>("/MPC_baseX_force", 1);
    BaseYforce_pub = nh.advertise<std_msgs::Float64>("/MPC_baseY_force", 1);
    BaseZforce_pub = nh.advertise<std_msgs::Float64>("/MPC_baseZ_force", 1);
    // dynamic reconfigure
    ros::NodeHandle nh_weight = ros::NodeHandle(controller_nh,"compliant");
    dynamic_srv_ = std::make_shared<dynamic_reconfigure::Server<qm_wbc::CompliantConfig>>(nh_weight);
    dynamic_reconfigure::Server<qm_wbc::CompliantConfig>::CallbackType cb = [this](auto&& PH1, auto&& PH2) {
        dynamicCallback(std::forward<decltype(PH1)>(PH1), std::forward<decltype(PH2)>(PH2));
    };
    dynamic_srv_->setCallback(cb);
}
// 单维度普通无力矩饱和导纳
void CompliantWbc::AdmittanceInit(const ocs2::PinocchioInterface &pinocchioInterface, ocs2::CentroidalModelInfo info,
                                         const ocs2::PinocchioEndEffectorKinematics &armEeKinematics, ros::NodeHandle &controller_nh) {
    ros::NodeHandle nh;
    admittance_controller2_ = std::make_shared<BoundedAdmittanceNoImp>(pinocchioInterface, info, armEeKinematics, controller_nh);
    admittance_controller2_->setJointIdx(2);
    Admittance_tau_cmd2_pub = nh.advertise<std_msgs::Float64>("/AdmittanceNoImp/tau_cmd_2", 1);

    admittance_controller1_ = std::make_shared<BoundedAdmittanceNoImp>(pinocchioInterface, info, armEeKinematics, controller_nh);
    admittance_controller1_->setJointIdx(1);
    Admittance_tau_cmd1_pub = nh.advertise<std_msgs::Float64>("/AdmittanceNoImp/tau_cmd_1", 1);
}
//常规多维导纳无微分包含初始化
void CompliantWbc::MultiAdmittanceInit(const ocs2::PinocchioInterface &pinocchioInterface, ocs2::CentroidalModelInfo info,
                                         const ocs2::PinocchioEndEffectorKinematics &armEeKinematics, ros::NodeHandle &controller_nh) {
    ros::NodeHandle nh;
    Multi_admittance_controller_ = std::make_shared<AdmittanceMultiDim>(pinocchioInterface, info, armEeKinematics, controller_nh);
    MultiAD_cmd1_pub = nh.advertise<std_msgs::Float64>("/MultiADM/direct_tau_cmd_1", 1);
    MultiAD_cmd2_pub = nh.advertise<std_msgs::Float64>("/MultiADM/direct_tau_cmd_2", 1);
}
// 机械臂多维集值算法初始化
void CompliantWbc::MultiBoundedAdmittanceInit(const ocs2::PinocchioInterface &pinocchioInterface, ocs2::CentroidalModelInfo info,
                                         const ocs2::PinocchioEndEffectorKinematics &armEeKinematics, ros::NodeHandle &controller_nh) {
    ros::NodeHandle nh;
    bounded_admittance_controller_ = std::make_shared<BoundedAdmittanceMultiDim>(pinocchioInterface, info, armEeKinematics, controller_nh);
    Multi_BA_tau_cmd1_pub = nh.advertise<std_msgs::Float64>("/MultiBA/tau_cmd_1", 1);
    Multi_BA_tau_cmd2_pub = nh.advertise<std_msgs::Float64>("/MultiBA/tau_cmd_2", 1);
    Multi_cmd1_pub = nh.advertise<std_msgs::Float64>("/MultiBA/direct_tau_cmd_1", 1);
    Multi_cmd2_pub = nh.advertise<std_msgs::Float64>("/MultiBA/direct_tau_cmd_2", 1);

    // 发布机械臂动力学惯性项和非线性项
    Multi_M_1_pub = nh.advertise<std_msgs::Float64>("/MultiBA/M_1", 1);
    Multi_M_2_pub = nh.advertise<std_msgs::Float64>("/MultiBA/M_2", 1);
    Multi_N_1_pub = nh.advertise<std_msgs::Float64>("/MultiBA/N_1", 1);
    Multi_N_2_pub = nh.advertise<std_msgs::Float64>("/MultiBA/N_2", 1);
    Gravity1_pub = nh.advertise<std_msgs::Float64>("/MultiBA/G_1", 1);
    Gravity2_pub = nh.advertise<std_msgs::Float64>("/MultiBA/G_2", 1);
}

// init bounded admittance of the base （单维度，只有x方向）
void CompliantWbc::BaseBoundedAdmittanceInit(const ocs2::PinocchioInterface &pinocchioInterface, ocs2::CentroidalModelInfo info,
                                             const ocs2::PinocchioEndEffectorKinematics &armEeKinematics, ros::NodeHandle &controller_nh) {
    bounded_admittance_controller_base_x_ = std::make_shared<BoundedAdmittanceWithK>(pinocchioInterface, info, armEeKinematics, controller_nh);
    bounded_admittance_controller_base_x_->setJointIdx(0);

//    bounded_admittance_controller_base_y_ = std::make_shared<BoundedAdmittanceWithK>(pinocchioInterface, info, armEeKinematics, controller_nh);
//    bounded_admittance_controller_base_y_->setJointIdx(1);
}

// init bounded admittance of the base （base多维集值算法初始化）
void CompliantWbc::BaseBAMultiDimiInit(const PinocchioInterface& pinocchioInterface, CentroidalModelInfo info,const PinocchioEndEffectorKinematics& armEeKinematics, ros::NodeHandle &controller_nh){
    Base_bounded_admittance_controller_ = std::make_shared<BaseBAMultiDim>(pinocchioInterface, info, armEeKinematics, controller_nh);
}

// init bounded admittance of the base （base多维AdmC算法初始化）
void CompliantWbc::BaseAdmCMultiDimiInit(const PinocchioInterface& pinocchioInterface, CentroidalModelInfo info,const PinocchioEndEffectorKinematics& armEeKinematics, ros::NodeHandle &controller_nh){
    Base_AdmC_controller_ = std::make_shared<BaseAdmCMultiDim>(pinocchioInterface, info, armEeKinematics, controller_nh);
}

void CompliantWbc::BaseAdmittanceInit(const ocs2::PinocchioInterface &pinocchioInterface, ocs2::CentroidalModelInfo info,
                                             const ocs2::PinocchioEndEffectorKinematics &armEeKinematics, ros::NodeHandle &controller_nh) {
    admittance_controller_base_x_ = std::make_shared<BaseBoundedAdmittanceNoImp>(pinocchioInterface, info, armEeKinematics, controller_nh);
    admittance_controller_base_x_->setJointIdx(0);

//    bounded_admittance_controller_base_y_ = std::make_shared<BoundedAdmittanceWithK>(pinocchioInterface, info, armEeKinematics, controller_nh);
//    bounded_admittance_controller_base_y_->setJointIdx(1);
}
// bounded admittance of joints control law   tau_feedback 应该设成0
void CompliantWbc::AdmittanceUpdate(const ocs2::vector_t &rbdStateMeasured, ocs2::scalar_t time,
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
    admittance_controller1_->setTorqueMax(tau_max_[1]);
    admittance_controller1_->setParam(M1_, K1_, D1_, L1_, M1x_, D1x_,K1x_);
    admittance_controller1_->setForceFeedback(0);
    admittance_controller1_->setTorqueExternal(torque_ext[1]);
    admittance_controller1_->setDesired(0, 0, q1_d);    //设置公式里的期望关节加速度，速度，位置
    //admittance_controller1_->setForceDesired(tau_desired[1]);
    admittance_controller1_->update(rbdStateMeasured, time, period);
    //tau_addmitance[1] = tmp[1];

    admittance_controller2_->setTorqueMax(tau_max_[2]);
    admittance_controller2_->setParam(M2_, K2_, D2_, L2_, M2x_, D2x_,K2x_);
    admittance_controller2_->setForceFeedback(0);
    admittance_controller2_->setTorqueExternal(torque_ext[2]);
    admittance_controller2_->setDesired(0, 0, q2_d);
    //admittance_controller2_->setForceDesired(tau_desired[2]);
    admittance_controller2_->update(rbdStateMeasured, time, period);
    //tau_addmitance[2] = tmp[1];
    //return tau_addmitance;
}

vector6_t CompliantWbc::MultiAdmittanceUpdate(const vector_t& rbdStateMeasured, scalar_t time, scalar_t period,vector_t imp) {
    vector6_t tau_feedback, tau_desired,  torque_ext;
    vector2_t torque_ext23;
    vector2_t tau_adm_joint23;
    vector6_t tau_admittance;
    tau_feedback.setZero();
    tau_desired.setZero();
    tau_admittance.setZero();
    torque_ext.setZero();

    torque_ext = getExternalArmTorque();
    torque_ext23 = torque_ext.block(1, 0, 2, 1);
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
    vector2_t q0(3.61, 0.86);
    vector2_t dot_q0(0.0, 0.0);
    vector2_t ddot_q0(0.0, 0.0);
    vector2_t torque_desired(0.0, 0.0);
    vector2_t torque_feedback(0.0, 0.0);
   
    // 得到2、3关节的粗略惯性项
    matrix2_t M_a = getArmInertiaTerm();
    matrix2_t C_a = getArmCoriolisTerm();
    vector6_t n_a = getArmNonlinearTerm();
    vector2_t n_arm23 = n_a.segment<2>(1);

    // matrix2_t M_a;
    // M_a.setZero();
    // M_a(0, 0) = MultiBA_M1_;  // 设置左上角元素
    // M_a(1, 1) = MultiBA_M2_;  // 设置右下角元素

    // vector2_t n_a(-8.71, 3.98);
    // matrix2_t C_a;
    // C_a.setZero();
    // C_a(0, 0) = MultiBA_C1_;  // 设置左上角元素
    // C_a(1, 1) = MultiBA_C2_;  // 设置右下角元素

    // 设置控制器的PD项
    matrix2_t Kp;
    Kp.setZero();
    Kp(0, 0) = MultiBA_Kp1_;  // 设置左上角元素
    Kp(1, 1) = MultiBA_Kp2_;  // 设置右下角元素

    matrix2_t Kd;
    Kd.setZero();
    Kd(0, 0) = MultiBA_Kd1_;  // 设置左上角元素
    Kd(1, 1) = MultiBA_Kd2_;  // 设置右下角元素

    // 设置控制器的滑模参数(矩阵形式)
    matrix2_t Lambda;
    Lambda.setZero();
    Lambda(0, 0) = Lambda1;  // 设置左上角元素
    Lambda(1, 1) = Lambda2;  // 设置右下角元素

    matrix2_t K1;
    K1.setZero();
    K1(0, 0) = K11;  // 设置左上角元素
    K1(1, 1) = K12;  // 设置右下角元素

    vector6_t G_a = getArmGravityTerm();
    vector2_t g_arm23 = G_a.segment<2>(1);

    
    std_msgs::Float64 M_msg1, M_msg2, N_msg1, N_msg2;
    M_msg1.data = M_a(0,0);
    M_msg2.data = M_a(1,1);
    N_msg1.data = n_a(1);
    N_msg2.data = n_a(2);
    Multi_M_1_pub.publish(M_msg1);
    Multi_M_2_pub.publish(M_msg2);
    Multi_N_1_pub.publish(N_msg1);
    Multi_N_2_pub.publish(N_msg2);   
    //        ros::Publisher Multi_M_1_pub, Multi_M_2_pub, Multi_N_1_pub, Multi_N_2_pub;
    //     std_msgs::Float64 torque_msg1,torque_msg2;
    // torque_msg1.data = ba_tau[0];
    // torque_msg2.data = ba_tau[1];
    // Multi_cmd1_pub.publish(torque_msg1);
    // Multi_cmd2_pub.publish(torque_msg2);
    // 多维集值导纳控制器
    vector_t tmp;
    Multi_admittance_controller_ -> setTorqueMax(tau_max_[1], tau_max_[2]);
    Multi_admittance_controller_ -> setProxyParam(MultiBA_M1x_, MultiBA_M2x_, MultiBA_D1x_, MultiBA_D2x_, MultiBA_K1x_, MultiBA_K2x_);
    Multi_admittance_controller_ -> setScalarSlidingModeParam(SM_lambda, SM_K1);
    Multi_admittance_controller_ -> setMatrixSlidingModeParam(Lambda, K1);
    Multi_admittance_controller_ -> setIDControllerParam(M_a, C_a, g_arm23);
    Multi_admittance_controller_ -> setPDParam(Kp, Kd);
    Multi_admittance_controller_ -> setDesired(ddot_q0, dot_q0, q0);
    Multi_admittance_controller_ -> setTorqueDesired(torque_desired);
    Multi_admittance_controller_ -> setTorqueFeedback(torque_feedback);
    Multi_admittance_controller_ -> setTorqueExternal(torque_ext23);
    tau_adm_joint23 = Multi_admittance_controller_ -> update(rbdStateMeasured, time, period);
    tau_admittance[1] = tau_adm_joint23[0];
    tau_admittance[2] = tau_adm_joint23[1];
    return tau_admittance;

/*
    
    //一维集值导纳控制器   只考虑机械臂2，3关节的纯柔顺
    //vector_t tmp;
    admittance_controller1_->setTorqueMax(tau_max_[1]);
    admittance_controller1_->setParam(M1_, K1_, D1_, L1_, M1x_, D1x_,K1x_);
    admittance_controller1_->setForceFeedback(0);
    admittance_controller1_->setTorqueExternal(torque_ext[1]);
    admittance_controller1_->setDesired(0, 0, q1_d);    //设置公式里的期望关节加速度，速度，位置
    //admittance_controller1_->setForceDesired(tau_desired[1]);
    admittance_controller1_->update(rbdStateMeasured, time, period);
    //tau_addmitance[1] = tmp[1];

    admittance_controller2_->setTorqueMax(tau_max_[2]);
    admittance_controller2_->setParam(M2_, K2_, D2_, L2_, M2x_, D2x_,K2x_);
    admittance_controller2_->setForceFeedback(0);
    admittance_controller2_->setTorqueExternal(torque_ext[2]);
    admittance_controller2_->setDesired(0, 0, q2_d);
    //admittance_controller2_->setForceDesired(tau_desired[2]);
    admittance_controller2_->update(rbdStateMeasured, time, period);
    //tau_addmitance[2] = tmp[1];
*/
}
vector6_t CompliantWbc::MultiBoundedAdmittanceUpdate(const vector_t& rbdStateMeasured, scalar_t time, scalar_t period,vector_t imp) {
    vector6_t tau_feedback, tau_desired,  torque_ext;
    vector2_t torque_ext23;
    vector2_t tau_adm_joint23;
    vector6_t tau_admittance;
    tau_feedback.setZero();
    tau_desired.setZero();
    tau_admittance.setZero();
    torque_ext.setZero();

    torque_ext = getExternalArmTorque();
    torque_ext23 = torque_ext.block(1, 0, 2, 1);
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
    vector2_t q0(3.61, 0.86);
    vector2_t dot_q0(0.0, 0.0);
    vector2_t ddot_q0(0.0, 0.0);
    vector2_t torque_desired(0.0, 0.0);
    vector2_t torque_feedback(0.0, 0.0);
   
    // 得到2、3关节的粗略惯性项
    matrix2_t M_a = getArmInertiaTerm();
    matrix2_t C_a = getArmCoriolisTerm();
    vector6_t n_a = getArmNonlinearTerm();
    vector2_t n_arm23 = n_a.segment<2>(1);

    // matrix2_t M_a;
    // M_a.setZero();
    // M_a(0, 0) = MultiBA_M1_;  // 设置左上角元素
    // M_a(1, 1) = MultiBA_M2_;  // 设置右下角元素

    // vector2_t n_a(-8.71, 3.98);
    // matrix2_t C_a;
    // C_a.setZero();
    // C_a(0, 0) = MultiBA_C1_;  // 设置左上角元素
    // C_a(1, 1) = MultiBA_C2_;  // 设置右下角元素

    // 设置控制器的PD项
    matrix2_t Kp;
    Kp.setZero();
    Kp(0, 0) = MultiBA_Kp1_;  // 设置左上角元素
    Kp(1, 1) = MultiBA_Kp2_;  // 设置右下角元素

    matrix2_t Kd;
    Kd.setZero();
    Kd(0, 0) = MultiBA_Kd1_;  // 设置左上角元素
    Kd(1, 1) = MultiBA_Kd2_;  // 设置右下角元素

    // 设置控制器的滑模参数(矩阵形式)
    matrix2_t Lambda;
    Lambda.setZero();
    Lambda(0, 0) = Lambda1;  // 设置左上角元素
    Lambda(1, 1) = Lambda2;  // 设置右下角元素

    // 设置矩阵PID的参数
    matrix2_t Mpid, Kpid, Bpid, Lpid;
    Mpid.setZero();
    Kpid.setZero();
    Bpid.setZero();
    Lpid.setZero();
    Mpid(0, 0) = PID_M1;  // 设置左上角元素
    Mpid(1, 1) = PID_M2;  // 设置右下角元素
    Kpid(0, 0) = PID_K1;  // 设置左上角元素
    Kpid(1, 1) = PID_K2;  // 设置右下角元素
    Bpid(0, 0) = PID_B1;  // 设置左上角元素
    Bpid(1, 1) = PID_B2;  // 设置右下角元素
    Lpid(0, 0) = PID_L1;  // 设置左上角元素
    Lpid(1, 1) = PID_L2;  // 设置右下角元素

    matrix2_t K1;
    K1.setZero();
    K1(0, 0) = K11;  // 设置左上角元素
    K1(1, 1) = K12;  // 设置右下角元素

    vector6_t G_a = getArmGravityTerm();
    vector2_t g_arm23 = G_a.segment<2>(1);

    
    std_msgs::Float64 M_msg1, M_msg2, N_msg1, N_msg2;
    M_msg1.data = M_a(0,0);
    M_msg2.data = M_a(1,1);
    N_msg1.data = n_a(1);
    N_msg2.data = n_a(2);
    Multi_M_1_pub.publish(M_msg1);
    Multi_M_2_pub.publish(M_msg2);
    Multi_N_1_pub.publish(N_msg1);
    Multi_N_2_pub.publish(N_msg2);   
    //        ros::Publisher Multi_M_1_pub, Multi_M_2_pub, Multi_N_1_pub, Multi_N_2_pub;
    //     std_msgs::Float64 torque_msg1,torque_msg2;
    // torque_msg1.data = ba_tau[0];
    // torque_msg2.data = ba_tau[1];
    // Multi_cmd1_pub.publish(torque_msg1);
    // Multi_cmd2_pub.publish(torque_msg2);
    // 多维集值导纳控制器
    vector_t tmp;
    bounded_admittance_controller_ -> setTorqueMax(tau_max_[1], tau_max_[2]);
    bounded_admittance_controller_ -> setProxyParam(MultiBA_M1x_, MultiBA_M2x_, MultiBA_D1x_, MultiBA_D2x_, MultiBA_K1x_, MultiBA_K2x_);
    bounded_admittance_controller_ -> setScalarSlidingModeParam(SM_lambda, SM_K1);
    bounded_admittance_controller_ -> setMatrixSlidingModeParam(Lambda, K1);
    bounded_admittance_controller_ -> setIDControllerParam(M_a, C_a, g_arm23);
    bounded_admittance_controller_ -> setPDParam(Kp, Kd);
    bounded_admittance_controller_ -> setPIDControllerParam(Mpid, Kpid, Bpid, Lpid);
    bounded_admittance_controller_ -> setDesired(ddot_q0, dot_q0, q0);
    bounded_admittance_controller_ -> setTorqueDesired(torque_desired);
    bounded_admittance_controller_ -> setTorqueFeedback(torque_feedback);
    bounded_admittance_controller_ -> setTorqueExternal(torque_ext23);
    tau_adm_joint23 = bounded_admittance_controller_ -> update(rbdStateMeasured, time, period);
    tau_admittance[1] = tau_adm_joint23[0];
    tau_admittance[2] = tau_adm_joint23[1];
    return tau_admittance;

/*
    
    //一维集值导纳控制器   只考虑机械臂2，3关节的纯柔顺
    //vector_t tmp;
    admittance_controller1_->setTorqueMax(tau_max_[1]);
    admittance_controller1_->setParam(M1_, K1_, D1_, L1_, M1x_, D1x_,K1x_);
    admittance_controller1_->setForceFeedback(0);
    admittance_controller1_->setTorqueExternal(torque_ext[1]);
    admittance_controller1_->setDesired(0, 0, q1_d);    //设置公式里的期望关节加速度，速度，位置
    //admittance_controller1_->setForceDesired(tau_desired[1]);
    admittance_controller1_->update(rbdStateMeasured, time, period);
    //tau_addmitance[1] = tmp[1];

    admittance_controller2_->setTorqueMax(tau_max_[2]);
    admittance_controller2_->setParam(M2_, K2_, D2_, L2_, M2x_, D2x_,K2x_);
    admittance_controller2_->setForceFeedback(0);
    admittance_controller2_->setTorqueExternal(torque_ext[2]);
    admittance_controller2_->setDesired(0, 0, q2_d);
    //admittance_controller2_->setForceDesired(tau_desired[2]);
    admittance_controller2_->update(rbdStateMeasured, time, period);
    //tau_addmitance[2] = tmp[1];
*/
}
// Base单维集值导纳只有X方向（师兄方案）
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

// Base XY方向多维集值导纳
vector6_t CompliantWbc::BaseBAMultiDimUpdate(const vector_t& rbdStateMeasured, scalar_t time, scalar_t period, vector_t imp, scalar_t force_z){

    vector6_t tau_feedback, tau_desired,  torque_ext;
    vector2_t torque_extXY;
    vector2_t tau_adm_baseXY;
    vector6_t tau_admittance;
    tau_feedback.setZero();
    tau_desired.setZero();
    tau_admittance.setZero();
    torque_ext.setZero();

    torque_ext = getExternalBaseTorque();
    torque_extXY = torque_ext.block(0, 0, 2, 1);
    // get the desired position from the reference
    scalar_t base_xd;
    scalar_t base_yd;

    if(referenceManagerPtr_ == nullptr){
        throw std::runtime_error("[CompliantWbc] ReferenceManager pointer cannot be a nullptr!");
    } else{
        const auto& targetTrajectories = referenceManagerPtr_->getTargetTrajectories();
        const auto& timeTrajectory = targetTrajectories.timeTrajectory;
        const auto& stateTrajectory = targetTrajectories.stateTrajectory;   //维度可以参考src/qm_control/qm_controllers/src/QMController.cpp  119行
        const size_t size = timeTrajectory.size();
        base_xd = stateTrajectory[size-1][6];
        base_yd = stateTrajectory[size-1][7];
    }
    vector2_t pos0(base_xd, base_yd);
    vector2_t dot_pos0(0.0, 0.0);
    vector2_t ddot_pos0(0.0, 0.0);
    vector2_t torque_desired(0.0, 0.0);
    vector2_t torque_feedback(0.0, 0.0);
   
    // 得到base的粗略动力学项
    // matrix2_t M_a = getBaseInertiaTerm();
    // matrix2_t C_a = getBaseCoriolisTerm();
    vector6_t n_a = getBaseNonlinearTerm();
    vector2_t n_baseXY = n_a.segment<2>(0);

    matrix2_t M_a;
    M_a.setZero();
    M_a(0, 0) = MultiBA_M1_;  // 设置左上角元素
    M_a(1, 1) = MultiBA_M2_;  // 设置右下角元素

    // vector2_t n_a(-8.71, 3.98);
    matrix2_t C_a;
    C_a.setZero();
    C_a(0, 0) = MultiBA_C1_;  // 设置左上角元素
    C_a(1, 1) = MultiBA_C2_;  // 设置右下角元素

    // 设置控制器的滑模参数(矩阵形式)
    matrix2_t Lambda;
    Lambda.setZero();
    Lambda(0, 0) = Base_Lambdax;  // 设置左上角元素
    Lambda(1, 1) = Base_Lambday;  // 设置右下角元素

    // 设置矩阵PID的参数
    matrix2_t Mpid, Kpid, Bpid, Lpid;
    Mpid.setZero();
    Kpid.setZero();
    Bpid.setZero();
    Lpid.setZero();
    Mpid(0, 0) = PID_M1;  // 设置左上角元素
    Mpid(1, 1) = PID_M2;  // 设置右下角元素
    Kpid(0, 0) = PID_K1;  // 设置左上角元素
    Kpid(1, 1) = PID_K2;  // 设置右下角元素
    Bpid(0, 0) = PID_B1;  // 设置左上角元素
    Bpid(1, 1) = PID_B2;  // 设置右下角元素
    Lpid(0, 0) = PID_L1;  // 设置左上角元素
    Lpid(1, 1) = PID_L2;  // 设置右下角元素

    matrix2_t K1;
    K1.setZero();
    K1(0, 0) = Base_K1x;  // 设置左上角元素
    K1(1, 1) = Base_K1y;  // 设置右下角元素

    vector6_t G_a = getBaseGravityTerm();
    vector2_t g_BaseXY = G_a.segment<2>(0);

    
    std_msgs::Float64 M_msg1, M_msg2, N_msg1, N_msg2;
    M_msg1.data = M_a(0,0);
    M_msg2.data = M_a(1,1);
    N_msg1.data = n_a(1);
    N_msg2.data = n_a(2);
    Multi_M_1_pub.publish(M_msg1);
    Multi_M_2_pub.publish(M_msg2);
    Multi_N_1_pub.publish(N_msg1);
    Multi_N_2_pub.publish(N_msg2);   
    //        ros::Publisher Multi_M_1_pub, Multi_M_2_pub, Multi_N_1_pub, Multi_N_2_pub;
    //     std_msgs::Float64 torque_msg1,torque_msg2;
    // torque_msg1.data = ba_tau[0];
    // torque_msg2.data = ba_tau[1];
    // Multi_cmd1_pub.publish(torque_msg1);
    // Multi_cmd2_pub.publish(torque_msg2);
    // 多维集值导纳控制器
    vector_t tmp;
    Base_bounded_admittance_controller_ -> setTorqueMax(force_z * mu_, force_z * mu_);
    Base_bounded_admittance_controller_ -> setProxyParam(Mbase_px_, Mbase_py_, Bbase_px_, Bbase_py_, Kbase_px_, Kbase_py_);
    Base_bounded_admittance_controller_ -> setMatrixSlidingModeParam(Lambda, K1);
    Base_bounded_admittance_controller_ -> setPIDControllerParam(Mpid, Kpid, Bpid, Lpid);
    Base_bounded_admittance_controller_ -> setIDControllerParam(M_a, C_a, g_BaseXY);
    Base_bounded_admittance_controller_ -> setDesired(ddot_pos0, dot_pos0, pos0);
    Base_bounded_admittance_controller_ -> setTorqueDesired(torque_desired);
    Base_bounded_admittance_controller_ -> setTorqueFeedback(torque_feedback);
    Base_bounded_admittance_controller_ -> setTorqueExternal(torque_extXY);
    tau_adm_baseXY = Base_bounded_admittance_controller_ -> update(rbdStateMeasured, time, period);
    tau_admittance[0] = tau_adm_baseXY[0];
    tau_admittance[1] = tau_adm_baseXY[1];
    return tau_admittance;
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

//base AdmC
vector6_t CompliantWbc::BaseAdmCMultiDimUpdate(const vector_t& rbdStateMeasured, scalar_t time, scalar_t period, vector_t imp, scalar_t force_z){
    vector6_t tau_feedback, tau_desired,  torque_ext;
    vector2_t torque_extXY;
    vector2_t tau_adm_baseXY;
    vector6_t tau_admittance;
    tau_feedback.setZero();
    tau_desired.setZero();
    tau_admittance.setZero();
    torque_ext.setZero();

    torque_ext = getExternalBaseTorque();
    torque_extXY = torque_ext.block(0, 0, 2, 1);
    // get the desired position from the reference
    scalar_t base_xd;
    scalar_t base_yd;

    if(referenceManagerPtr_ == nullptr){
        throw std::runtime_error("[CompliantWbc] ReferenceManager pointer cannot be a nullptr!");
    } else{
        const auto& targetTrajectories = referenceManagerPtr_->getTargetTrajectories();
        const auto& timeTrajectory = targetTrajectories.timeTrajectory;
        const auto& stateTrajectory = targetTrajectories.stateTrajectory;   //维度可以参考src/qm_control/qm_controllers/src/QMController.cpp  119行
        const size_t size = timeTrajectory.size();
        base_xd = stateTrajectory[size-1][6];
        base_yd = stateTrajectory[size-1][7];
    }
    vector2_t pos0(base_xd, base_yd);
    vector2_t dot_pos0(0.0, 0.0);
    vector2_t ddot_pos0(0.0, 0.0);
    vector2_t torque_desired(0.0, 0.0);
    vector2_t torque_feedback(0.0, 0.0);
   
    // 得到base的粗略动力学项
    matrix2_t M_a = getBaseInertiaTerm();
    matrix2_t C_a = getBaseCoriolisTerm();
    vector6_t n_a = getBaseNonlinearTerm();
    vector2_t n_baseXY = n_a.segment<2>(0);

    // matrix2_t M_a;
    // M_a.setZero();
    // M_a(0, 0) = MultiBA_M1_;  // 设置左上角元素
    // M_a(1, 1) = MultiBA_M2_;  // 设置右下角元素

    // vector2_t n_a(-8.71, 3.98);
    // matrix2_t C_a;
    // C_a.setZero();
    // C_a(0, 0) = MultiBA_C1_;  // 设置左上角元素
    // C_a(1, 1) = MultiBA_C2_;  // 设置右下角元素

    // 设置控制器的滑模参数(矩阵形式)
    matrix2_t Lambda;
    Lambda.setZero();
    Lambda(0, 0) = Base_Lambdax;  // 设置左上角元素
    Lambda(1, 1) = Base_Lambday;  // 设置右下角元素

    // 设置矩阵PID的参数
    matrix2_t Mpid, Kpid, Bpid, Lpid;
    Mpid.setZero();
    Kpid.setZero();
    Bpid.setZero();
    Lpid.setZero();
    Mpid(0, 0) = PID_M1;  // 设置左上角元素
    Mpid(1, 1) = PID_M2;  // 设置右下角元素
    Kpid(0, 0) = PID_K1;  // 设置左上角元素
    Kpid(1, 1) = PID_K2;  // 设置右下角元素
    Bpid(0, 0) = PID_B1;  // 设置左上角元素
    Bpid(1, 1) = PID_B2;  // 设置右下角元素
    Lpid(0, 0) = PID_L1;  // 设置左上角元素
    Lpid(1, 1) = PID_L2;  // 设置右下角元素

    matrix2_t K1;
    K1.setZero();
    K1(0, 0) = Base_K1x;  // 设置左上角元素
    K1(1, 1) = Base_K1y;  // 设置右下角元素

    vector6_t G_a = getBaseGravityTerm();
    vector2_t g_BaseXY = G_a.segment<2>(0);

    
    std_msgs::Float64 M_msg1, M_msg2, N_msg1, N_msg2;
    M_msg1.data = M_a(0,0);
    M_msg2.data = M_a(1,1);
    N_msg1.data = n_a(1);
    N_msg2.data = n_a(2);
    Multi_M_1_pub.publish(M_msg1);
    Multi_M_2_pub.publish(M_msg2);
    Multi_N_1_pub.publish(N_msg1);
    Multi_N_2_pub.publish(N_msg2);   
    //        ros::Publisher Multi_M_1_pub, Multi_M_2_pub, Multi_N_1_pub, Multi_N_2_pub;
    //     std_msgs::Float64 torque_msg1,torque_msg2;
    // torque_msg1.data = ba_tau[0];
    // torque_msg2.data = ba_tau[1];
    // Multi_cmd1_pub.publish(torque_msg1);
    // Multi_cmd2_pub.publish(torque_msg2);
    // 多维集值导纳控制器
    vector_t tmp;
    Base_AdmC_controller_ -> setTorqueMax(force_z * mu_, force_z * mu_);
    Base_AdmC_controller_ -> setProxyParam(Mbase_px_, Mbase_py_, Bbase_px_, Bbase_py_, Kbase_px_, Kbase_py_);
    Base_AdmC_controller_ -> setMatrixSlidingModeParam(Lambda, K1);
    Base_AdmC_controller_ -> setPIDControllerParam(Mpid, Kpid, Bpid, Lpid);
    Base_AdmC_controller_ -> setIDControllerParam(M_a, C_a, g_BaseXY);
    Base_AdmC_controller_ -> setDesired(ddot_pos0, dot_pos0, pos0);
    Base_AdmC_controller_ -> setTorqueDesired(torque_desired);
    Base_AdmC_controller_ -> setTorqueFeedback(torque_feedback);
    Base_AdmC_controller_ -> setTorqueExternal(torque_extXY);
    tau_adm_baseXY = Base_AdmC_controller_ -> update(rbdStateMeasured, time, period);
    tau_admittance[0] = tau_adm_baseXY[0];
    tau_admittance[1] = tau_adm_baseXY[1];
    return tau_admittance;
} 

/**
 * @brief 常规多维饱和导纳  
 * 
 * @param stateDesired 
 * @param inputDesired 
 * @param rbdStateMeasured 
 * @param mode 
 * @param period 
 * @param time 
 * @return vector_t 
 */

vector_t CompliantWbc::MultiAdmittanceControl(const ocs2::vector_t &stateDesired, const ocs2::vector_t &inputDesired,
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
    //AdmittanceUpdate(rbdStateMeasured, time, period);
    vector_t ba_tau = MultiAdmittanceUpdate(rbdStateMeasured, time, period, imp);
    //重力补偿
    vector6_t G_a = getArmGravityTerm();
    vector6_t nle_a = getArmNonlinearTerm();
    // ba_tau = ba_tau + nle_a;
    // ba_tau = ba_tau + G_a;

    vector6_t ImpArm_tau  = imp.tail(6);
    vector6_t compliant_tau = imp.tail(6);

    for (int i = begin_; i < end_; ++i) {        //手动使能多维导纳关节           
        compliant_tau[i] = ba_tau[i];
    }
    std_msgs::Float64 torque_msg1, torque_msg2;
    torque_msg1.data = ba_tau[1];
    torque_msg2.data = ba_tau[2];
    MultiAD_cmd1_pub.publish(torque_msg1);
    MultiAD_cmd2_pub.publish(torque_msg2);
    // base 仍采用师兄的先阻抗再导纳  然后只采用跟踪base的proxy
    scalar_t force_z = 0;                      
    for (size_t i = 0; i < 4; ++i) {
        force_z += inputDesired[3*i+2];
    } 
    scalar_t force_x = 0;                      
    for (size_t i = 0; i < 4; ++i) {
        force_x += inputDesired[3*i];
    } 
    scalar_t force_y = 0;                      
    for (size_t i = 0; i < 4; ++i) {
        force_y += inputDesired[3*i+1];
    } 
    //发布MPC计算出来的XYZ三个方向的足底合力
    std_msgs::Float64 baseXforce_msg, baseYforce_msg, baseZforce_msg;
    baseXforce_msg.data  = force_x;
    baseYforce_msg.data  = force_y;
    baseZforce_msg.data  = force_z;
    BaseXforce_pub.publish(baseXforce_msg);
    BaseYforce_pub.publish(baseYforce_msg);
    BaseZforce_pub.publish(baseZforce_msg);
    //vector_t tau_cmd = BaseBoundedAdmittanceUpdate(rbdStateMeasured, time, period, imp, force_z);
    vector_t Base_tau_cmd = BaseAdmCMultiDimUpdate(rbdStateMeasured, time, period, imp, force_z).topRows(2);
    vector_t proxy_x = Base_AdmC_controller_->getBaseXProxyState();  
    vector_t proxy_y = Base_AdmC_controller_->getBaseYProxyState();
    //vector_t proxy_x = bounded_admittance_controller_base_x_->getProxyState();    // 多维暂时先对base采用先阻抗后导纳，然后只跟踪proxy
    vector_t proxy_1 = Multi_admittance_controller_ -> getJoint1ProxyState();
    vector_t proxy_2 = Multi_admittance_controller_ -> getJoint2ProxyState();

    Task task0 = formulateFloatingBaseEomWithEEForceTask() + formulateTorqueLimitsTaskWithEEForceTask()
                    + formulateNoContactMotionTask() + formulateFrictionConeTask() ;
    // Task task0 = formulateFloatingBaseEomWithEEForceTask() + formulateTorqueLimitsTaskWithEEForceTask()
    //                 + formulateNoContactMotionTask() + formulateFrictionConeTask() + formulateJoint1SetPostitionTask(3.61, 0, 0)+ formulateJoint0SetPostitionTask(3.14, 0, 0);
    // Task task0 = formulateFloatingBaseEomWithEEForceTask() + formulateTorqueLimitsTaskWithEEForceTask()
    //                 + formulateNoContactMotionTask() + formulateFrictionConeTask() + formulateJoint2SetPostitionTask(0.86, 0, 0) + formulateJoint0SetPostitionTask(3.14, 0, 0);
    Task taskBA = formulateManipulatorTorqueTask(compliant_tau.tail(6));  // 任务BA对关节2、3采用多维导纳，对另外四个关节仍然采用先阻抗再导纳（师兄方案）
    Task taskBaseProxy = formulateBaseXMotionTrackingTask(proxy_x[0], proxy_x[1], proxy_x[2]) + formulateBaseYMotionTrackingTask(proxy_y[0], proxy_y[1], proxy_y[2]);  //proxy_x[0] [1] [2]分别代表proxy的位置 速度 加速度
    //Task taskBaseProxy = formulateBaseXMotionTrackingTask(proxy_x[0], proxy_x[1], proxy_x[2]);
    // Task taskBaseProxy = formulateBaseYMotionTrackingTask(proxy_y[0], proxy_y[1], proxy_y[2]);
    Task task1 = formulateBaseHeightMotionTask() + formulateBaseAngularMotionTask() + formulateSwingLegTask() * 100
                + formulateEeAngularMotionTrackingTask() + taskBA + taskBaseProxy /*formulateMultiJoint2ProxyTrackingTask(proxy_2[0],proxy_2[1],proxy_2[2])*/ /*+ formulateBaseYLinearMotionTask()*/
                /*+ formulateMultiJoint12ProxyTrackingTask(proxy_1[0],proxy_1[1],proxy_1[2],proxy_2[0],proxy_2[1],proxy_2[2])*/ /*formulateMulitJoint1ProxyTrackingTask(proxy_1[0],proxy_1[1],proxy_1[2])*/; 
    // 对Task3增加跟踪proxy的子任务
    // Task task3 = formulateContactForceTask(inputDesired) + formulateMultiJoint12ProxyTrackingTask(proxy_1[0],proxy_1[1],proxy_1[2],proxy_2[0],proxy_2[1],proxy_2[2]);
    Task task3 = formulateZContactForceTask(inputDesired) + formulateXYContactForceTaskWithCompliant(inputDesired, Base_tau_cmd);  // XY 方向进行柔顺，Z方向的足底力跟踪MPC算出来的
    //Task task3 = formulateContactForceTask(inputDesired);
    HoQp hoQp(task3, std::make_shared<HoQp>(task1, std::make_shared<HoQp>(task0)));
    vector_t x_optimal = hoQp.getSolutions();


    return WbcBase::updateCmdWithEEForce(x_optimal);
}

/**
 * @brief 力矩饱和导纳控制实现（多维离散化）
 * 
 * @param stateDesired 
 * @param inputDesired 
 * @param rbdStateMeasured 
 * @param mode 
 * @param period 
 * @param time 
 * @return vector_t 
 */
vector_t CompliantWbc::MultiBoundedAdmittanceControl(const ocs2::vector_t &stateDesired, const ocs2::vector_t &inputDesired,
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
    //AdmittanceUpdate(rbdStateMeasured, time, period);
    vector_t ba_tau = MultiBoundedAdmittanceUpdate(rbdStateMeasured, time, period, imp);
    //重力补偿
    vector6_t G_a = getArmGravityTerm();
    vector6_t nle_a = getArmNonlinearTerm();
    // ba_tau = ba_tau + nle_a;
    // ba_tau = ba_tau + G_a;
    // 发布重力话题
    std_msgs::Float64 G_msg1, G_msg2;
    G_msg1.data = G_a[1];
    G_msg2.data = G_a[2];
    Gravity1_pub.publish(G_msg1);
    Gravity2_pub.publish(G_msg2);

    vector6_t ImpArm_tau  = imp.tail(6);
    vector6_t compliant_tau = imp.tail(6);

    for (int i = begin_; i < end_; ++i) {        //手动使能多维导纳关节           
        compliant_tau[i] = ba_tau[i];
    }
    std_msgs::Float64 torque_msg1, torque_msg2;
    torque_msg1.data = ba_tau[1];
    torque_msg2.data = ba_tau[2];
    Multi_cmd1_pub.publish(torque_msg1);
    Multi_cmd2_pub.publish(torque_msg2);
    // base 仍采用师兄的先阻抗再导纳  然后只采用跟踪base的proxy
    scalar_t force_z = 0;                      
    for (size_t i = 0; i < 4; ++i) {
        force_z += inputDesired[3*i+2];
    } 
    scalar_t force_x = 0;                      
    for (size_t i = 0; i < 4; ++i) {
        force_x += inputDesired[3*i];
    } 
    scalar_t force_y = 0;                      
    for (size_t i = 0; i < 4; ++i) {
        force_y += inputDesired[3*i+1];
    } 
    //发布MPC计算出来的XYZ三个方向的足底合力
    std_msgs::Float64 baseXforce_msg, baseYforce_msg, baseZforce_msg;
    baseXforce_msg.data  = force_x;
    baseYforce_msg.data  = force_y;
    baseZforce_msg.data  = force_z;
    BaseXforce_pub.publish(baseXforce_msg);
    BaseYforce_pub.publish(baseYforce_msg);
    BaseZforce_pub.publish(baseZforce_msg);
    
    //vector_t tau_cmd = BaseBoundedAdmittanceUpdate(rbdStateMeasured, time, period, imp, force_z);
    vector_t Base_tau_cmd = BaseBAMultiDimUpdate(rbdStateMeasured, time, period, imp, force_z).topRows(2);
    // vector_t Base_tau_cmd = BaseBAMultiDimUpdate(rbdStateMeasured, time, period, imp, force_z);
    vector_t proxy_x = Base_bounded_admittance_controller_->getBaseXProxyState();  
    vector_t proxy_y = Base_bounded_admittance_controller_->getBaseYProxyState();   
    vector_t proxy_1 = bounded_admittance_controller_ -> getJoint1ProxyState();
    vector_t proxy_2 = bounded_admittance_controller_ -> getJoint2ProxyState();

    Task task0 = formulateFloatingBaseEomWithEEForceTask() + formulateTorqueLimitsTaskWithEEForceTask()
                    + formulateNoContactMotionTask() + formulateFrictionConeTask() ;
    // Task task0 = formulateFloatingBaseEomWithEEForceTask() + formulateTorqueLimitsTaskWithEEForceTask()
    //                 + formulateNoContactMotionTask() + formulateFrictionConeTask() + formulateJoint1SetPostitionTask(3.61, 0, 0)+ formulateJoint0SetPostitionTask(3.14, 0, 0);
    // Task task0 = formulateFloatingBaseEomWithEEForceTask() + formulateTorqueLimitsTaskWithEEForceTask()
    //                 + formulateNoContactMotionTask() + formulateFrictionConeTask() + formulateJoint2SetPostitionTask(0.86, 0, 0) + formulateJoint0SetPostitionTask(3.14, 0, 0);
    Task taskBA = formulateManipulatorTorqueTask(compliant_tau.tail(6));  // 任务BA对关节2、3采用多维导纳，对另外四个关节仍然采用先阻抗再导纳（师兄方案）
    Task taskBaseProxy = formulateBaseXMotionTrackingTask(proxy_x[0], proxy_x[1], proxy_x[2]) + formulateBaseYMotionTrackingTask(proxy_y[0], proxy_y[1], proxy_y[2]);  //proxy_x[0] [1] [2]分别代表proxy的位置 速度 加速度
    //Task taskBaseProxy = formulateBaseXMotionTrackingTask(proxy_x[0], proxy_x[1], proxy_x[2]);
    // Task taskBaseProxy = formulateBaseYMotionTrackingTask(proxy_y[0], proxy_y[1], proxy_y[2]);
    Task task1 = formulateBaseHeightMotionTask() + formulateBaseAngularMotionTask() + formulateSwingLegTask() * 100
                + formulateEeAngularMotionTrackingTask() + taskBA + taskBaseProxy /*formulateMultiJoint2ProxyTrackingTask(proxy_2[0],proxy_2[1],proxy_2[2])*/ /*+ formulateBaseXYLinearMotionTask()*//*+ formulateXYContactForceTaskWithCompliant(inputDesired, Base_tau_cmd)*/
                /*+ formulateMultiJoint12ProxyTrackingTask(proxy_1[0],proxy_1[1],proxy_1[2],proxy_2[0],proxy_2[1],proxy_2[2])*/ /*formulateMulitJoint1ProxyTrackingTask(proxy_1[0],proxy_1[1],proxy_1[2])*//*+formulateBaseYLinearMotionTask()*/; 
    // 对Task3增加跟踪proxy的子任务
    // Task task3 = formulateContactForceTask(inputDesired) + formulateMultiJoint12ProxyTrackingTask(proxy_1[0],proxy_1[1],proxy_1[2],proxy_2[0],proxy_2[1],proxy_2[2]);
    //Task task3 = formulateZContactForceTask(inputDesired) + formulateXYContactTorqueTaskWithCompliant(inputDesired, Base_tau_cmd);  // XY 方向进行柔顺，Z方向的足底力跟踪MPC算出来的
    //Task task3 = formulateZContactForceTask(inputDesired) ;  // XY 方向进行柔顺，Z方向的足底力跟踪MPC算出来的
    Task task3 = formulateZContactForceTask(inputDesired) + formulateXYContactForceTaskWithCompliant(inputDesired, Base_tau_cmd);  // XY 方向进行柔顺，Z方向的足底力跟踪MPC算出来的
    // Task task3 = formulateContactForceTask(inputDesired);
    HoQp hoQp(task3, std::make_shared<HoQp>(task1, std::make_shared<HoQp>(task0)));
    vector_t x_optimal = hoQp.getSolutions();


    return WbcBase::updateCmdWithEEForce(x_optimal);
/*
    //提取2、3关节的proxy状态向量（位置 速度 加速度）
    vector_t proxy_1  = admittance_controller1_->getProxyState();
    vector_t proxy_2  = admittance_controller2_->getProxyState();

    
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

*/

}

/**
 * @brief 力矩饱和导纳控制实现（多维离散化）机械臂的两个关节只追踪proxy
 * 
 * @param stateDesired 
 * @param inputDesired 
 * @param rbdStateMeasured 
 * @param mode 
 * @param period 
 * @param time 
 * @return vector_t 
 */
vector_t CompliantWbc::MultiBAProxyTrackingControl(const ocs2::vector_t &stateDesired, const ocs2::vector_t &inputDesired,
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
    //AdmittanceUpdate(rbdStateMeasured, time, period);
    vector6_t ba_tau = MultiBoundedAdmittanceUpdate(rbdStateMeasured, time, period, imp);
    vector6_t ImpArm_tau  = imp.tail(6);
    vector6_t compliant_tau = imp.tail(6);

    for (int i = begin_; i < end_; ++i) {        //手动使能多维导纳关节           
        compliant_tau[i] = ba_tau[i];
    }
    std_msgs::Float64 torque_msg1, torque_msg2;
    torque_msg1.data = ba_tau[1];
    torque_msg2.data = ba_tau[2];
    Multi_cmd1_pub.publish(torque_msg1);
    Multi_cmd2_pub.publish(torque_msg2);

    //提取2、3关节的proxy状态向量（位置 速度 加速度）
    vector_t proxy_1  = bounded_admittance_controller_->getJoint1ProxyState();
    vector_t proxy_2  = bounded_admittance_controller_->getJoint2ProxyState();

    // base 仍采用师兄的先阻抗再导纳  然后只采用跟踪base的proxy
    scalar_t force_z = 0;                      
    for (size_t i = 0; i < 4; ++i) {
        force_z += inputDesired[3*i+2];
    } 
    //vector_t tau_cmd = BaseBoundedAdmittanceUpdate(rbdStateMeasured, time, period, imp, force_z);
    vector_t proxy_x = bounded_admittance_controller_base_x_->getProxyState();    // 多维暂时先对base采用先阻抗后导纳，然后只跟踪proxy
    Task task0 = formulateFloatingBaseEomWithEEForceTask() + formulateTorqueLimitsTaskWithEEForceTask()
                    + formulateNoContactMotionTask() + formulateFrictionConeTask();

    // arm compliance
    Task taskAllImpedance  = formulateManipulatorTorqueTask(ImpArm_tau.tail(6));

    Task taskJoint12Admittance  = ManipulatorTorqueTaskTwoAdmittance(ImpArm_tau.tail(6));
    Task taskJoint1Admittance  = ManipulatorTorqueTaskJointOneAdmittance(ImpArm_tau.tail(6));  //除了joint2都是跟踪阻抗   joint2 留给导纳
    Task taskJoint2Admittance  = ManipulatorTorqueTaskJointTwoAdmittance(ImpArm_tau.tail(6));  //除了joint3都是跟踪阻抗   joint3 留给导纳

    Task trackJoint1Proxy = formulateMulitJoint1ProxyTrackingTask(proxy_1[0],proxy_1[1],proxy_1[2]);
    Task trackJoint2Proxy = formulateMultiJoint2ProxyTrackingTask(proxy_2[0],proxy_2[1],proxy_2[2]);
    Task trackJoint12Proxy = formulateMultiJoint12ProxyTrackingTask(proxy_1[0],proxy_1[1],proxy_1[2],proxy_2[0],proxy_2[1],proxy_2[2]);


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

    Task taskBaseProxy = formulateBaseXAdmittanceTrackingTask(proxy_x[0], proxy_x[1], proxy_x[2]);  //proxy_x[0] [1] [2]分别代表proxy的位置 速度 加速度
    Task task1 = formulateBaseHeightMotionTask() + formulateBaseAngularMotionTask() + formulateSwingLegTask() * 100
                + formulateEeAngularMotionTrackingTask() + taskBA + taskBaseProxy + formulateBaseYLinearMotionTask();
    Task task3 = formulateContactForceTask(inputDesired);
    HoQp hoQp(task3, std::make_shared<HoQp>(task1, std::make_shared<HoQp>(task0)));
    vector_t x_optimal = hoQp.getSolutions();


    return WbcBase::updateCmdWithEEForce(x_optimal);
/*
    //提取2、3关节的proxy状态向量（位置 速度 加速度）
    vector_t proxy_1  = admittance_controller1_->getProxyState();
    vector_t proxy_2  = admittance_controller2_->getProxyState();

    
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

*/

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

    if(controller_mode_ == 1)   // 常规饱和多维导纳
    {
        //发布tau_cmd
        // std_msgs::Float64 f_msg;
        // f_msg.data = -tau_max_;
        // torque_pub_.publish(f_msg);  
        return MultiAdmittanceControl(stateDesired, inputDesired, rbdStateMeasured, mode, period, time);
    }

    if(controller_mode_ == 2)   // 多维集值饱和导纳控制， arm的两个关节采用多维离散化算法，对base采用先阻抗再导纳追踪proxy
    {
        //发布tau_cmd
        // std_msgs::Float64 f_msg;
        // f_msg.data = -tau_max_;
        // torque_pub_.publish(f_msg);  
        vector_t torque = MultiBoundedAdmittanceControl(stateDesired, inputDesired, rbdStateMeasured, mode, period, time);
        torque = torque.tail(18);
        std_msgs::Float64 torque_msg1,torque_msg2;
        torque_msg1.data = torque[13];
        torque_msg2.data = torque[14];
        Multi_BA_tau_cmd1_pub.publish(torque_msg1);
        Multi_BA_tau_cmd2_pub.publish(torque_msg2);
        return MultiBoundedAdmittanceControl(stateDesired, inputDesired, rbdStateMeasured, mode, period, time);
    }
    if(controller_mode_ == 3)   // 饱和导纳控制， arm的两个关节采用多维离散化算法只追踪proxy，对base采用先阻抗再导纳追踪proxy
    {
        //发布tau_cmd
        // std_msgs::Float64 f_msg;
        // f_msg.data = -tau_max_;
        // torque_pub_.publish(f_msg);  
        vector_t torque = MultiBoundedAdmittanceControl(stateDesired, inputDesired, rbdStateMeasured, mode, period, time);
        torque = torque.tail(18);
        std_msgs::Float64 torque_msg1,torque_msg2;
        torque_msg1.data = torque[13];
        torque_msg2.data = torque[14];
        Multi_BA_tau_cmd1_pub.publish(torque_msg1);
        Multi_BA_tau_cmd2_pub.publish(torque_msg2);
        return MultiBAProxyTrackingControl(stateDesired, inputDesired, rbdStateMeasured, mode, period, time);
    }
    throw std::runtime_error("Update function: No matching condition, returning error.");

}



void CompliantWbc::dynamicCallback(qm_wbc::CompliantConfig &config, uint32_t) {
    tau_max_ << config.joint1_tau_max, config.joint2_tau_max, config.joint3_tau_max,
            config.joint4_tau_max, config.joint5_tau_max, config.joint6_tau_max;

    // pure admittance for joint1 & joint2
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

    Base_Lambdax = config.Base_Lambda_x;
    Base_Lambday = config.Base_Lambda_y;
    Base_K1x = config.Base_K1_x;
    Base_K1y = config.Base_K1_y;
    // multi_dimensional bounded admittance for arm
    MultiBA_M1x_ = config.Multi_M1x;
    MultiBA_M2x_ = config.Multi_M2x;
    MultiBA_D1x_ = config.Multi_D1x;
    MultiBA_D2x_ = config.Multi_D2x;
    MultiBA_K1x_ = config.Multi_K1x;
    MultiBA_K2x_ = config.Multi_K2x;
    //标量形式滑模参数
    SM_lambda = config.SM_lambda;
    SM_K1 = config.SM_k_1;
    //矩阵形式滑模参数
    Lambda1 = config.SM_Lambda1;
    Lambda2 = config.SM_Lambda2;
    K11 = config.SM_K1_1;
    K12 = config.SM_K1_2;
    //矩阵形式PID参数
    PID_M1 = config.Multi_M_pid1;
    PID_M2 = config.Multi_M_pid2;
    PID_K1 = config.Multi_K_pid1;
    PID_K2 = config.Multi_K_pid2;
    PID_B1 = config.Multi_B_pid1;
    PID_B2 = config.Multi_B_pid2;
    PID_L1 = config.Multi_L_pid1;
    PID_L2 = config.Multi_L_pid2;


    MultiBA_M1_ = config.Multi_M1;
    MultiBA_M2_ = config.Multi_M2;
    MultiBA_C1_ = config.Multi_C1;
    MultiBA_C2_ = config.Multi_C2;
    
    MultiBA_Kp1_ = config.Multi_Kp1;
    MultiBA_Kp2_ = config.Multi_Kp2;
    MultiBA_Kd1_ = config.Multi_Kd1;
    MultiBA_Kd2_ = config.Multi_Kd2;

    begin_ = config.begin;
    end_   = config.end;

    mu_ = config.mu;

    controller_mode_ = config.admittance_controller_model;
    ROS_INFO_STREAM("\033[32m Update the CompliantWbc param. \033[0m");
}

}
