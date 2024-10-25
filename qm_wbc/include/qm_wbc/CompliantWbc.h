//
// Created by skywoodsz on 2023/5/6.
//

#ifndef SRC_COMPLIANTWBC_H
#define SRC_COMPLIANTWBC_H

#include "qm_wbc/WbcBase.h"

#include <qm_compliant/BoundedAdmittance.h>
#include <qm_compliant/BoundedAdmittanceWithK.h>
#include <qm_compliant/CartesianImpendance.h>
#include <qm_compliant/BoundedAdmittanceNoImp.h>
#include <qm_compliant/BaseBoundedAdmittanceNoImp.h>
#include <qm_compliant/BoundedAdmittanceMultiDim.h>
#include <qm_compliant/AdmittanceMultiDim.h>
#include <qm_compliant/BaseBAMultiDim.h>
#include <qm_compliant/BaseAdmCMultiDim.h>
#include <ocs2_oc/synchronized_module/ReferenceManagerInterface.h>

#include <dynamic_reconfigure/server.h>
#include <qm_wbc/CompliantConfig.h>
#include <std_msgs/Float64.h>
namespace qm{
class CompliantWbc : public WbcBase{
    using vector6_t = Eigen::Matrix<scalar_t, 6, 1>;
public:
    CompliantWbc(const PinocchioInterface &pinocchioInterface, CentroidalModelInfo info,
                 const PinocchioEndEffectorKinematics &eeKinematics,
                 const ocs2::PinocchioEndEffectorKinematics &armEeKinematics,
                 ros::NodeHandle &controller_nh);

    vector_t update(const vector_t &stateDesired, const vector_t &inputDesired, const vector_t &rbdStateMeasured,
                    size_t mode, scalar_t period, scalar_t time) override;
    vector_t getImpDesiredTorque(){ return imp_desired_; }
private:
    void dynamicCallback(qm_wbc::CompliantConfig& config, uint32_t /*level*/);
    void AdmittanceInit(const PinocchioInterface& pinocchioInterface, CentroidalModelInfo info,
                               const PinocchioEndEffectorKinematics& armEeKinematics, ros::NodeHandle &controller_nh);
    void MultiBoundedAdmittanceInit(const PinocchioInterface& pinocchioInterface, CentroidalModelInfo info,
                            const PinocchioEndEffectorKinematics& armEeKinematics, ros::NodeHandle &controller_nh);  // 机械臂多维初始化（集值算法）
    void MultiAdmittanceInit(const PinocchioInterface& pinocchioInterface, CentroidalModelInfo info,
                            const PinocchioEndEffectorKinematics& armEeKinematics, ros::NodeHandle &controller_nh);  // 机械臂多维初始化（AdmC）
    void BaseBoundedAdmittanceInit(const PinocchioInterface& pinocchioInterface, CentroidalModelInfo info,
                                   const PinocchioEndEffectorKinematics& armEeKinematics, ros::NodeHandle &controller_nh);  //base x 方向单维
    void BaseAdmittanceInit(const PinocchioInterface& pinocchioInterface, CentroidalModelInfo info,
                                   const PinocchioEndEffectorKinematics& armEeKinematics, ros::NodeHandle &controller_nh);   //base x 方向单维      
    void BaseBAMultiDimiInit(const PinocchioInterface& pinocchioInterface, CentroidalModelInfo info,
                                   const PinocchioEndEffectorKinematics& armEeKinematics, ros::NodeHandle &controller_nh);   //base 多维离散化(集合值算法)
    void BaseAdmCMultiDimiInit(const PinocchioInterface& pinocchioInterface, CentroidalModelInfo info,
                                   const PinocchioEndEffectorKinematics& armEeKinematics, ros::NodeHandle &controller_nh);   //base 多维离散化（AdmC）                                               
    void AdmittanceUpdate(const vector_t& rbdStateMeasured, scalar_t time, scalar_t period);
    void BaseAdmittanceUpdate(const vector_t& rbdStateMeasured, scalar_t time, scalar_t period);
    vector6_t BaseBoundedAdmittanceUpdate(const vector_t& rbdStateMeasured, scalar_t time, scalar_t period,
                                          vector_t imp, scalar_t force_z);      //base x 方向单维度update
    vector6_t MultiBoundedAdmittanceUpdate(const vector_t& rbdStateMeasured, scalar_t time, scalar_t period,vector_t imp);   // 机械臂多维update（集值算法）
    vector6_t MultiAdmittanceUpdate(const vector_t& rbdStateMeasured, scalar_t time, scalar_t period,vector_t imp);   // 机械臂多维update（AdmC）
    vector6_t BaseBAMultiDimUpdate(const vector_t& rbdStateMeasured, scalar_t time, scalar_t period, vector_t imp, scalar_t force_z);  //base多维update(集值算法)
    vector6_t BaseAdmCMultiDimUpdate(const vector_t& rbdStateMeasured, scalar_t time, scalar_t period, vector_t imp, scalar_t force_z);  //base多维update（AdmCs）
    vector_t MultiAdmittanceControl(const vector_t &stateDesired, const vector_t &inputDesired, const vector_t &rbdStateMeasured,
                               size_t mode, scalar_t period, scalar_t time);
    vector_t MultiBoundedAdmittanceControl(const vector_t &stateDesired, const vector_t &inputDesired, const vector_t &rbdStateMeasured,
                               size_t mode, scalar_t period, scalar_t time);   // 多维离散化  机械臂只跟踪多维计算出来的力矩
    vector_t MultiBAProxyTrackingControl(const vector_t &stateDesired, const vector_t &inputDesired, const vector_t &rbdStateMeasured,
                               size_t mode, scalar_t period, scalar_t time);   // 多维离散化  机械臂只跟踪多维计算出来的力矩
    // impendance controller
    std::shared_ptr<CartesianImpendance> impendace_controller_;
    //  pure admittance controller
    std::shared_ptr<BoundedAdmittanceNoImp> admittance_controller2_;
    std::shared_ptr<BoundedAdmittanceNoImp> admittance_controller1_;
    // std::shared_ptr<BoundedAdmittance> bounded_admittance_controller2_;
    // std::shared_ptr<BoundedAdmittance> admittance_controller1_;
    std::shared_ptr<BoundedAdmittanceWithK> bounded_admittance_controller_base_x_;
    std::shared_ptr<BaseBoundedAdmittanceNoImp> admittance_controller_base_x_;
    //  Multi_bounded admittance controller for manipulator
    std::shared_ptr<BoundedAdmittanceMultiDim> bounded_admittance_controller_;   // 多维微分包含导纳离散化(机械臂两个关节)
    //  Multi_bounded admittance controller for base
    std::shared_ptr<BaseBAMultiDim> Base_bounded_admittance_controller_;   // 多维微分包含导纳离散化(base 的xy方向)

    // AdmC for manipulator
    std::shared_ptr<AdmittanceMultiDim> Multi_admittance_controller_;   // 多维常规饱和导纳离散化(机械臂两个关节)
    // AdmC for base
    std::shared_ptr<BaseAdmCMultiDim> Base_AdmC_controller_;   // 多维常规饱和导纳离散化(base 的xy方向)

    std::shared_ptr<dynamic_reconfigure::Server<qm_wbc::CompliantConfig>> dynamic_srv_{};

    vector6_t tau_max_;
    vector_t imp_desired_;
    
    // multi bounded admittance param for base
    scalar_t Mbase_px_, Bbase_px_, Kbase_px_; // base proxy
    scalar_t Mbase_py_, Bbase_py_, Kbase_py_; // base proxy
    // base PID  (uselesss)
    scalar_t Bbasex_, Lbasex_, Kbasex_, Mbasex_; // base controller
    scalar_t Bbasey_, Lbasey_, Kbasey_, Mbasey_; // base controller

    // base MultiBA Controller
    //Base矩阵形式滑模变量Lambda
    scalar_t Base_Lambdax, Base_Lambday;
    //Base矩阵形式滑模变量K1
    scalar_t Base_K1x, Base_K1y;

    // 师兄方案用到的单维导纳，proposed算法用不到
    scalar_t M1x_, D1x_, K1x_; // joint2 proxy
    scalar_t M2x_, D2x_, K2x_; // joint3 proxy
    scalar_t D1_, L1_, K1_, M1_; // joint2 controller
    scalar_t D2_, L2_, K2_, M2_; // joint3 controller

    // multi bounded admittance param for manipulator
    scalar_t MultiBA_M1x_, MultiBA_D1x_, MultiBA_K1x_; // joint2 proxy
    scalar_t MultiBA_M2x_, MultiBA_D2x_, MultiBA_K2x_; // joint3 proxy

    scalar_t SM_lambda;  // 标量形式滑模变量 lambda
    scalar_t SM_K1;  // 标量形式滑模变量k1
    //矩阵形式滑模变量Lambda
    scalar_t Lambda1, Lambda2;
    //矩阵形式滑模变量K1
    scalar_t K11, K12;
    //矩阵形式PID参数(作对比用)
    scalar_t PID_M1, PID_M2;
    scalar_t PID_K1, PID_K2;
    scalar_t PID_B1, PID_B2;
    scalar_t PID_L1, PID_L2;


    scalar_t MultiBA_M1_, MultiBA_M2_;
    scalar_t MultiBA_C1_, MultiBA_C2_;
    scalar_t MultiBA_Kp1_, MultiBA_Kp2_;
    scalar_t MultiBA_Kd1_, MultiBA_Kd2_;
    //纯导纳发布tau_cmd
    ros::Publisher Admittance_tau_cmd1_pub,Admittance_tau_cmd2_pub;
    //多维bounded admittance 发布tau_cmd  (这个tau_cmd是最终WBC下发的机械臂力矩)
    ros::Publisher Multi_BA_tau_cmd1_pub, Multi_BA_tau_cmd2_pub;
    //多维bounded admittance 发布tau_cmd  (这个tau_cmd是最终WBC下发的机械臂力矩)
    ros::Publisher Multi_cmd1_pub, Multi_cmd2_pub;
    ros::Publisher MultiAD_cmd1_pub, MultiAD_cmd2_pub;
    ros::Publisher Multi_M_1_pub, Multi_M_2_pub, Multi_N_1_pub, Multi_N_2_pub;
    ros::Publisher Gravity1_pub,Gravity2_pub;
    // bounded admittance param
    ros::Publisher BaseXforce_pub, BaseYforce_pub, BaseZforce_pub;
    scalar_t mu_{};
    size_t begin_{}, end_{};  //切换导纳控制器使能关节
    //柔顺控制模式切换标志位
    size_t controller_mode_;  // 1:pure admittance 2:torque_bounded admittance(老师离散化算法)
};
}

#endif //SRC_COMPLIANTWBC_H
