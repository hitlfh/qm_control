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

    void BoundedAdmittanceInit(const PinocchioInterface& pinocchioInterface, CentroidalModelInfo info,
                               const PinocchioEndEffectorKinematics& armEeKinematics, ros::NodeHandle &controller_nh);
    void BaseBoundedAdmittanceInit(const PinocchioInterface& pinocchioInterface, CentroidalModelInfo info,
                                   const PinocchioEndEffectorKinematics& armEeKinematics, ros::NodeHandle &controller_nh);
    void BoundedAdmittanceUpdate(const vector_t& rbdStateMeasured, scalar_t time, scalar_t period);
    vector6_t BaseBoundedAdmittanceUpdate(const vector_t& rbdStateMeasured, scalar_t time, scalar_t period,
                                          vector_t imp, scalar_t force_z);
    vector_t BoundanceAdmittanceControl(const vector_t &stateDesired, const vector_t &inputDesired, const vector_t &rbdStateMeasured,
                               size_t mode, scalar_t period, scalar_t time);

    // impendance controller
    std::shared_ptr<CartesianImpendance> impendace_controller_;
    // bounded admittance
    std::shared_ptr<BoundedAdmittanceNoImp> bounded_admittance_controller2_;
    std::shared_ptr<BoundedAdmittanceNoImp> bounded_admittance_controller1_;
    // std::shared_ptr<BoundedAdmittance> bounded_admittance_controller2_;
    // std::shared_ptr<BoundedAdmittance> bounded_admittance_controller1_;
    std::shared_ptr<BoundedAdmittanceWithK> bounded_admittance_controller_base_x_;

    // param
    std::shared_ptr<dynamic_reconfigure::Server<qm_wbc::CompliantConfig>> dynamic_srv_{};

    vector6_t tau_max_;
    vector_t imp_desired_;

    scalar_t Mbase_px_, Bbase_px_, Kbase_px_; // base proxy
    scalar_t Mbase_py_, Bbase_py_, Kbase_py_; // base proxy
    scalar_t Bbasex_, Lbasex_, Kbasex_, Mbasex_; // base controller
    scalar_t Bbasey_, Lbasey_, Kbasey_, Mbasey_; // base controller

    scalar_t M1x_, D1x_, K1x_; // joint2 proxy
    scalar_t M2x_, D2x_, K2x_; // joint3 proxy
    scalar_t D1_, L1_, K1_, M1_; // joint2 controller
    scalar_t D2_, L2_, K2_, M2_; // joint3 controller

    size_t begin_{}, end_{};  //切换导纳控制器使能关节
    // scalar_t Mbx_, Bbx_; // proxy
    // scalar_t Bb2_, Lb2_, Kb2_, Mb2_; // controller
    // scalar_t Bb1_, Lb1_, Kb1_, Mb1_; // controller

    //发布tau_cmd
    ros::Publisher tau_cmd1_pub,tau_cmd2_pub;
    scalar_t mu_{};
};
}

#endif //SRC_COMPLIANTWBC_H
