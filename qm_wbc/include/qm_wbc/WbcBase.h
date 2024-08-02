//
// Created by skywoodsz on 2023/4/9.
//

#ifndef SRC_WBCBASE_H
#define SRC_WBCBASE_H

#include "qm_wbc/Task.h"

#include <ocs2_centroidal_model/PinocchioCentroidalDynamics.h>
#include <ocs2_legged_robot/gait/MotionPhaseDefinition.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_oc/synchronized_module/ReferenceManagerInterface.h>

#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include "qm_wbc/WbcWeightConfig.h"
#include <std_msgs/Float64.h>

namespace qm{
using namespace ocs2;
using namespace legged_robot;
using vector6_t = Eigen::Matrix<scalar_t, 6, 1>;
using vector5_t = Eigen::Matrix<scalar_t, 5, 1>;
using vector4_t = Eigen::Matrix<scalar_t, 4, 1>;
using vector2_t = Eigen::Matrix<scalar_t, 2, 1>;
// Decision Variables: x = [\dot v^T, F^T]^T
class WbcBase{
    using vector6_t = Eigen::Matrix<scalar_t, 6, 1>;
    using Matrix6 = Eigen::Matrix<scalar_t, 6, 6>;
public:
    WbcBase(const PinocchioInterface& pinocchioInterface, CentroidalModelInfo info,
            const PinocchioEndEffectorKinematics& eeKinematics, const PinocchioEndEffectorKinematics& armEeKinematics,
            ros::NodeHandle &controller_nh);
    virtual vector_t update(const vector_t& stateDesired, const vector_t& inputDesired,
                            const vector_t& rbdStateMeasured, size_t mode, scalar_t period, scalar_t time);

    virtual void loadTasksSetting(const std::string& taskFile, bool verbose);

    void setReferenceManager(std::shared_ptr<ReferenceManagerInterface> referenceManagerPtr);
    virtual vector_t getImpDesiredTorque() = 0;

protected:
    void updateMeasured(const vector_t& rbdStateMeasured);
    void updateDesired(const vector_t& stateDesired, const vector_t& inputDesired, ocs2::scalar_t period);
    vector_t updateCmd(vector_t x_optimal);
    void setManipulatorTorqueLimit(const vector_t torqueLimit);

    vector6_t getExternalArmTorque();
    vector6_t getExternalBaseTorque();

    matrix2_t getInertiaTerm();
    matrix2_t getCoriolisTerm();
    vector6_t getNonlinearTerm();
    vector6_t getGravityTerm();

    vector3_t getEEPosition();
    vector3_t getEEVelocity();
    size_t getNumDecisionVars() const { return numDecisionVars_; }

    Task formulateFloatingBaseEomTask();
    Task formulateTorqueLimitsTask();
    Task formulateNoContactMotionTask();
    Task formulateFrictionConeTask();

    Task formulateBaseHeightMotionTask();
    Task formulateBaseAngularMotionTask();
    Task formulateBaseXYLinearAccelTask();
    Task formulateBaseXYLinearMotionTask();
    Task formulateBaseYLinearMotionTask();
    Task formulateSwingLegTask();

    Task formulateArmJointNomalTrackingTask();
    Task formulateEeLinearMotionTrackingTask();
    Task formulateEeAngularMotionTrackingTask();

    Task formulateContactForceTask(const vector_t& inputDesired) const;

    // with ee force
    Task formulateFloatingBaseEomWithEEForceTask();
    Task formulateTorqueLimitsTaskWithEEForceTask();
    vector_t updateCmdWithEEForce(vector_t x_optimal);
    Task formulateManipulatorTorqueTask(const vector_t& inputDesired);
    Task ManipulatorTorqueTaskTwoAdmittance(const vector_t& inputDesired);
    Task ManipulatorTorqueTaskJointOneAdmittance(const vector_t& inputDesired);
    Task ManipulatorTorqueTaskJointTwoAdmittance(const vector_t& inputDesired);

    Task formulateBaseXMotionTrackingTask(const scalar_t qx, const scalar_t dot_qx, const scalar_t ddot_qx);
    Task formulateBaseXAdmittanceTrackingTask(const scalar_t qx, const scalar_t dot_qx, const scalar_t ddot_qx);
    Task formulateJoint1ProxyTrackingTask(const scalar_t qx, const scalar_t dot_qx, const scalar_t ddot_qx);
    Task formulateJoint2ProxyTrackingTask(const scalar_t qx, const scalar_t dot_qx, const scalar_t ddot_qx);
    Task formulateJoint12ProxyTrackingTask(const scalar_t proxy1,const scalar_t proxy1_dot,const scalar_t proxy1_ddot,const scalar_t proxy2 ,const scalar_t proxy2_dot,const scalar_t proxy2_ddot);
    Task formulateJoint1SetPostitionTask(const scalar_t q0, const scalar_t dot_q0, const scalar_t ddot_q0);
    Task formulateJoint2SetPostitionTask(const scalar_t q0, const scalar_t dot_q0, const scalar_t ddot_q0);

    Task formulateJoint0SetPostitionTask(const scalar_t q0, const scalar_t dot_q0, const scalar_t ddot_q0);
    Task formulateJoint3SetPostitionTask(const scalar_t q0, const scalar_t dot_q0, const scalar_t ddot_q0);
    Task formulateJoint4SetPostitionTask(const scalar_t q0, const scalar_t dot_q0, const scalar_t ddot_q0);
    Task formulateJoint5SetPostitionTask(const scalar_t q0, const scalar_t dot_q0, const scalar_t ddot_q0);
    // 多维饱和proxy跟踪任务
    Task formulateMulitJoint1ProxyTrackingTask(const scalar_t qx, const scalar_t dot_qx, const scalar_t ddot_qx);
    Task formulateMultiJoint2ProxyTrackingTask(const scalar_t qx, const scalar_t dot_qx, const scalar_t ddot_qx);
    Task formulateMultiJoint12ProxyTrackingTask(const scalar_t proxy1,const scalar_t proxy1_dot,const scalar_t proxy1_ddot,const scalar_t proxy2 ,const scalar_t proxy2_dot,const scalar_t proxy2_ddot);


    Task formulateBaseYMotionTrackingTask(const scalar_t qx, const scalar_t dot_qx, const scalar_t ddot_qx);
    Task formulateXYContactForceTaskWithCompliant(const vector_t &inputDesired, const vector_t &impDesired);
    Task formulateZContactForceTask(const vector_t &inputDesired);
    Task formulateYContactForceTask(const vector_t &inputDesired);
    Task formulateXContactForceTaskWithCompliant(const vector_t &inputDesired, const vector_t &impDesired);

    size_t generalizedCoordinatesNum_{};
    scalar_t frictionCoeff_{};
    vector_t qDesired_, vDesired_, baseAccDesired_;
    vector_t eeForce_;

    ros::Publisher Joint1MPCdesiredPublisher_, Joint2MPCdesiredPublisher_;

    std::shared_ptr<ReferenceManagerInterface> referenceManagerPtr_;

private:
    void dynamicCallback(qm_wbc::WbcWeightConfig& config, uint32_t /*level*/);

    size_t numDecisionVars_;
    PinocchioInterface pinocchioInterfaceMeasured_, pinocchioInterfaceDesired_;
    CentroidalModelInfo info_;
    CentroidalModelPinocchioMapping mapping_;
    std::unique_ptr<PinocchioEndEffectorKinematics> eeKinematics_, armEeKinematics_;

    contact_flag_t contactFlag_{};
    size_t numContacts_{};
    vector_t qMeasured_, vMeasured_, inputLast_;
    vector_t jointAccel_;
    matrix_t j_, dj_;
    matrix_t arm_j_, arm_dj_;
    matrix_t base_j_, base_dj_;

    // Task Parameters:
    std::shared_ptr<dynamic_reconfigure::Server<qm_wbc::WbcWeightConfig>> dynamic_srv_{};
    vector_t legTorqueLimits_, armTorqueLimits_;
    scalar_t swingKp_{}, swingKd_{};
    scalar_t baseHeightKp_{}, baseHeightKd_{};
    scalar_t baseAngularKp_{}, baseAngularKd_{};
    scalar_t baseLinearKp_{}, baseLinearKd_{};

    //纯导纳proxy追踪
    scalar_t joint1ProxyTrackingKp_{},joint2ProxyTrackingKp_{};
    scalar_t joint1ProxyTrackingKd_{},joint2ProxyTrackingKd_{};
    scalar_t baseXAdmittanceKp_{}, baseXAdmittanceKd_{};
    scalar_t baseYAdmittanceKp_{}, baseYAdmittanceKd_{};

    //多维饱和导纳proxy 追踪
    scalar_t Multi_joint1ProxyTrackingKp_{},Multi_joint2ProxyTrackingKp_{};
    scalar_t Multi_joint1ProxyTrackingKd_{},Multi_joint2ProxyTrackingKd_{};

    matrix_t jointKp_, jointKd_;
    matrix_t armEeLinearKp_{}, armEeLinearKd_{};
    matrix_t armEeAngularKp_{}, armEeAngularKd_{};
    size_t armEeFrameIdx_{};
};

}


#endif //SRC_WBCBASE_H
