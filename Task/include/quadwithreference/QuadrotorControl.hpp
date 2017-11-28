//
// Created by jhwangbo on 30.11.16.
//

//#ifndef RAI_QUADROTORCONTROL_HPP
//#define RAI_QUADROTORCONTROL_HPP

// custom inclusion- Modify for your task
#include "rai/tasks/common/Task.hpp"
#include "raiCommon/enumeration.hpp"
#include "raiCommon/utils/RandomNumberGenerator.hpp"
#include "raiCommon/TypeDef.hpp"
#include "raiCommon/math/inverseUsingCholesky.hpp"
#include "raiCommon/math/RAI_math.hpp"
#include <rai/RAI_core>
#include "raiGraphics/RAI_graphics.hpp"
#include "quadwithreference/visualizer/QuadrotorTrack_Visualizer.hpp"
#include "raiCommon/utils/StopWatch.hpp"

#pragma once

namespace rai {
namespace Task {

constexpr int StateDim = 33;
constexpr int ActionDim = 4;
constexpr int CommandDim = 0;

template<typename Dtype>
class QuadrotorControl : public Task<Dtype,
                                     StateDim,
                                     ActionDim,
                                     CommandDim> {
 public:
  using TaskBase = Task<Dtype, StateDim, ActionDim, CommandDim>;
  using State = typename TaskBase::State;
  using StateBatch = typename TaskBase::StateBatch;
  using Action = typename TaskBase::Action;
  using ActionBatch = typename TaskBase::ActionBatch;
  using Command = typename TaskBase::Command;
  using MatrixXD = Eigen::Matrix<Dtype, Eigen::Dynamic, Eigen::Dynamic>;
  using VectorXD = Eigen::Matrix<Dtype, Eigen::Dynamic, 1>;
  using PhysicalState = VectorXD;
  using MatrixJacobian = typename TaskBase::JacobianStateResAct;
  using MatrixJacobianCostResAct = typename TaskBase::JacobianCostResAct;

  using GeneralizedCoordinate = Eigen::Matrix<double, 7, 1>;
  using GeneralizedVelocity = Eigen::Matrix<double, 6, 1>;
  using GeneralizedAcceleration = Eigen::Matrix<double, 6, 1>;

  QuadrotorControl() {

    //// set default parameters
    this->valueAtTermination_ = 1.5;
    this->discountFactor_ = 0.99;
    this->timeLimit_ = 15.0;
    this->controlUpdate_dt_ = 0.01;
    gravity_ << 0.0, 0.0, -9.81;

    Eigen::Vector3d diagonalInertia;
//    diagonalInertia << 0.0105, 0.0105, 0.018;
    diagonalInertia << 0.007, 0.007, 0.012;

    inertia_ = diagonalInertia.asDiagonal();
    cholInv(inertia_, inertiaInv_);
    comLocation_ << 0.0, 0.0, -0.05;

    /////// scale //////
    actionScale_ = 2.0;
    orientationScale_ = 1.0;
    positionScale_ = 0.5;
    angVelScale_ = 0.15;
    linVelScale_ = 0.5;
    position_t << 0.0, 0.0, 0.0;

    /////// adding constraints////////////////////
    State upperStateBound, lowerStateBound;
    upperStateBound << 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0,
                        3.0, 3.0, 3.0,
                        5.0, 5.0, 5.0,
                        6.0, 6.0, 6.0,
                        2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0,
                        5.0, 5.0, 5.0,
                        6.0, 6.0, 6.0;
    lowerStateBound = -upperStateBound;
    this->setBoxConstraints(lowerStateBound, upperStateBound);
    transsThrust2GenForce << 0, 0, length_, -length_,
        -length_, length_, 0, 0,
        dragCoeff_, dragCoeff_, -dragCoeff_, -dragCoeff_,
        1, 1, 1, 1;
    transsThrust2GenForceInv = transsThrust2GenForce.inverse();
    targetPosition.setZero();
  }

  ~QuadrotorControl() {
  }

  void step(const Action &action_t,
            State &state_tp1,
            TerminationType &termType,
            Dtype &costOUT) {

    //Get current state from q_, u_
    orientation = q_.head(4); //Orientation of quadrotor
    orientation_t = qt_.head(4);
    double angle = 2.0 * std::acos(q_(0)); //Angle of rotation

    position = q_.tail(3); //Position of quadrotor
    R_ = Math::MathFunc::quatToRotMat(orientation); //Calculate rotation matrix

    v_I_ = u_.tail(3); //linear velocity
    w_I_ = u_.head(3); //angular velocity
    w_B_ = R_.transpose() * w_I_; //body rates // TODO: u_.head(3) = w_I_

    //Control input from action

    Action actionGenForce;
    actionGenForce = transsThrust2GenForce * actionScale_ * action_t; //Force generated by action
    B_torque = actionGenForce.segment(0, 3); //Moment input
    B_force << 0.0, 0.0, actionGenForce(3); // Thrust vector in {B}

    //Control input from PD stabilization
    double kp_rot = -0.2, kd_rot = -0.06;
    Torque fbTorque_b;

    if (angle > 1e-6)
      fbTorque_b = kp_rot * angle * (R_.transpose() * orientation.tail(3))
          / std::sin(angle) + kd_rot * (R_.transpose() * u_.head(3));
    else
      fbTorque_b = kd_rot * (R_.transpose() * u_.head(3));
    fbTorque_b(2) = fbTorque_b(2) * 0.15; //Lower yaw gains

    B_torque += fbTorque_b; //Sum of torque inputs
    B_force(2) += mass_ * 9.81; //Sum of thrust inputs

    // clip inputs
    Action genForce;
    genForce << B_torque, B_force(2);
    Action thrust = transsThrust2GenForceInv * genForce;    // TODO: This is not exactly thrust: change name
    thrust = thrust.array().cwiseMax(1e-8); // clip for min value
    genForce = transsThrust2GenForce * thrust;
    B_torque = genForce.segment(0, 3);
    B_force(2) = genForce(3);

    du_.tail(3) = (R_ * B_force) / mass_ + gravity_; // acceleration
//  w_B_ = R_.transpose() * u_.head(3); //body rates // TODO: u_.head(3) = w_I_

    du_.head(3) = R_ * (inertiaInv_ * (B_torque - w_B_.cross(inertia_ * w_B_))); //acceleration by inputs

    //Integrate states
    u_ += du_ * this->controlUpdate_dt_; //velocity after timestep
    w_I_ = u_.head(3);
    w_IXdt_ = w_I_ * this->controlUpdate_dt_;
    orientation = Math::MathFunc::boxplusI_Frame(orientation, w_IXdt_);
    Math::MathFunc::normalizeQuat(orientation);
    q_.head(4) = orientation;
    q_.tail(3) = q_.tail(3) + u_.tail(3) * this->controlUpdate_dt_;
    w_B_ = R_.transpose() * u_.head(3);

    // visualizer_.drawWorld(visualizeFrame, position, orientation);

    if (std::isnan(orientation.norm())) {
      std::cout << "u_ " << u_.transpose() << std::endl;
      std::cout << "du_ " << du_.transpose() << std::endl;
      std::cout << "B_torque " << B_torque.transpose() << std::endl;
      std::cout << "orientation " << orientation.transpose() << std::endl;
      std::cout << "state_tp1 " << q_.transpose() << std::endl;
      std::cout << "action_t " << action_t.transpose() << std::endl;
    }

    u_(0) = clip(u_(0), -20.0, 20.0);
    u_(1) = clip(u_(1), -20.0, 20.0);
    u_(2) = clip(u_(2), -20.0, 20.0);
    u_(3) = clip(u_(3), -5.0, 5.0);
    u_(4) = clip(u_(4), -5.0, 5.0);
    u_(5) = clip(u_(5), -5.0, 5.0);

    getState(state_tp1);

    costOUT = 0.004 * std::sqrt((q_ - qt_).norm()) +
        0.00005 * action_t.norm() +
        0.00005 * (u_.head(3) - ut_.head(3)).norm() +
        0.00005 * (u_.tail(3) - ut_.tail(3)).norm();

    // visualization
    if (this->visualization_ON_) {

      updateVisualizationFrames();
      visualizer_.drawWorld(visualizeFrame, position, orientation, position_t, orientation_t);
      double waitTime = std::max(0.0, this->controlUpdate_dt_ / realTimeRatio - watch.measure("sim", true));
      watch.start("sim");
      usleep(waitTime * 1e6);

    }
  }

  void changeTarget(Position& position){
    targetPosition = position;
  }

  bool isTerminalState(State &state) { return false; }

  void init() {
    /// initial state is random
    double oriF[4], posiF[3], angVelF[3], linVelF[3];
    double toriF[4], tangVelF[3], tlinVelF[3];
    rn_.template sampleOnUnitSphere<4>(oriF);
    rn_.template sampleVectorInNormalUniform<3>(posiF);
    rn_.template sampleVectorInNormalUniform<3>(angVelF);
    rn_.template sampleVectorInNormalUniform<3>(linVelF);
    rn_.template sampleOnUnitSphere<4>(toriF);
    rn_.template sampleVectorInNormalUniform<3>(tangVelF);
    rn_.template sampleVectorInNormalUniform<3>(tlinVelF);

    Quaternion orientation, torientation;
    Position position, tposition;
    AngularVelocity angularVelocity, tangularVelocity;
    LinearVelocity linearVelocity, tlinearVelocity;

    orientation << double(std::abs(oriF[0])), double(oriF[1]), double(oriF[2]), double(oriF[3]);
    rai::Math::MathFunc::normalizeQuat(orientation);
    position << double(posiF[0])*2., double(posiF[1])*2., double(posiF[2])*2.;
    angularVelocity << double(angVelF[0]), double(angVelF[1]), double(angVelF[2]);
    linearVelocity << double(linVelF[0]), double(linVelF[1]), double(linVelF[2]);

    torientation << double(std::abs(toriF[0])), double(toriF[1]), double(toriF[2]), double(toriF[3]);
    rai::Math::MathFunc::normalizeQuat(torientation);
    tposition << 0.0, 0.0, 0.0;
    tangularVelocity << double(tangVelF[0]), double(tangVelF[1]), double(tangVelF[2]);
    tlinearVelocity << double(tlinVelF[0]), double(tlinVelF[1]), double(tlinVelF[2]);

    q_ << orientation, position;
    qt_ << torientation, tposition;
    u_ << angularVelocity, linearVelocity;
    ut_ << tangularVelocity, tlinearVelocity;
  }

  void translate(Position& position) {
    q_.segment(4,3) += position;
  }

  void getInitialState(State &state) {
    init();
    getState(state);
  }

  void setInitialState(const State &in) {
    LOG(FATAL) << "The initial state is random. No need to set it" << std::endl;
  }

  void initTo(const State &state) {
    State stateT = state;
    R_.col(0) = stateT.segment(0, 3);
    R_.col(1) = stateT.segment(3, 3);
    R_.col(2) = stateT.segment(6, 3);
    orientation = Math::MathFunc::rotMatToQuat(R_);
    q_.head(4) = orientation;
    q_.tail(3) = state.segment(9, 3) / positionScale_;
    u_.head(3) = state.segment(12, 3) / angVelScale_;
    u_.tail(3) = state.segment(15, 3) / linVelScale_;
  }

  void getState(State &state) {
    LOG_IF(FATAL, std::isnan(q_.head(4).norm())) << "simulation unstable";
    orientation = q_.head(4);
    Math::MathFunc::normalizeQuat(orientation);
    R_ = Math::MathFunc::quatToRotMat(orientation);

    orientation_t = qt_.head(4);
    Math::MathFunc::normalizeQuat(orientation_t);
    Rt_ = Math::MathFunc::quatToRotMat(orientation_t);

    state << R_.col(0), R_.col(1), R_.col(2),
        q_.tail(3) * positionScale_,
        u_.head(3) * angVelScale_,
        u_.tail(3) * linVelScale_,
        Rt_.col(0), Rt_.col(1), Rt_.col(2),
        ut_.head(3) * angVelScale_,
        ut_.tail(3) * linVelScale_;
  };

  // Misc implementations
  void getGradientStateResAct(const State &stateIN,
                              const Action &actionIN,
                              MatrixJacobian &gradientOUT) {
    LOG(FATAL) << "To do!" << std::endl;
  };

  void getGradientCostResAct(const State &stateIN,
                             const Action &actionIN,
                             MatrixJacobianCostResAct &gradientOUT) {
    LOG(FATAL) << "To do!" << std::endl;
  }

  void getOrientation(Quaternion &quat){
    quat = orientation;
  }

  void getPosition(Position &posi){
    posi = q_.tail(3);
  }

  void getLinvel(LinearVelocity &linvel){
    linvel = u_.tail(3);
  }

  void getAngvel(AngularVelocity &angvel){
    angvel = u_.head(3);
  }

  void startRecordingVideo(std::string dir, std::string fileName) {
    mkdir(dir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    visualizer_.getGraphics()->savingSnapshots(dir, fileName);
  }

  void endRecordingVideo() {
    visualizer_.getGraphics()->images2Video();
  }

 private:


  void updateVisualizationFrames() {

    visualizeFrame.row(3).setZero();
    visualizeFrame(3, 3) = 1.0;

  }

  double clip(double input, double lower, double upper) {
    input = (input > lower) * input + !(input > lower) * lower;
    return (input < upper) * input + !(input < upper) * upper;
  }

  template<typename T>
  inline double sgn(T val) {
    return double((T(0) < val) - (val < T(0)));
  }

  GeneralizedCoordinate q_, qt_; // generalized state and velocity
  GeneralizedVelocity u_, ut_;
  GeneralizedAcceleration du_;

  RotationMatrix R_, Rt_;
  LinearAcceleration gravity_;

  double orientationScale_, positionScale_, angVelScale_, linVelScale_;
  double actionScale_;
  /// robot parameters
  double length_ = 0.17;
  Position comLocation_;
  double dragCoeff_ = 0.016;
  double mass_ = 0.665;
  Inertia inertia_;
  Inertia inertiaInv_;
  Quaternion orientation, orientation_t;
  Position position, position_t;
  Force B_force;
  Torque B_torque;
  AngularVelocity w_I_, w_B_;
  LinearVelocity v_I_;
  RandomNumberGenerator<Dtype> rn_;
  EulerVector w_IXdt_;
  static rai_graphics::RAI_graphics graphics;
//  static rai_graphics::object::Quadrotor quadrotor;
  static rai_graphics::object::Sphere target;
  static Position targetPosition;
  double visualizationTime = 0;

  Eigen::Matrix4d transsThrust2GenForce;
  Eigen::Matrix4d transsThrust2GenForceInv;


  //Visualization
  StopWatch watch;
  double realTimeRatio = 1;
  static rai::Vis::QuadrotorTrack_Visualizer visualizer_;

  HomogeneousTransform visualizeFrame;

};
}
} /// namespaces
template<typename Dtype>
rai::Position rai::Task::QuadrotorControl<Dtype>::targetPosition;
template<typename Dtype>
rai::Vis::QuadrotorTrack_Visualizer rai::Task::QuadrotorControl<Dtype>::visualizer_;
//#endif //RAI_QUADROTORCONTROL_HPP