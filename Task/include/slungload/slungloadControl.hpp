//
// Created by Jaeyoung Lim on 25.11.17.
//

//#ifndef RAI_SLUNGLOADCONTROL_HPP
//#define RAI_SLUNGLOADCONTROL_HPP

// custom inclusion- Modify for your task
#include "rai/tasks/common/Task.hpp"
#include "raiCommon/enumeration.hpp"
#include "raiCommon/utils/RandomNumberGenerator.hpp"
#include "raiCommon/TypeDef.hpp"
#include "raiCommon/math/inverseUsingCholesky.hpp"
#include "raiCommon/math/RAI_math.hpp"
#include <rai/RAI_core>
#include "raiGraphics/RAI_graphics.hpp"
#include "slungload/visualizer/slungload_Visualizer.hpp"
#include "raiCommon/utils/StopWatch.hpp"

#pragma once

namespace rai {
namespace Task {

constexpr int StateDim = 24;
constexpr int ActionDim = 4;
constexpr int CommandDim = 0;

template<typename Dtype>
class slungloadControl : public Task<Dtype,
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

  using GeneralizedCoordinate = Eigen::Matrix<double, 10, 1>;
  using GeneralizedVelocity = Eigen::Matrix<double, 9, 1>;
  using GeneralizedAcceleration = Eigen::Matrix<double, 9, 1>;

  slungloadControl() {

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

    /////// adding constraints////////////////////
    State upperStateBound, lowerStateBound;
    upperStateBound << 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, //Rotation Matrix
                        3.0, 3.0, 3.0, //Quad Position
                        M_PI*3./8., M_PI*3./8., tether_length, //Load State Position
                        5.0, 5.0, 5.0, //Quad Angular Velocity
                        6.0, 6.0, 6.0, //Quad Linear Velocity
                        6.0, 6.0, 6.0; //Load State Velocity
    lowerStateBound = -upperStateBound;
    lowerStateBound(15) = 0.3*tether_length;
    this->setBoxConstraints(lowerStateBound, upperStateBound);
    transsThrust2GenForce << 0, 0, length_, -length_,
        -length_, length_, 0, 0,
        dragCoeff_, dragCoeff_, -dragCoeff_, -dragCoeff_,
        1, 1, 1, 1;
    transsThrust2GenForceInv = transsThrust2GenForce.inverse();
    targetPosition.setZero();
  }

  ~slungloadControl() {
  }

  void step(const Action &action_t,
            State &state_tp1,
            TerminationType &termType,
            Dtype &costOUT) {


    //Get current state from q_, u_
    orientation = q_.head(4); //Orientation of quadrotor
    double angle = 2.0 * std::acos(q_(0)); //Angle of rotation

    position = q_.segment<3>(4); //Position of quadrotor
    load_position = q_.tail(3);
    R_ = Math::MathFunc::quatToRotMat(orientation); //Calculate rotation matrix

    v_I_ = u_.segment<3>(3); //linear velocity
    w_I_ = u_.head(3); //angular velocity
    vl_I_ = u_.tail(3); //Load velocity
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

    load_direction = load_position - position;
    Position direction = (1 /load_direction.norm())* load_direction;
    T_force = load_mass_ * (gravity_ + du_.tail(3)).norm() * direction;

    if(load_direction.norm() < tether_length){
      T_force = 0.0*T_force;

      du_.segment<3>(3) = (R_ * B_force) / mass_ + gravity_; // quadrotor acceleration
//      du_.segment<3>(3) = 0.0*du_.segment<3>(3); // quadrotor acceleration
      du_.head(3) = R_ * (inertiaInv_ * (B_torque - w_B_.cross(inertia_ * w_B_))); //acceleration by inputs
      du_.tail(3) = gravity_; // Load Acceleration

      //Integrate states
      u_ += du_ * this->controlUpdate_dt_; //velocity after timestep

      w_I_ = u_.head(3);
      w_IXdt_ = w_I_ * this->controlUpdate_dt_;
      orientation = Math::MathFunc::boxplusI_Frame(orientation, w_IXdt_);
      Math::MathFunc::normalizeQuat(orientation);

      q_.head(4) = orientation;
      q_.segment<3>(4) = q_.segment<3>(4) + u_.segment<3>(3) * this->controlUpdate_dt_; //Quad Position
      q_.tail(3) = q_.tail(3) + u_.tail(3) * this->controlUpdate_dt_; // Load Posittion

      if ((q_.tail(3) - q_.segment<3>(4)).norm() > tether_length)
        q_.tail(3) = q_.segment<3>(4) + tether_length * (q_.tail(3) - q_.segment<3>(4))/(q_.tail(3) - q_.segment<3>(4)).norm(); // Position Constraint

    } else {
      du_.segment<3>(3) = (R_ * B_force) / mass_ + gravity_ + T_force; // quadrotor acceleration
//      du_.segment<3>(3) = 0.0*du_.segment<3>(3); // quadrotor acceleration
      du_.head(3) = R_ * (inertiaInv_ * (B_torque - w_B_.cross(inertia_ * w_B_))); //acceleration by inputs
      du_.tail(3) = gravity_ - T_force / load_mass_ - 0.01*u_.tail(3).norm()*u_.tail(3)/u_.segment<3>(6).norm(); // Load Acceleration

      //Integrate states
      u_ += du_ * this->controlUpdate_dt_; //velocity after timestep

      //Velocity Constraint

      w_I_ = u_.head(3);
      w_IXdt_ = w_I_ * this->controlUpdate_dt_;
      orientation = Math::MathFunc::boxplusI_Frame(orientation, w_IXdt_);
      Math::MathFunc::normalizeQuat(orientation);
      q_.head(4) = orientation;
      q_.segment<3>(4) = q_.segment<3>(4) + u_.segment<3>(3) * this->controlUpdate_dt_; //Quad Position
      q_.tail(3) = q_.tail(3) + u_.tail(3) * this->controlUpdate_dt_; // Load Posittion

      q_.tail(3) = q_.segment<3>(4) + tether_length * (q_.tail(3) - q_.segment<3>(4))/(q_.tail(3) - q_.segment<3>(4)).norm(); // Position Constraint
    }


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
    u_(6) = clip(u_(6), -5.0, 5.0);
    u_(7) = clip(u_(7), -5.0, 5.0);

    getState(state_tp1);

    if (this->isViolatingBoxConstraint(state_tp1))
      termType = TerminationType::terminalState;

    costOUT = 0.004 * std::sqrt(q_.tail(3).norm()) +  // load position
        0.00005 * action_t.norm() +                   // action
        0.00005 * u_.head(3).norm() +                 // angular velocity
        0.00005 * u_.segment<3>(3).norm() +           // linear velocity
        0.00005 * u_.tail(3).norm();                  //

    // visualization
    if (this->visualization_ON_) {

      updateVisualizationFrames();
      visualizer_.drawWorld(visualizeFrame, position, orientation, load_position);
      double waitTime = std::max(0.0, this->controlUpdate_dt_ / realTimeRatio - watch.measure("sim", true));
      watch.start("sim");
      usleep(waitTime * 1e6);

    }
  }


  void stepSim(const Action &action_t) {
    Action thrust = action_t.array().square() * 8.5486e-6;
    thrust = thrust.array().cwiseMax(1e-8);
    Eigen::Matrix<Dtype, 4, 1> genforce;
    genforce = transsThrust2GenForce * thrust;
    B_torque = genforce.segment(0,3);
    B_force << 0.0, 0.0, genforce(3);

    orientation = q_.head(4);
    double angle = 2.0 * std::acos(q_(0));
    position = q_.tail(3);
    R_ = Math::MathFunc::quatToRotMat(orientation);

    du_.tail(3) = (R_ * B_force) / mass_ + gravity_;
    w_B_ = R_.transpose() * u_.head(3);

    /// compute in body coordinate and transform it to world coordinate
//    B_torque += comLocation_.cross(mass_ * R_.transpose() * gravity_);
    du_.head(3) = R_ * (inertiaInv_ * (B_torque - w_B_.cross(inertia_ * w_B_)));

    u_ += du_ * this->controlUpdate_dt_;

    w_I_ = u_.head(3);
    w_IXdt_ = w_I_ * this->controlUpdate_dt_;
    orientation = Math::MathFunc::boxplusI_Frame(orientation, w_IXdt_);
    Math::MathFunc::normalizeQuat(orientation);
    q_.head(4) = orientation;
    q_.tail(3) = q_.tail(3) + u_.tail(3) * this->controlUpdate_dt_;
    w_B_ = R_.transpose() * u_.head(3);

    if ( std::isnan(orientation.norm()) ) {
      std::cout << "u_ " << u_.transpose() << std::endl;
      std::cout << "du_ " << du_.transpose() << std::endl;
      std::cout << "B_torque " << B_torque.transpose() << std::endl;
      std::cout << "orientation " << orientation.transpose() << std::endl;
      std::cout << "state_tp1 " << q_.transpose() << std::endl;
      std::cout << "action_t " << action_t.transpose() << std::endl;
    }
    // visualization

    if (this->visualization_ON_) {

      updateVisualizationFrames();
      visualizer_.drawWorld(visualizeFrame, position, orientation, load_position);
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
    double oriF[4], posiF[3], angVelF[3], linVelF[3], loadPosF[3],loadVelF[3], tetherLength;
    rn_.template sampleOnUnitSphere<4>(oriF);
    rn_.template sampleInUnitSphere<3>(posiF);
    rn_.template sampleInUnitSphere<3>(angVelF);
    rn_.template sampleInUnitSphere<3>(linVelF);
    rn_.template sampleInUnitSphere<3>(loadPosF);
    rn_.template sampleVectorInNormalUniform<3>(loadVelF);


    Quaternion orientation;
    Position position, loadPosition;
    AngularVelocity angularVelocity;
    LinearVelocity linearVelocity, loadVelocity;

    orientation << double(std::abs(oriF[0])), double(oriF[1]), double(oriF[2]), double(oriF[3]);
    rai::Math::MathFunc::normalizeQuat(orientation);
    position << double(posiF[0])*2., double(posiF[1])*2., double(posiF[2])*2.;
    angularVelocity << double(angVelF[0]), double(angVelF[1]), double(angVelF[2]);
    linearVelocity << double(linVelF[0]), double(linVelF[1]), double(linVelF[2]);
    loadPosition << double(loadPosF[0])*tether_length, double(loadPosF[1])*tether_length, -double(std::abs(loadPosF[2]))*tether_length;
    loadVelocity << double(loadVelF[0]), double(loadVelF[1]), double(loadVelF[2]);

    load_direction = rai::Math::MathFunc::quatToRotMat(orientation)*loadPosition;

    q_ << orientation, position, position + load_direction;
    u_ << angularVelocity, linearVelocity, linearVelocity;

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
    Position load_state_v;

    R_.col(0) = stateT.segment(0, 3);
    R_.col(1) = stateT.segment(3, 3);
    R_.col(2) = stateT.segment(6, 3);
    orientation = Math::MathFunc::rotMatToQuat(R_);
    q_.head(4) = orientation;
    q_.segment<3>(4) = state.segment(9, 3) / positionScale_;


    q_.tail(3) << load_state_v(2)*sin(load_state_v(0)),
                  load_state_v(2)*sin(load_state_v(1)),
                  -load_state_v(2)*std::sqrt(1 - sin(load_state_v(0))*sin(load_state_v(0)) - sin(load_state_v(1)) *sin(load_state_v(1)));

    u_.head(3) = state.segment(12, 3) / angVelScale_;
    u_.segment<3>(3) = state.segment(15, 3) / linVelScale_;
    u_.tail(3) = state.tail(3) * linVelScale_;
    du_ = 0.0*du_;
  }

  void getState(State &state) {
    LOG_IF(FATAL, std::isnan(q_.head(4).norm())) << "simulation unstable";
    orientation = q_.head(4);
    Math::MathFunc::normalizeQuat(orientation);
    R_ = Math::MathFunc::quatToRotMat(orientation);

    Position load_state, load_direction_b, load_direction_i; //load direction in body frame
    LinearVelocity load_velocity;
    load_direction_i = q_.segment<3>(7) - q_.segment<3>(4);
    load_direction_b = rai::Math::MathFunc::quatToRotMat(orientation).transpose()*load_direction_i;
    load_state << std::asin(load_direction_b(0)/load_direction_b.norm()), std::asin(load_direction_b(1)/load_direction_b.norm()), load_direction_b.norm();
    load_velocity = u_.tail(3);

    state << R_.col(0), R_.col(1), R_.col(2),
        q_.segment<3>(4) * positionScale_,
        load_state, //parameterized load position
        u_.head(3) * angVelScale_,
        u_.segment<3>(3) * linVelScale_,
        load_velocity * linVelScale_; //parameterized load velocity
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

  GeneralizedCoordinate q_; // generalized state and velocity
  GeneralizedVelocity u_;
  GeneralizedAcceleration du_;

  RotationMatrix R_;
  LinearAcceleration gravity_;

  double orientationScale_, positionScale_, angVelScale_, linVelScale_;
  double actionScale_;
  /// robot parameters
  double length_ = 0.17;
  Position comLocation_;
  double dragCoeff_ = 0.016;
  double mass_ = 0.665;
  double tether_length = 1.0;
  double load_mass_ = 0.08;
  Inertia inertia_;
  Inertia inertiaInv_;
  Quaternion orientation;
  Position position, load_position, load_direction;
  Force B_force, T_force;
  Torque B_torque;
  AngularVelocity w_I_, w_B_;
  LinearVelocity v_I_, vl_I_;
  RandomNumberGenerator<Dtype> rn_;
  EulerVector w_IXdt_;
  static rai_graphics::RAI_graphics graphics;
  static rai_graphics::object::Quadrotor quadrotor;
  static rai_graphics::object::Sphere target;
  static Position targetPosition;
  double visualizationTime = 0;

  Eigen::Matrix4d transsThrust2GenForce;
  Eigen::Matrix4d transsThrust2GenForceInv;


  //Visualization
  StopWatch watch;
  double realTimeRatio = 1;
  static rai::Vis::slungload_Visualizer visualizer_;
  HomogeneousTransform visualizeFrame;

};
}
} /// namespaces
template<typename Dtype>
rai::Position rai::Task::slungloadControl<Dtype>::targetPosition;
template<typename Dtype>
rai::Vis::slungload_Visualizer rai::Task::slungloadControl<Dtype>::visualizer_;
//#endif //RAI_SLUNGLOADCONTROL_HPP