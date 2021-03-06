//
// Created by joonho on 11/23/17.
//


/// IMPLEMENTATION OF TRPO GAE

#ifndef RAI_CUSTOMALGO_HPP
#define RAI_CUSTOMALGO_HPP

#include <iostream>
#include "glog/logging.h"

#include "rai/tasks/common/Task.hpp"
#include <Eigen/Core>
#include <rai/noiseModel/NormalDistributionNoise.hpp>
#include <rai/noiseModel/NoNoise.hpp>
#include "rai/RAI_core"

// Neural network
//function approximations
#include "rai/function/common/Policy.hpp"
#include "rai/function/common/ValueFunction.hpp"
#include "rai/function/common/StochasticPolicy.hpp"

// acquisitor
#include "rai/experienceAcquisitor/TrajectoryAcquisitor_MultiThreadBatch.hpp"
#include <rai/experienceAcquisitor/TrajectoryAcquisitor_Sequential.hpp>
#include <rai/algorithm/common/PerformanceTester.hpp>
#include <rai/algorithm/common/LearningData.hpp>


#include <Eigen/StdVector>

namespace rai {
namespace Algorithm {
template<typename Dtype, int StateDim, int ActionDim>
class Algo {

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef Eigen::Matrix<Dtype, StateDim, 1> State;
  typedef Eigen::Matrix<Dtype, StateDim, Eigen::Dynamic> StateBatch;
  typedef Eigen::Matrix<Dtype, ActionDim, 1> Action;
  typedef Eigen::Matrix<Dtype, ActionDim, Eigen::Dynamic> ActionBatch;
  typedef Eigen::Matrix<Dtype, 1, 1> Value;
  typedef Eigen::Matrix<Dtype, 1, Eigen::Dynamic> ValueBatch;
  typedef Eigen::Matrix<Dtype, ActionDim, ActionDim> Covariance;
  typedef Eigen::Matrix<Dtype, -1, -1> MatrixXD;
  typedef Eigen::Matrix<Dtype, -1, 1> VectorXD;
  typedef Eigen::Matrix<Dtype, -1, 1> Parameter;

  using Task_ = Task::Task<Dtype, StateDim, ActionDim, 0>;
  using Noise_ = Noise::NormalDistributionNoise<Dtype, ActionDim>;
  using ValueFunc_ = FuncApprox::ValueFunction<Dtype, StateDim>;
  using Policy_ = FuncApprox::StochasticPolicy<Dtype, StateDim, ActionDim>;
  using Trajectory_ = Memory::Trajectory<Dtype, StateDim, ActionDim>;
  using Acquisitor_ = ExpAcq::TrajectoryAcquisitor<Dtype, StateDim, ActionDim>;

  Algo(std::vector<Task_ *> &tasks,
           ValueFunc_ *vfunction,
           Policy_ *policy,
           std::vector<Noise_ *> &noises,
           Acquisitor_ *acquisitor,
           Dtype lambda,
           int K = 0,
           int numofjunctions = 0,
           unsigned testingTrajN = 1,
           Dtype Cov = 1) :
      task_(tasks),
      vfunction_(vfunction),
      policy_(policy),
      noise_(noises),
      acquisitor_(acquisitor),
      lambda_(lambda),
      testingTrajN_(testingTrajN),
      numOfJunct_(numofjunctions),
      numOfBranchPerJunct_(K),
      stepsTaken(0),
      cg_damping(0.1),
      klD_threshold(0.01),
      cov_in(Cov),
      ld_(acquisitor),
      Dataset_(){
    ld_.appendData(&Dataset_);
    parameter_.setZero(policy_->getLPSize());
    policy_->getLP(parameter_);
    timeLimit = task_[0]->timeLimit();
    noNoiseRaw_.resize(task_.size());
    noNoise_.resize(task_.size());

    ///update input stdev
    stdev_o.setOnes();
    stdev_o *= std::sqrt(cov_in);
    policy_->setStdev(stdev_o);

    for (int i = 0; i < task_.size(); i++)
      noiseBasePtr_.push_back(noise_[i]);
    updatePolicyVar();

    for (int i = 0; i < task_.size(); i++)
      noNoise_[i] = &noNoiseRaw_[i];
  };

  ~Algo() {};

  void runOneLoop(int numOfSteps) {
    iterNumber_++;
    tester_.testPerformance(task_,
                            noiseBasePtr_,
                            policy_,
                            task_[0]->timeLimit(),
                            testingTrajN_,
                            acquisitor_->stepsTaken(),
                            vis_lv_,
                            std::to_string(iterNumber_));
    LOG(INFO) << "Simulation";
    ld_.acquireVineTrajForNTimeSteps(task_,
                                     noiseBasePtr_,
                                     policy_,
                                     numOfSteps,
                                     numOfJunct_,
                                     numOfBranchPerJunct_,
                                     vfunction_,
                                     vis_lv_);
    LOG(INFO) << "Vfunction update";
    VFupdate();
    LOG(INFO) << "Policy update";
    TRPOUpdater();
  }

  void set_cg_daming(Dtype cgd) { cg_damping = cgd; }
  void set_kl_thres(Dtype thres) { klD_threshold = thres; }
  void setVisualizationLevel(int vis_lv) { vis_lv_ = vis_lv; }

 private:

  void VFupdate() {
    ValueBatch valuePrev(ld_.stateBat.cols());
    Dtype loss;
    vfunction_->forward(ld_.stateBat, valuePrev);
    mixfrac = 0.1;
    Utils::timer->startTimer("Vfunction update");
    ld_.valueBat = ld_.valueBat * mixfrac + valuePrev * (1 - mixfrac);
    for (int i = 0; i < 50; i++)
      loss = vfunction_->performOneSolverIter(ld_.stateBat, ld_.valueBat);
    Utils::timer->stopTimer("Vfunction update");
    LOG(INFO) << "value function loss : " << loss;
  }

  void TRPOUpdater() {
    Utils::timer->startTimer("policy Training");
    /// Update Advantage
    ld_.computeAdvantage(task_[0],vfunction_,lambda_);

    /// Update Policy
    Parameter policy_grad = Parameter::Zero(parameter_.rows());
    Parameter Nat_grad = Parameter::Zero(parameter_.rows());
    Parameter fullstep = Parameter::Zero(parameter_.rows());

    policy_->getLP(parameter_);
    policy_->getStdev(stdev_o);

    LOG(INFO) << "stdev :" << stdev_o.transpose();
    Utils::timer->startTimer("Gradient computation");
    policy_->TRPOpg(Dataset_, stdev_o, policy_grad);
    Utils::timer->stopTimer("Gradient computation");
    LOG_IF(FATAL, isnan(policy_grad.norm())) << "policy_grad is nan!" << policy_grad.transpose();

    Utils::timer->startTimer("Conjugate gradient");
    Dtype CGerror = policy_->TRPOcg(Dataset_,
                                    stdev_o,
                                    policy_grad,
                                    Nat_grad); // TODO : test
    Utils::timer->stopTimer("Conjugate gradient");
    LOG(INFO) << "conjugate grad error :" << CGerror;

    Dtype beta = std::sqrt(2 * klD_threshold / Nat_grad.dot(policy_grad));
    Nat_grad = -Nat_grad;

    Dtype expected = -policy_grad.dot(Nat_grad);
    fullstep = beta * Nat_grad;

    Utils::timer->startTimer("lineSearch");
    parameter_ += line_search(fullstep, expected);
    Utils::timer->stopTimer("lineSearch");

    policy_->setLP(parameter_);
    updatePolicyVar();/// save stdev & Update Noise Covariance
    Utils::timer->stopTimer("policy Training");
  }

  void updatePolicyVar() {
    Action temp;
    policy_->getStdev(stdev_o);
    temp = stdev_o;
    temp = temp.array().square(); //var
    policycov = temp.asDiagonal();
    for (auto &noise : noise_)
      noise->updateCovariance(policycov);
  }

  inline VectorXD line_search(VectorXD &initialUpdate, Dtype &expected_improve) {

    int max_shrinks = 20;
    Dtype shrink_multiplier = 0.7;
    VectorXD paramUpdate = initialUpdate;
    VectorXD bestParamUpdate = initialUpdate;
    VectorXD parameterTest = parameter_ + initialUpdate;

    Dtype Initialcost = costOfParam(parameterTest);
    Dtype lowestCost = Initialcost;

    int i, best_indx = 0;
    for (i = 1; i < max_shrinks; i++) {
      paramUpdate = paramUpdate * shrink_multiplier;
      parameterTest = parameter_ + paramUpdate;

      Dtype cost = costOfParam(parameterTest);
      if (lowestCost > cost) {
        bestParamUpdate = paramUpdate;
        lowestCost = cost;
        best_indx = i;
      }
    }
    LOG(INFO) << "best_idx :" << best_indx;

    return bestParamUpdate;
  }

  inline Dtype costOfParam(VectorXD &param) {
    policy_->setLP(param);
    return policy_->TRPOloss(Dataset_, stdev_o);
  }

  /////////////////////////// Core //////////////////////////////////////////
  std::vector<Task_ *> task_;
  std::vector<Noise_ *> noise_;
  std::vector<Noise::Noise<Dtype, ActionDim> *> noiseBasePtr_;
  std::vector<Noise::Noise<Dtype, ActionDim> *> noNoise_;
  std::vector<Noise::NoNoise<Dtype, ActionDim>> noNoiseRaw_;
  ValueFunc_ *vfunction_;
  Policy_ *policy_;
  Acquisitor_ *acquisitor_;
  Dtype lambda_;
  PerformanceTester<Dtype, StateDim, ActionDim> tester_;
  LearningData<Dtype, StateDim, ActionDim> ld_;
  historyWithAdvantage<Dtype, StateDim, ActionDim> Dataset_;

  /////////////////////////// Algorithmic parameter ///////////////////
  int stepsTaken;
  int numOfJunct_;
  int numOfBranchPerJunct_;
  Dtype cov_in;
  Dtype mixfrac;
  Dtype klD_threshold;
  Dtype cg_damping;
  double timeLimit;


  /////////////////////////// Policy parameter
  VectorXD parameter_;
  Action stdev_o;
  Covariance policycov;

  /////////////////////////// plotting
  int iterNumber_ = 0;

  /////////////////////////// random number generator
  RandomNumberGenerator<Dtype> rn_;

  ///////////////////////////testing
  unsigned testingTrajN_;

  /////////////////////////// visualization
  int vis_lv_ = 0;
};

}
}


#endif //RAI_CUSTOMALGO_HPP
