//
// Created by joonho on 11/23/17.
//

#ifndef RAI_CUSTOMPOLICY_HPP
#define RAI_CUSTOMPOLICY_HPP

#include <rai/noiseModel/NormalDistributionNoise.hpp>
#include "rai/function/common/StochasticPolicy.hpp"
#include "rai/function/tensorflow/common/ParameterizedFunction_TensorFlow.hpp"

template<typename Dtype, int stateDim, int actionDim>
class customPolicy : public virtual rai::FuncApprox::StochasticPolicy<Dtype, stateDim, actionDim>,
                                    public virtual rai::FuncApprox::ParameterizedFunction_TensorFlow<Dtype, stateDim, actionDim> {
 public:
  typedef Eigen::Matrix<Dtype, -1, 1> VectorXD;
  typedef Eigen::Matrix<Dtype, 2 * actionDim, -1> JacobianWRTparam;

  using Advantages = Eigen::Matrix<Dtype, 1, Eigen::Dynamic>;
  using PolicyBase = rai::FuncApprox::StochasticPolicy<Dtype, stateDim, actionDim>;
  using Pfunction_tensorflow = rai::FuncApprox::ParameterizedFunction_TensorFlow<Dtype, stateDim, actionDim>;
  using Noise_ = rai::Noise::NormalDistributionNoise<Dtype, actionDim>;

  typedef typename PolicyBase::State State;
  typedef typename PolicyBase::StateBatch StateBatch;
  typedef typename PolicyBase::Action Action;
  typedef typename PolicyBase::ActionBatch ActionBatch;

  typedef typename PolicyBase::Gradient Gradient;
  typedef typename PolicyBase::Jacobian Jacobian;
  typedef typename PolicyBase::Jacobian JacobianWRTstate;

  typedef typename PolicyBase::Tensor1D Tensor1D;
  typedef typename PolicyBase::Tensor2D Tensor2D;
  typedef typename PolicyBase::Tensor3D Tensor3D;
  typedef typename PolicyBase::historyWithA historyWithA_;

  customPolicy(std::string pathToGraphDefProtobuf, Dtype learningRate = 1e-3) :
      Pfunction_tensorflow::ParameterizedFunction_TensorFlow(pathToGraphDefProtobuf, learningRate) {
  }

  virtual void getdistribution(StateBatch &states, ActionBatch &means, Action &stdev) {
    std::vector<MatrixXD> vectorOfOutputs;
    this->tf_->run({{"state", states}}, {"action", "stdev"}, {}, vectorOfOutputs);
    means = vectorOfOutputs[0];
    stdev = vectorOfOutputs[1].col(0);
  }
  ///TRPO

  virtual void TRPOpg(historyWithA_ &batch,
                      Action &Stdev,
                      VectorXD &grad) {
    std::vector<MatrixXD> vectorOfOutputs;
    Tensor1D StdevT(Stdev, {Stdev.rows()}, "stdv_o");

    this->tf_->run({batch.states,
                    batch.actions,
                    batch.actionNoises,
                    batch.advantages,
                    StdevT},
                   {"Algo/TRPO/Pg"},
                   {},
                   vectorOfOutputs);

    grad = vectorOfOutputs[0];
  }

  virtual Dtype TRPOcg(historyWithA_ &batch,
                       Action &Stdev,
                       VectorXD &grad, VectorXD &getng) {
    std::vector<MatrixXD> vectorOfOutputs;
    Tensor1D StdevT(Stdev, {Stdev.rows()}, "stdv_o");
    Tensor1D gradT(grad, {grad.rows()}, "tangent");

    this->tf_->run({batch.states,
                    batch.actions,
                    batch.actionNoises,
                    batch.advantages,
                    StdevT,
                    gradT},
                   {"Algo/TRPO/Cg", "Algo/TRPO/Cgerror"}, {}, vectorOfOutputs);
    getng = vectorOfOutputs[0];
    return  vectorOfOutputs[1](0);
  }

  virtual Dtype TRPOloss(historyWithA_ &batch,
                         Action &Stdev) {
    std::vector<MatrixXD> vectorOfOutputs;
    Tensor1D StdevT(Stdev, {Stdev.rows()}, "stdv_o");

    this->tf_->run({batch.states,
                    batch.actions,
                    batch.actionNoises,
                    batch.advantages,
                    StdevT},
                   {"Algo/TRPO/loss"},
                   {}, vectorOfOutputs);

    return vectorOfOutputs[0](0);
  }

  virtual void setStdev(const Action &Stdev) {
    std::vector<MatrixXD> dummy;
    this->tf_->run({{"Stdev_placeholder", Stdev}}, {}, {"assignStdev"}, dummy);
  }

  virtual void getStdev(Action &Stdev) {
    std::vector<MatrixXD> vectorOfOutputs;
    this->tf_->run({{"Stdev_placeholder", Stdev}}, {"getStdev"}, {}, vectorOfOutputs);
    Stdev = vectorOfOutputs[0];
  }

  virtual void forward(State &state, Action &action) {
    std::vector<MatrixXD> vectorOfOutputs;
    this->tf_->forward({{"state", state}},
                       {"action"}, vectorOfOutputs);

    action = vectorOfOutputs[0];
  }
  virtual void forward(StateBatch &state, ActionBatch &action) {
    std::vector<MatrixXD> vectorOfOutputs;
    this->tf_->forward({{"state", state}},
                       {"action"}, vectorOfOutputs);
    action = vectorOfOutputs[0];
  }

  virtual void forward(Tensor3D &states, Tensor3D &actions) {
    std::vector<tensorflow::Tensor> vectorOfOutputs;
    this->tf_->forward({states}, {"action"}, vectorOfOutputs);
    actions.copyDataFrom(vectorOfOutputs[0]);
  }

  virtual Dtype performOneSolverIter(StateBatch &states, ActionBatch &actions) {
    std::vector<MatrixXD> loss, dummy;
    this->tf_->run({{"state", states},
                    {"targetAction", actions},
                    {"trainUsingTargetAction/learningRate", this->learningRate_}}, {"trainUsingTargetAction/loss"},
                   {"trainUsingTargetAction/solver"}, loss);
    this->tf_->run({{"state", states}}, {},
                   {"action"}, dummy);
    return loss[0](0);
  }

 protected:
  using MatrixXD = typename rai::FuncApprox::TensorFlowNeuralNetwork<Dtype>::MatrixXD;

};

#endif //RAI_CUSTOMPOLICY_HPP
