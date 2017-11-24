//
// Created by joonho on 11/23/17.
//

#ifndef RAI_CUSTOMVALUE_HPP
#define RAI_CUSTOMVALUE_HPP
#include "rai/function/common/ValueFunction.hpp"
#include "rai/function/tensorflow/common/ParameterizedFunction_TensorFlow.hpp"

template<typename Dtype, int stateDim>
class customValue : public virtual rai::FuncApprox::ParameterizedFunction_TensorFlow<Dtype, stateDim, 1>,
                    public virtual rai::FuncApprox::ValueFunction<Dtype, stateDim> {

 public:
  using ValueFunctionBase = rai::FuncApprox::ValueFunction<Dtype, stateDim>;
  using Pfunction_tensorflow = rai::FuncApprox::ParameterizedFunction_TensorFlow<Dtype, stateDim, 1>;

  typedef typename ValueFunctionBase::State State;
  typedef typename ValueFunctionBase::StateBatch StateBatch;
  typedef typename ValueFunctionBase::Value Value;
  typedef typename ValueFunctionBase::ValueBatch ValueBatch;
  typedef typename ValueFunctionBase::Gradient Gradient;
  typedef typename ValueFunctionBase::Jacobian Jacobian;

  customValue(std::string pathToGraphDefProtobuf, Dtype learningRate = 1e-3) :
      Pfunction_tensorflow::ParameterizedFunction_TensorFlow(pathToGraphDefProtobuf, learningRate) {
  }

  ~customValue() {};

  virtual void forward(State &state, Dtype &value) {
    std::vector<MatrixXD> vectorOfOutputs;
    this->tf_->run({{"state", state}},
                   {"value"}, {}, vectorOfOutputs);
    value = vectorOfOutputs[0](0);
  }

  virtual void forward(StateBatch &states, ValueBatch &values) {
    std::vector<MatrixXD> vectorOfOutputs;
    this->tf_->run({{"state", states}},
                   {"value"}, {}, vectorOfOutputs);
    values = vectorOfOutputs[0];
  }

  virtual Dtype performOneSolverIter(StateBatch &states, ValueBatch &values) {
    std::vector<MatrixXD> loss, dummy;
    this->tf_->run({{"state", states},
                    {"targetValue", values},
                    {"trainUsingTargetValue/learningRate", this->learningRate_}},
                   {"trainUsingTargetValue/loss"},
                   {"trainUsingTargetValue/solver"}, loss);
    return loss[0](0);
  }

 protected:
  using MatrixXD = typename rai::FuncApprox::TensorFlowNeuralNetwork<Dtype>::MatrixXD;

};
#endif //RAI_CUSTOMVALUE_HPP
