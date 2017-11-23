//
// Created by joonho on 03.04.17.
//

#include <rai/RAI_core>

// Eigen
#include <Eigen/Dense>

// task
#include "rai/tasks/poleBalancing/PoleBalancing.hpp"

// noise model
#include "rai/noiseModel/NormalDistributionNoise.hpp"

// Neural network
#include "functions/customPolicy.hpp"
#include "functions/customValue.hpp"

// algorithm
#include "customAlgo.hpp"
#include <rai/experienceAcquisitor/TrajectoryAcquisitor_MultiThreadBatch.hpp>
#include <rai/experienceAcquisitor/TrajectoryAcquisitor_SingleThreadBatch.hpp>

using namespace std;
using namespace boost;

/// learning states
using Dtype = float;

/// shortcuts
using rai::Task::ActionDim;
using rai::Task::StateDim;
using rai::Task::CommandDim;
using Task = rai::Task::PoleBalancing<Dtype>;
using State = Task::State;
using StateBatch = Task::StateBatch;
using Action = Task::Action;
using ActionBatch = Task::ActionBatch;
using CostBatch = Task::CostBatch;
using VectorXD = Task::VectorXD;
using MatrixXD = Task::MatrixXD;
using Policy_= customPolicy<Dtype, StateDim, ActionDim>;
using Vfunction_ = customValue<Dtype, StateDim>;

using Acquisitor_ = rai::ExpAcq::TrajectoryAcquisitor_MultiThreadBatch<Dtype, StateDim, ActionDim>;
using Noise = rai::Noise::NormalDistributionNoise<Dtype, ActionDim>;
using NoiseCovariance = Eigen::Matrix<Dtype, ActionDim, ActionDim>;

#define nThread 4

int main(int argc, char *argv[]) {

  RAI_init();
  omp_set_num_threads(nThread);

  ////////////////////////// Define task ////////////////////////////
  std::vector<Task> taskVec(nThread, Task(Task::fixed, Task::easy));
  std::vector<rai::Task::Task<Dtype, StateDim, ActionDim, 0> *> taskVector;

  for (auto &task : taskVec) {
    task.setControlUpdate_dt(0.05);
    task.setDiscountFactor(0.995);
    task.setRealTimeFactor(2);
    task.setTimeLimitPerEpisode(25.0);
    taskVector.push_back(&task);
  }

  ////////////////////////// Define Noise Model //////////////////////
  Dtype Stdev = 1;
  NoiseCovariance covariance = NoiseCovariance::Identity() * Stdev;
  std::vector<Noise> noiseVec(nThread, Noise(covariance));
  std::vector<Noise *> noiseVector;
  for (auto &noise : noiseVec)
    noiseVector.push_back(&noise);

  ////////////////////////// Define GRAPH
  std::string graphParamPolicy, graphParamValue;
  graphParamPolicy = "3 1 / 32 32";
  graphParamValue = "3 1 / 32 32";

  std::string shellFilePath;
  std::string cmdPolicy, cmdValue;

  shellFilePath = std::string(std::getenv("RAI_ROOT"))
      + "/applications/examples/DIY/proto/";
  shellFilePath = shellFilePath + "run_python_scripts.sh " + shellFilePath + "protobufGenerator.py";
  if (typeid(Dtype) == typeid(double)) shellFilePath += " 2 ";
  else shellFilePath += " 1 ";
  shellFilePath += RAI_LOG_PATH;

  cmdPolicy = shellFilePath + " " + "cpu" + " customPolicy " + "MLP_"+ " " + graphParamPolicy;
  cmdValue = shellFilePath + " " + "cpu" + " customValue " + "MLP_"+ " " + graphParamValue;

  system(cmdPolicy.c_str());
  system(cmdValue.c_str());

  ////////////////////////// Define Function approximations //////////
  Policy_ policy( RAI_LOG_PATH + "/customPolicy_MLP_.pb",  0.001);
  Vfunction_ Vfunction( RAI_LOG_PATH + "/customValue_MLP_.pb", 0.001);

  ////////////////////////// Acquisitor
  Acquisitor_ acquisitor;

  ////////////////////////// Algorithm ////////////////////////////////
  rai::Algorithm::Algo<Dtype, StateDim, ActionDim>
      algorithm(taskVector, &Vfunction, &policy, noiseVector, &acquisitor, 0.97, 2, 3, 1);
  algorithm.setVisualizationLevel(0);

  /////////////////////// Plotting properties ////////////////////////
  rai::Utils::Graph::FigProp2D
      figurePropertiesEVP("N. Steps Taken", "Performance", "Number of Steps Taken vs Performance");
  rai::Utils::Graph::FigPropPieChart propChart;
  constexpr int loggingInterval = 50;

  ////////////////////////// Learning /////////////////////////////////
  for (int iterationNumber = 0; iterationNumber < 101; iterationNumber++) {

    if (iterationNumber % loggingInterval == 0) {
      algorithm.setVisualizationLevel(0);
      taskVector[0]->enableVideoRecording();
    }
    LOG(INFO) << iterationNumber << "th Iteration";
    algorithm.runOneLoop(5000);

    if (iterationNumber % loggingInterval == 0) {
      algorithm.setVisualizationLevel(0);
      taskVector[0]->disableRecording();
      graph->figure(1, figurePropertiesEVP);
      graph->appendData(1, logger->getData("PerformanceTester/performance", 0),
                        logger->getData("PerformanceTester/performance", 1),
                        logger->getDataSize("PerformanceTester/performance"),
                        rai::Utils::Graph::PlotMethods2D::linespoints,
                        "performance",
                        "lw 2 lc 4 pi 1 pt 5 ps 1");
      graph->drawFigure(1);

    }
  }

  policy.dumpParam(RAI_LOG_PATH + "/policy.txt");
  graph->drawPieChartWith_RAI_Timer(5, timer->getTimedItems(), propChart);
  graph->drawFigure(5, rai::Utils::Graph::OutputFormat::pdf);
  graph->waitForEnter();

}