/*********************************************************************
*
*  Copyright (c) 2017, Saurav Agarwal
*  All rights reserved.
*
*********************************************************************/

/* Authors: Saurav Agarwal */

#include <limits>

#include <open_karto/Karto.h>
#include <ros/console.h>

#include "lesson6/gtsam_solver/gtsam_solver.h"

using namespace gtsam;

GTSAMSolver::GTSAMSolver()
{
  // add the prior on the first node which is known
  noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Sigmas(Vector3(1e-6, 1e-6, 1e-8));
  graph_.emplace_shared<PriorFactor<Pose2>>(0, Pose2(0, 0, 0), priorNoise);
}

GTSAMSolver::~GTSAMSolver()
{
}

void GTSAMSolver::AddNode(karto::Vertex<karto::LocalizedRangeScan> *pVertex)
{
  karto::Pose2 odom = pVertex->GetObject()->GetCorrectedPose();
  initialGuess_.insert(pVertex->GetObject()->GetUniqueId(),
                       Pose2(odom.GetX(), odom.GetY(), odom.GetHeading()));
  graphNodes_.push_back(Eigen::Vector2d(odom.GetX(), odom.GetY()));
  ROS_DEBUG("[gtsam] Adding node %d.", pVertex->GetObject()->GetUniqueId());
}

void GTSAMSolver::AddConstraint(karto::Edge<karto::LocalizedRangeScan> *pEdge)
{
  // Set source and target
  int sourceID = pEdge->GetSource()->GetObject()->GetUniqueId();
  int targetID = pEdge->GetTarget()->GetObject()->GetUniqueId();

  // Set the measurement (poseGraphEdge distance between vertices)
  karto::LinkInfo *pLinkInfo = (karto::LinkInfo *)(pEdge->GetLabel());
  karto::Pose2 diff = pLinkInfo->GetPoseDifference();

  // Set the covariance of the measurement
  karto::Matrix3 precisionMatrix = pLinkInfo->GetCovariance();

  Eigen::Matrix<double, 3, 3> cov;
  cov(0, 0) = precisionMatrix(0, 0);
  cov(0, 1) = cov(1, 0) = precisionMatrix(0, 1);
  cov(0, 2) = cov(2, 0) = precisionMatrix(0, 2);
  cov(1, 1) = precisionMatrix(1, 1);
  cov(1, 2) = cov(2, 1) = precisionMatrix(1, 2);
  cov(2, 2) = precisionMatrix(2, 2);
  noiseModel::Gaussian::shared_ptr model = noiseModel::Diagonal::Covariance(cov);

  // Add odometry factors
  // Create odometry (Between) factors between consecutive poses
  graph_.emplace_shared<BetweenFactor<Pose2>>(sourceID, targetID, Pose2(diff.GetX(), diff.GetY(), diff.GetHeading()), model);

  // Add the constraint to the optimizer
  ROS_DEBUG("[gtsam] Adding Edge from node %d to node %d.", sourceID, targetID);
}

void GTSAMSolver::Compute()
{
  corrections_.clear();
  graphNodes_.clear();

  ROS_INFO("[gtsam] Calling gtsam for Optimization");

  LevenbergMarquardtParams parameters;

  // Stop iterating once the change in error between steps is less than this value
  parameters.relativeErrorTol = 1e-5;

  // Do not perform more than N iteration steps
  parameters.maxIterations = 500;

  // Create the optimizer ...
  LevenbergMarquardtOptimizer optimizer(graph_, initialGuess_, parameters);

  // ... and optimize
  Values result = optimizer.optimize();

  Values::ConstFiltered<Pose2> viewPose2 = result.filter<Pose2>();

  // put values into corrections container
  for (const Values::ConstFiltered<Pose2>::KeyValuePair &key_value : viewPose2)
  {
    karto::Pose2 pose(key_value.value.x(), key_value.value.y(), key_value.value.theta());
    corrections_.push_back(std::make_pair(key_value.key, pose));
    graphNodes_.push_back(Eigen::Vector2d(key_value.value.x(), key_value.value.y()));
  }
}

void GTSAMSolver::getGraph(std::vector<Eigen::Vector2d> &nodes, std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> &edges)
{
}

void GTSAMSolver::Clear()
{
  corrections_.clear();
}

const karto::ScanSolver::IdPoseVector &GTSAMSolver::GetCorrections() const
{
  return corrections_;
}
