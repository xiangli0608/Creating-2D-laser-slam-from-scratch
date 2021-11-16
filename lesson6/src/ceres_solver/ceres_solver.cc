// Ceres Solver - A fast non-linear least squares minimizer
// Copyright 2016 Google Inc. All rights reserved.
// http://ceres-solver.org/
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of Google Inc. nor the names of its contributors may be
//   used to endorse or promote products derived from this software without
//   specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: vitus@google.com (Michael Vitus)
//
// Cost function for a 2D pose graph formulation.

#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "lesson6/ceres_solver/ceres_solver.h"
#include "lesson6/ceres_solver/angle_local_parameterization.h"
#include "lesson6/ceres_solver/pose_graph_2d_error_term.h"

// 指定角度相加时使用的方法
CeresSolver::CeresSolver()
{
}

CeresSolver::~CeresSolver()
{
}

void CeresSolver::Clear()
{
  corrections_.clear();
}

const karto::ScanSolver::IdPoseVector &CeresSolver::GetCorrections() const
{
  return corrections_;
}

// 添加节点
void CeresSolver::AddNode(karto::Vertex<karto::LocalizedRangeScan> *pVertex)
{
  karto::Pose2 pose = pVertex->GetObject()->GetCorrectedPose();
  int pose_id = pVertex->GetObject()->GetUniqueId();
  Pose2d pose2d;
  pose2d.x = pose.GetX();
  pose2d.y = pose.GetY();
  pose2d.yaw_radians = pose.GetHeading();
  poses_[pose_id] = pose2d;

  ROS_DEBUG("[ceres] AddNode %d", pVertex->GetObject()->GetUniqueId());
}

// 添加约束
void CeresSolver::AddConstraint(karto::Edge<karto::LocalizedRangeScan> *pEdge)
{
  karto::LocalizedRangeScan *pSource = pEdge->GetSource()->GetObject();
  karto::LocalizedRangeScan *pTarget = pEdge->GetTarget()->GetObject();
  karto::LinkInfo *pLinkInfo = (karto::LinkInfo *)(pEdge->GetLabel());

  karto::Pose2 diff = pLinkInfo->GetPoseDifference();
  karto::Matrix3 precisionMatrix = pLinkInfo->GetCovariance().Inverse();
  Eigen::Matrix<double, 3, 3> info;
  info(0, 0) = precisionMatrix(0, 0);
  info(0, 1) = info(1, 0) = precisionMatrix(0, 1);
  info(0, 2) = info(2, 0) = precisionMatrix(0, 2);
  info(1, 1) = precisionMatrix(1, 1);
  info(1, 2) = info(2, 1) = precisionMatrix(1, 2);
  info(2, 2) = precisionMatrix(2, 2);
  Eigen::Vector3d measurement(diff.GetX(), diff.GetY(), diff.GetHeading());

  Constraint2d constraint2d;
  constraint2d.id_begin = pSource->GetUniqueId();
  constraint2d.id_end = pTarget->GetUniqueId();
  constraint2d.x = measurement(0);
  constraint2d.y = measurement(1);
  constraint2d.yaw_radians = measurement(2);
  constraint2d.information = info;
  constraints_.push_back(constraint2d);

  ROS_DEBUG("[ceres] AddConstraint %d  %d", pSource->GetUniqueId(), pTarget->GetUniqueId());
}

// 优化求解
void CeresSolver::Compute()
{
  corrections_.clear();

  ROS_INFO("[ceres] Calling ceres for Optimization");
  ceres::Problem problem;
  BuildOptimizationProblem(constraints_, &poses_, &problem);
  SolveOptimizationProblem(&problem);
  ROS_INFO("[ceres] Optimization finished\n");

  for (std::map<int, Pose2d>::const_iterator pose_iter = poses_.begin(); pose_iter != poses_.end(); ++pose_iter)
  {
    karto::Pose2 pose(pose_iter->second.x, pose_iter->second.y, pose_iter->second.yaw_radians);
    corrections_.push_back(std::make_pair(pose_iter->first, pose));
  }
}

/**
 * @brief 从位姿图约束构造非线性最小二乘优化问题
 * 
 * @param constraints 位姿图约束
 * @param poses 节点位姿
 * @param problem 优化问题
 */
void CeresSolver::BuildOptimizationProblem(const std::vector<Constraint2d> &constraints, std::map<int, Pose2d> *poses,
                                           ceres::Problem *problem)
{
  if (constraints.empty())
  {
    std::cout << "No constraints, no problem to optimize.";
    return;
  }

  // 定义了角度的更新方式
  ceres::LocalParameterization *angle_local_parameterization = AngleLocalParameterization::Create();

  for (std::vector<Constraint2d>::const_iterator constraints_iter = constraints.begin();
       constraints_iter != constraints.end(); ++constraints_iter)
  {
    const Constraint2d &constraint = *constraints_iter;

    std::map<int, Pose2d>::iterator pose_begin_iter = poses->find(constraint.id_begin);
    std::map<int, Pose2d>::iterator pose_end_iter = poses->find(constraint.id_end);

    // 对information开根号
    const Eigen::Matrix3d sqrt_information = constraint.information.llt().matrixL();

    // Ceres will take ownership of the pointer.
    ceres::CostFunction *cost_function =
        PoseGraph2dErrorTerm::Create(constraint.x, constraint.y, constraint.yaw_radians, sqrt_information);
    
    problem->AddResidualBlock(cost_function, nullptr,
                              &pose_begin_iter->second.x,
                              &pose_begin_iter->second.y,
                              &pose_begin_iter->second.yaw_radians,
                              &pose_end_iter->second.x,
                              &pose_end_iter->second.y,
                              &pose_end_iter->second.yaw_radians);
    problem->SetParameterization(&pose_begin_iter->second.yaw_radians, angle_local_parameterization);
    problem->SetParameterization(&pose_end_iter->second.yaw_radians, angle_local_parameterization);
  }

  // The pose graph optimization problem has three DOFs that are not fully
  // constrained. This is typically referred to as gauge freedom. You can apply
  // a rigid body transformation to all the nodes and the optimization problem
  // will still have the exact same cost. The Levenberg-Marquardt algorithm has
  // internal damping which mitigate this issue, but it is better to properly
  // constrain the gauge freedom. This can be done by setting one of the poses
  // as constant so the optimizer cannot change it.
  std::map<int, Pose2d>::iterator pose_start_iter = poses->begin();
  problem->SetParameterBlockConstant(&pose_start_iter->second.x);
  problem->SetParameterBlockConstant(&pose_start_iter->second.y);
  problem->SetParameterBlockConstant(&pose_start_iter->second.yaw_radians);
}

// Returns true if the solve was successful.
bool CeresSolver::SolveOptimizationProblem(ceres::Problem *problem)
{
  assert(problem != NULL);

  ceres::Solver::Options options;
  options.max_num_iterations = 100;
  options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;

  ceres::Solver::Summary summary;
  ceres::Solve(options, problem, &summary);

  std::cout << summary.BriefReport() << '\n';
  return summary.IsSolutionUsable();
}
