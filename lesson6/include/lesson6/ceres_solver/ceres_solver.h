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

#ifndef LESSON6_CERES_SOLVER_CERES_SOLVER_H
#define LESSON6_CERES_SOLVER_CERES_SOLVER_H

#include <ros/ros.h>
#include <vector>
#include <map>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "ceres/ceres.h"

#include "open_karto/Mapper.h"

// The state for each vertex in the pose graph.
struct Pose2d
{
  double x;
  double y;
  double yaw_radians;
};

// The constraint between two vertices in the pose graph. The constraint is the
// transformation from vertex id_begin to vertex id_end.
struct Constraint2d
{
  int id_begin;
  int id_end;

  double x;
  double y;
  double yaw_radians;

  // The inverse of the covariance matrix for the measurement. The order of the
  // entries are x, y, and yaw.
  Eigen::Matrix3d information;
};

class CeresSolver : public karto::ScanSolver
{
public:
  CeresSolver();
  virtual ~CeresSolver();

  virtual void Clear();
  virtual void Compute();
  virtual const karto::ScanSolver::IdPoseVector &GetCorrections() const;

  virtual void AddNode(karto::Vertex<karto::LocalizedRangeScan> *pVertex);
  virtual void AddConstraint(karto::Edge<karto::LocalizedRangeScan> *pEdge);

private:
  void BuildOptimizationProblem(const std::vector<Constraint2d> &constraints, std::map<int, Pose2d> *poses, ceres::Problem *problem);
  bool SolveOptimizationProblem(ceres::Problem *problem);

  std::map<int, Pose2d> poses_;
  std::vector<Constraint2d> constraints_;

  karto::ScanSolver::IdPoseVector corrections_;
};

#endif // LESSON6_CERES_SOLVER_H