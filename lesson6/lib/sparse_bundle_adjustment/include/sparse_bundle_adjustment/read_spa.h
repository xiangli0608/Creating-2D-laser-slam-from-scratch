/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

// convert a file in Freiburg's VERTEX / EDGE format into a set of constraints

#ifndef SBA_READ_SPA_H
#define SBA_READ_SPA_H

#ifndef EIGEN_USE_NEW_STDVECTOR
#define EIGEN_USE_NEW_STDVECTOR
#endif // EIGEN_USE_NEW_STDVECTOR

#define EIGEN_DEFAULT_IO_FORMAT Eigen::IOFormat(10)
#include <Eigen/Eigen>
#include <vector>

struct tinfo
{
  int pn;                       // index of point
  int fn0, fn1;                 // index of frames
  int fpn0, fpn1;               // local point index in frame
  double x0,y0,z0,u0,v0;        // local XYZ coords and projection
  double x1,y1,z1,u1,v1;        // local XYZ coords and projection
};

int 
ReadSPAFile(char *fin,          // input file
            // node translation
            std::vector< Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > &ntrans,
            // node rotation
            std::vector< Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > &nqrot,
            // constraint indices
            std::vector< Eigen::Vector2i, Eigen::aligned_allocator<Eigen::Vector2i> > &cind,
            // constraint local translation 
            std::vector< Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > &ctrans,
            // constraint local rotation as quaternion
            std::vector< Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > &cqrot,
            // constraint covariance
            std::vector< Eigen::Matrix<double,6,6>, Eigen::aligned_allocator<Eigen::Matrix<double,6,6> > > &cvar,
            // track info: point projections, see format description in IntelSeattle files
            std::vector<struct tinfo> &tracks
  );

int 
ReadSPA2dFile(char *fin,          // input file
            // node translation
            std::vector< Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > &ntrans,
            // node rotation
            std::vector< double > &nqrot,
            // constraint indices
            std::vector< Eigen::Vector2i, Eigen::aligned_allocator<Eigen::Vector2i> > &cind,
            // constraint local translation 
            std::vector< Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > &ctrans,
            // constraint local rotation as quaternion
            std::vector< double > &cqrot,
            // constraint covariance
            std::vector< Eigen::Matrix<double,3,3>, Eigen::aligned_allocator<Eigen::Matrix<double,3,3> > > &cvar,
            // scan points
            std::vector< std::vector< Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > > &scans
  );

#endif
