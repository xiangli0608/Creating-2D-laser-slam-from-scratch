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

//
// Sparse pose adjustment, 2d version
//

#ifndef _SPA2D_H_
#define _SPA2D_H_

#ifndef EIGEN_USE_NEW_STDVECTOR
#define EIGEN_USE_NEW_STDVECTOR
#endif // EIGEN_USE_NEW_STDVECTOR

#include <stdio.h>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>
#ifndef EIGEN_USE_NEW_STDVECTOR
#define EIGEN_USE_NEW_STDVECTOR
#endif
#include <Eigen/StdVector>
#include <vector>

// sparse Cholesky
#include <sparse_bundle_adjustment/csparse.h>
// block jacobian pcg
#include <sparse_bundle_adjustment/bpcg/bpcg.h>

// Defines for methods to use with doSBA().
#define SBA_DENSE_CHOLESKY 0
#define SBA_SPARSE_CHOLESKY 1
#define SBA_GRADIENT 2
#define SBA_BLOCK_JACOBIAN_PCG 3


// put things into a namespace
namespace sba
{

  /// NODE2d holds graph nodes corresponding to frames, for use in
  /// sparse bundle adjustment 

    /// type double must be <double> or <float>
    ///
    /// Each node has a 6DOF pose, encoded as a translation vector and
    /// rotation unit quaternion (Eigen classes).  These represent the
    /// pose of the node in the world frame.
    /// 
    /// The pose generates a 3x4 homogenous transform, taking a point
    /// in world coordinates into the node coordinates.
    /// 
    /// Additionally a 3x4 homogenous transform is composed from the
    /// pose transform and a projection transform to the frame image
    /// coordinates. 
    ///
    /// Projections from points to features are in a list.  There
    /// should also be a reverse index from features to projections
    /// and points, so that matched features can tie in to points.
    ///

  class Node2d
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // needed for 16B alignment

    /// node id - somewhat redundant, but can be useful, e.g., in KARTO links
    int nodeId;

    /// 6DOF pose as a unit quaternion and translation vector
    Eigen::Matrix<double,3,1> trans;    // homogeneous coordinates, last element is 1.0
    double arot;                // angle in radians, normalized to [-pi,+pi]
    /// Normalize to [-pi,+pi]
    inline void normArot()
    { 
      if (arot > M_PI) arot -= 2.0*M_PI;
      if (arot < -M_PI) arot += 2.0*M_PI;
    }

    /// Resultant transform from world to node coordinates;
    Eigen::Matrix<double,2,3> w2n;
    void setTransform();

    /// Covariance matrix, 3x3.  Variables are [trans,rot], with the
    /// rotational part being the x parameter of the unit
    /// quaternion
    //    Eigen::Matrix<double,3,3> covar;

    /// Derivatives of the rotation matrix transpose wrt quaternion xyz, used for
    /// calculating Jacobian wrt pose of a projection.
    Eigen::Matrix2d dRdx;

    void setDr();               // set local angle derivatives

    /// For SPA, is this camera fixed or free?
    bool isFixed;

    /// 3DOF pose as a unit quaternion and translation vector, saving
    /// for LM step
    Eigen::Matrix<double,3,1> oldtrans; // homogeneous coordinates, last element is 1.0
    double oldarot;             // angle
  };


  /// CONP2 holds a constraint measurement of a pose to a pose.
  /// They are a repository for links between poses within a frame,
  /// with aux info such as jacobians

  class Con2dP2
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // needed for 16B alignment

    /// Reference pose index
    int ndr;

    /// Node2d index for the second node
    int nd1;

    /// Mean vector, quaternion (inverse) and precision matrix for this constraint
    Eigen::Vector2d tmean;
    double amean;
    Eigen::Matrix<double,3,3> prec;

    /// error
    Eigen::Matrix<double,3,1> err;
    /// calculates projection error and stores it in <err>
    inline double calcErr(const Node2d &nd0, const Node2d &nd1);

    /// calculate error in distance only, no weighting
    double calcErrDist(const Node2d &nd0, const Node2d &nd1);


    /// jacobian with respect to frames; uses dR'/dq from Node2d calculation
    Eigen::Matrix<double,3,3> J0,J0t,J1,J1t;

    /// scaling factor for quaternion derivatives relative to translational ones;
    /// not sure if this is needed, it's close to 1.0
    static constexpr double qScale = 1.0;

    /// dpc/dq = dR'/dq [pw-t], in homogeneous form, with q a quaternion param
    /// 

    /// dpc/dx = -R' * [1 0 0]', in homogeneous form, with x a translation param
    /// 

    /// d(px/pz)/du = [ pz dpx/du - px dpz/du ] / pz^2,
    /// works for all variables
    ///
    void setJacobians(std::vector<Node2d,Eigen::aligned_allocator<Node2d> > &nodes);

    /// valid or not (could be out of bounds)
    bool isValid;
  };



  /// SysSPA2d holds a set of nodes and constraints for sparse pose adjustment

  class SysSPA2d
    {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      /// constructor
      SysSPA2d() { nFixed = 1; verbose = false; lambda = 1.0e-4, print_iros_stats=false; }

      /// add a node at a pose
      /// <pos> is x,y,th, with th in radians
      int addNode(const Vector3d &pos, int id);
      void removeNode(int id);

      // add a constraint
      // <nd0>, <nd1> are node id's
      // <mean> is x,y,th, with th in radians
      // <prec> is a 3x3 precision matrix (inverse covariance
      bool addConstraint(int nd0, int nd1, const Vector3d &mean, const Matrix3d &prec);
      bool removeConstraint(int ndi0, int ndi1);


      /// set of nodes (camera frames) for SPA system, indexed by position;
      std::vector<Node2d,Eigen::aligned_allocator<Node2d> > nodes;
      std::vector<Node2d,Eigen::aligned_allocator<Node2d> > getNodes()
        { return nodes; }

      /// set of point scans, corresponding to nodes
      std::vector< std::vector< Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > > scans;

      /// Number of fixed nodes
      int nFixed;               

      /// Set of P2 constraints
      std::vector<Con2dP2,Eigen::aligned_allocator<Con2dP2> >  p2cons;

      /// calculate the error in the system;
      ///   if <tcost> is true, just the distance error without weighting
      double calcCost(bool tcost = false);
      /// calculate error assuming outliers > dist
      double calcCost(double dist);
      double errcost;


      /// print some system stats
      void printStats();
      void writeSparseA(char *fname, bool useCSparse = false); // save precision matrix in CSPARSE format

      /// set up linear system, from RSS submission (konolige 2010)
      /// <sLambda> is the diagonal augmentation for the LM step
      void setupSys(double sLambda);
      void setupSparseSys(double sLambda, int iter, int sparseType);

      /// do LM solution for system; returns number of iterations on
      /// finish.  Argument is max number of iterations to perform,
      /// initial diagonal augmentation, and sparse form of Cholesky.
      /// useCSParse = 0 for dense Cholesky, 1 for sparse Cholesky, 2 for BPCG
      double lambda;
      int doSPA(int niter, double sLambda = 1.0e-4, int useCSparse = SBA_SPARSE_CHOLESKY, double initTol = 1.0e-8, int CGiters = 50);
      int doSPAwindowed(int window, int niter, double sLambda, int useCSparse);


#ifdef SBA_DSIF
      /// Delayed Sparse Information Filter for comparison
      void doDSIF(int newnode);
      void setupSparseDSIF(int newnode);
#endif

      /// Convergence bound (square of minimum acceptable delta change)
      double sqMinDelta;

      /// linear system matrix and vector
      Eigen::MatrixXd A;
      Eigen::VectorXd B;

      /// sparse matrix object
      CSparse2d csp;

      /// use CHOLMOD or CSparse
      void useCholmod(bool yes)
      { csp.useCholmod = yes; }

      /// if we want statistics
      bool verbose;
      bool print_iros_stats;

      /// return the graph of constraints
      /// x,y -> x',y'   4 floats per connection
      void getGraph(std::vector<float> &graph);
    };




  /// constraint files

  /// reads in a file of pose constraints
  bool read2dP2File(char *fname, SysSPA2d spa);


} // ends namespace sba

#endif  // _SPA2d_H_
