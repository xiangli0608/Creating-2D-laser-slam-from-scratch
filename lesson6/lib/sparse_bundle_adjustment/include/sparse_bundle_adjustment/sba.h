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
// Sparse bundle/pose adjustment classes and functions
//

#ifndef _SBA_H_
#define _SBA_H_

#ifndef EIGEN_USE_NEW_STDVECTOR
#define EIGEN_USE_NEW_STDVECTOR
#endif // EIGEN_USE_NEW_STDVECTOR

#include <stdio.h>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/StdVector>
#include <vector>
#include <algorithm>
#include <Eigen/Cholesky>

#include <sparse_bundle_adjustment/node.h>
#include <sparse_bundle_adjustment/proj.h>


// sparse Cholesky
#include <sparse_bundle_adjustment/csparse.h>
// block jacobian pcg
#include <sparse_bundle_adjustment/bpcg/bpcg.h>

// Defines for methods to use with doSBA().
#define SBA_DENSE_CHOLESKY 0
#define SBA_SPARSE_CHOLESKY 1
#define SBA_GRADIENT 2
#define SBA_BLOCK_JACOBIAN_PCG 3

namespace sba
{
  /// SysSBA holds a set of nodes and points for sparse bundle adjustment

  class SysSBA
    {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW // needed for 16B alignment

      /// \brief How much information to print to console.
      int verbose;
      long long t0, t1, t2, t3, t4; // save timing

      /// \brief Default constructor.
        SysSBA() { nFixed = 1; useLocalAngles = true; Node::initDr(); 
          verbose = 1; huber = 0.0; }

      /// \brief Set of nodes (camera frames) for SBA system, indexed by node number.
      std::vector<Node, Eigen::aligned_allocator<Node> > nodes;

      /// \brief Number of fixed nodes (nodes with known poses) from the first node.
      int nFixed;               

      /// \brief Set of tracks for each point P (projections onto camera frames).
      std::vector<Track, Eigen::aligned_allocator<Track> > tracks;

      /// \brief Return total number of projections
      int countProjs();

      /// \brief Calculate the total cost of the system by adding the squared 
      /// error over all projections.
      /// \return Total error of the system, in pixels^2.
      double calcCost();
      
      /// \brief Calculates total cost of the system, not counting projections 
      /// with errors higher than <dist>.
      /// \param dist Distance, in pixels^2, above which projections are
      /// considered outliers.
      /// \return Total error of the system, in pixels^2.
      double calcCost(double dist);
      
      /// \brief Huber parameter; greater than 0.0 for Huber weighting of cost
      double huber;

      /// \brief Calculates total RMS cost of the system, not counting 
      /// projections with errors higher than <dist>.
      /// \param dist RMS distance, in pixels, above which projections are
      /// considered outliers.
      /// \return Total RMS error of the system, in pixels.
      double calcRMSCost(double dist = 10000.0);
      
      /// \brief Calculates average cost of the system.
      /// \returns Average error of the system over all projections, in pixels.
      double calcAvgError();
      
      /// \brief Find number of points with Z < 0 (projected behind the camera).
      int numBadPoints();

      /// \brief Find number of projections with errors over a certain value.
      int countBad(double dist);
      
      /// \brief Remove projections with too high of an error.
      /// \return Number of projections removed.
      int removeBad(double dist);

      /// \brief Reduce tracks by eliminating single tracks and invalid
      /// projections.
      int reduceTracks();

      /// \brief Print some system stats.
      void printStats();

      /// local or global angle coordinates
      bool useLocalAngles;

      /// set up linear system, from Engels and Nister 2006;
      /// <sLambda> is the diagonal augmentation for the LM step
      double lambda;            // save for continuation
      void setupSys(double sLambda);
      void setupSparseSys(double sLambda, int iter, int sparseType);

      /// do LM solution for system; returns number of iterations on
      /// finish.  Argument is max number of iterations to perform.
      /// <lambda> is the LM diagonal factor
      /// <useCSparse> is one of 
      ///   SBA_DENSE_CHOLESKY, SBA_SPARSE_CHOLESKY, SBA_GRADIENT, SBA_BLOCK_JACOBIAN_PCG 
      /// initTol is the initial tolerance for CG iterations
      int doSBA(int niter, double lambda = 1.0e-4, int useCSparse = 0, double initTol = 1.0e-8,
                  int maxCGiters = 100);

      /// Convergence bound (square of minimum acceptable delta change)
      double sqMinDelta;

      /// \brief Adds a node to the system.
      /// \param trans A 4x1 vector of translation of the camera.
      /// \param qrot A Quaternion containing the rotatin of the camera.
      /// \param cp Camera parameters containing the K parameters, including
      /// baseline and other camera parameters.
      /// \param isFixed Whether this camera is fixed in space or not, for sba.
      /// \return the index of the node added.
      int addNode(Eigen::Matrix<double,4,1> &trans, 
                      Eigen::Quaternion<double> &qrot,
                      const fc::CamParams &cp,
                      bool isFixed = false);

      /// \brief Adds a point to the system.
      /// \param p A point (4x1 Eigen Vector).
      /// \return the index of the point added.
      int addPoint(Point p);

      /// \brief Add a projection between point and camera, in setting up the 
      /// system.
      /// \param ci camera/node index (same as in nodes structure).
      /// \param pi point index (same as in tracks structure).
      /// \param q  the keypoint of the projection in image coordinates as u,v,u-d.
      /// \param stereo whether the point is stereo or not (true is stereo, 
      /// false is monocular).
      /// \return whether the projection was added (true) or not (false).
      /// Should only fail if the projection is a duplicate of an existing one
      /// with a different keypoint (i.e., same point projected to 2 locations
      /// in an image).
      bool addProj(int ci, int pi, Eigen::Vector3d &q, bool stereo=true);
      
      /// \brief Add a projection between point and camera, in setting up the 
      /// system.
      /// \param ci camera/node index (same as in nodes structure).
      /// \param pi point index (same as in tracks structure).
      /// \param q  the keypoint of the projection in image coordinates as u,v,u-d.
      /// \return whether the projection was added (true) or not (false).
      /// Should only fail if the projection is a duplicate of an existing one
      /// with a different keypoint (i.e., same point projected to 2 locations
      /// in an image).
      bool addMonoProj(int ci, int pi, Eigen::Vector2d &q);
      
      /// \brief Add a projection between point and camera, in setting up the 
      /// system.
      /// \param ci camera/node index (same as in nodes structure).
      /// \param pi point index (same as in tracks structure).
      /// \param q  the keypoint of the projection in image coordinates as u,v,u-d.
      /// \return whether the projection was added (true) or not (false).
      /// Should only fail if the projection is a duplicate of an existing one
      /// with a different keypoint (i.e., same point projected to 2 locations
      /// in an image).
      bool addStereoProj(int ci, int pi, Eigen::Vector3d &q);
      
      /// \brief Sets the covariance matrix of a projection.
      /// \param ci camera/node index (same as in nodes structure).
      /// \param pi point index (same as in tracks structure).
      /// \param covar 3x3 covariance matrix that affects the cost of the
      /// projection. Instead of the cost being ||err||, the cost is now
      /// (err)T*covar*(err).
      void setProjCovariance(int ci, int pi, Eigen::Matrix3d &covar);
      
      /// \brief Adds a pair of point-plane projection matches. 
      /// Assumes the points have already been added to the system with 
      /// point-to-point projections; this just takes care of the forward and 
      /// backward point-to-plane projections.
      /// \param ci0 Camera index of the first point in the match.
      /// \param pi0 Point index of the first point in the match.
      /// \param normal0 3D normal for the first point in camera0's coordinate frame.
      /// \param ci1 Camera index of the second point in the match.
      /// \param pi1 Point index of the second point in the match.
      /// \param normal1 3D normal for the second point in camera1's coordinate frame.
      void addPointPlaneMatch(int ci0, int pi0, Eigen::Vector3d normal0, int ci1, int pi1, Eigen::Vector3d normal1);
      
      /// \brief Update normals in point-plane matches, if any.
      void updateNormals();
      
      /// linear system matrix and vector
      Eigen::MatrixXd A;
      Eigen::VectorXd B;

      /// sparse connectivity matrix
      /// for each node, holds vector of connecting nodes
      /// "true" for don't use connection
      std::vector<std::vector<bool> > connMat;

      /// Sets up the connectivity matrix by clearing connections with 
      /// less than minpts.
      void setConnMat(int minpts);
      /// sets up connectivity matrix by greedy spanning tree
      void setConnMatReduced(int maxconns);
      /// removes tracks that aren't needed
      int remExcessTracks(int minpts);

      /// get rid of long tracks
      int reduceLongTracks(double pct);

      /// merge tracks based on identity pairs
      void mergeTracks(std::vector<std::pair<int,int> > &prs);
      /// merge two tracks if possible (no conflicts)
      int  mergeTracksSt(int tr0, int tr1);

      /// use CHOLMOD or CSparse
      void useCholmod(bool yes)
      { csp.useCholmod = yes; }

      /// sparse matrix object
      /// putting this at the end gets rid of alignment errors when making SBA objects      
      CSparse csp;
    
    // Private helper functions
    protected:
      /// Generate a connections map, used for setConnMat() and 
      /// setConnMatReduced().
      vector<map<int,int> > generateConns_();
      
      /// Split a track into random tracks. (What is len?)
      void tsplit(int tri, int len);
      
      /// Storage for old points, for checking LM step and reverting 
      std::vector<Point, Eigen::aligned_allocator<Point> > oldpoints;
      
      /// variables for each track
      std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > tps;

      /// storage for Jacobian products
        std::vector<JacobProds, Eigen::aligned_allocator<JacobProds> > jps;

    };


  /// CONP2 holds a constraint measurement of a pose to a pose.
  /// They are a repository for links between poses within a frame,
  /// with aux info such as jacobians

  class ConP2
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // needed for 16B alignment

    /// Reference pose index
    int ndr;

    /// Node index for the second node
    int nd1;

    /// Mean vector, quaternion (inverse) and precision matrix for this constraint
    Eigen::Vector3d tmean;
    Eigen::Quaternion<double> qpmean;
    Eigen::Matrix<double,6,6> prec;


    /// error
    Eigen::Matrix<double,6,1> err;
    /// calculates projection error and stores it in <err>
    inline double calcErr(const Node &nd0, const Node &nd1);

    /// calculate error in distance only, no weighting
    double calcErrDist(const Node &nd0, const Node &nd1);


    /// jacobian with respect to frames; uses dR'/dq from Node calculation
    Eigen::Matrix<double,6,6> J0,J0t,J1,J1t;

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
    void setJacobians(std::vector<Node,Eigen::aligned_allocator<Node> > &nodes);

    /// valid or not (could be out of bounds)
    bool isValid;
  };


  /// CONSCALE holds a constraint measurement on the scale of 
  /// a pose-pose constraint, in a scale group

  class ConScale
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // needed for 16B alignment

    /// Reference pose index
    int nd0;

    /// Node index for the second node
    int nd1;

    /// Scale variable index
    int sv;

    /// Scale factor for this constraint
    double ks;

    /// Weight for this constraint
    double w;

    /// error
    double err;
    /// calculates projection error and stores it in <err>
      inline double calcErr(const Node &nd0, const Node &nd1, double alpha);

    /// jacobian with respect to reference frame:
    ///   -2(t1 - t0)
    Eigen::Vector3d J0;

    /// jacobian with respect to second frame:
    ///    2(t1 - t0)
    Eigen::Vector3d J1;

    /// jacobians are computed from (ti - tj)^2 - a*kij = 0
    void setJacobians(std::vector<Node,Eigen::aligned_allocator<Node> > &nodes);

    /// valid or not (could be out of bounds)
    bool isValid;
  };


  /// CONP3P holds a constraint measurement between 3 poses.
  /// They are a repository for links between poses within a frame,
  /// with aux info such as jacobians

  class ConP3P
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // needed for 16B alignment

    /// Reference pose index
    int ndr;

    /// Node indices, the constraint for this object.  
    int nd1, nd2;

    /// Mean vector and precision matrix for this constraint
    Eigen::Matrix<double,12,1> mean;
    Eigen::Matrix<double,12,12> prec;

    /// error
    Eigen::Matrix<double,12,1> err;
    /// calculates projection error and stores it in <err>
    Eigen::Matrix<double,12,1> calcErr(const Node &nd, const Point &pt);

    /// Jacobians of 0p1,0p2 with respect to global p0, p1, p2
    Eigen::Matrix<double,6,6> J10, J11, J20, J22;

    /// dpc/dq = dR'/dq [pw-t], in homogeneous form, with q a quaternion param
    /// 

    /// dpc/dx = -R' * [1 0 0]', in homogeneous form, with x a translation param
    /// 

    /// d(px/pz)/du = [ pz dpx/du - px dpz/du ] / pz^2,
    /// works for all variables
    ///
    void setJacobians(std::vector<Node,Eigen::aligned_allocator<Node> > nodes);

    /// temp storage for Hpc, Tpc matrices in SBA
    Eigen::Matrix<double,3,6> Hpc;
    Eigen::Matrix<double,6,3> Tpc;

    /// valid or not (could be out of bounds)
    bool isValid;
  };



  /// SysSPA holds a set of nodes and constraints for sparse pose adjustment

  class SysSPA
    {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      /// constructor
        SysSPA() { nFixed = 1; useLocalAngles = true; Node::initDr(); lambda = 1.0e-4; 
                   verbose = false;}

      /// print info
      bool verbose;

      /// \brief Adds a node to the system.
      /// \param trans A 4x1 vector of translation of the camera.
      /// \param qrot A Quaternion containing the rotatin of the camera.
      /// \param isFixed Whether this camera is fixed in space or not, for spa.
      /// \return the index of the node added.
      int addNode(Eigen::Matrix<double,4,1> &trans, 
                      Eigen::Quaternion<double> &qrot,
                      bool isFixed = false);

      /// \brief Adds a pose constraint to the system.
      /// \param n1 Index of first node of the constraint
      /// \param n2 Index of second node of the constraint
      /// \param tmean A 3x1 vector, local translation from n1 to n2
      /// \param qpmean A Quaternion, local rotation from n1 to n2
      /// \param prec A 6x6 matrix, precision matrix for this link
      /// \return true if constraint added, false if nd0 or nd1 not found
      bool addConstraint(int nd0, int nd1,
                        Eigen::Vector3d &tmean,
                        Eigen::Quaterniond &qpmean,
                        Eigen::Matrix<double,6,6> &prec);

      /// set of nodes (camera frames) for SPA system, indexed by position;
      std::vector<Node,Eigen::aligned_allocator<Node> > nodes;

      /// set of scale for SPA system, indexed by position;
      std::vector<double> scales;

      /// Number of fixed nodes
      int nFixed;               

      /// Set of P2 constraints
      std::vector<ConP2,Eigen::aligned_allocator<ConP2> >  p2cons;

      /// Set of scale constraints
      std::vector<ConScale,Eigen::aligned_allocator<ConScale> >  scons;

      /// local or global angle coordinates
      bool useLocalAngles;

      /// calculate the error in the system;
      ///   if <tcost> is true, just the distance error without weighting
      double calcCost(bool tcost = false);
      /// calculate error assuming outliers > dist
      double calcCost(double dist);


      /// print some system stats
      void printStats();
      void writeSparseA(char *fname, bool useCSparse = false); // save precision matrix in CSPARSE format

      /// set up linear system, from RSS submission (konolige 2010)
      /// <sLambda> is the diagonal augmentation for the LM step
      double lambda;            // save for continuation
      void setupSys(double sLambda);
      void setupSparseSys(double sLambda, int iter, int sparseType);

      /// do LM solution for system; returns number of iterations on
      /// finish.  Argument is max number of iterations to perform,
      /// initial diagonal augmentation, and sparse form of Cholesky.
      int doSPA(int niter, double sLambda = 1.0e-4, int useCSparse = SBA_SPARSE_CHOLESKY,
                  double initTol = 1.0e-8, int CGiters = 50);

      /// Convergence bound (square of minimum acceptable delta change)
      double sqMinDelta;

      /// Spanning tree initialization
      void spanningTree(int node=0);

      /// Add a constraint between two poses, in a given pose frame.
      /// <pr> is reference pose, <p0> and <p1> are the pose constraints.
      void addConstraint(int pr, int p0, int p1, Eigen::Matrix<double,12,1> &mean, Eigen::Matrix<double,12,12> &prec);

      /// linear system matrix and vector
      Eigen::MatrixXd A;
      Eigen::VectorXd B;

      /// sparse matrix object
      CSparse csp;

      /// 6DOF pose as a unit quaternion and translation vector, saving
      /// for LM step
      Eigen::Matrix<double,4,1> oldtrans; // homogeneous coordinates, last element is 1.0
      Eigen::Matrix<double,4,1> oldqrot;  // this is the quaternion as coefficients, note xyzw order

    };

  /// constraint files

  /// reads in a file of pose constraints
  bool readP2File(char *fname, SysSPA spa);


} // ends namespace sba

#endif  // _SBA_H_

