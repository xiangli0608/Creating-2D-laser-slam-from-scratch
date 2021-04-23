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
// Sparse Pose Adjustment classes and functions, 2D version
//

#include <stdio.h>
#include "sparse_bundle_adjustment/spa2d.h"
#include <Eigen/Cholesky>
#include <chrono>

using namespace Eigen;
using namespace std;

#include <iostream>
#include <iomanip>
#include <fstream>

// elapsed time in microseconds
static long long utime()
{
  auto duration = std::chrono::system_clock::now().time_since_epoch();
  return std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
}

namespace sba
{

  // R = [[c -s][s c]]
  // [R' | R't]
  void Node2d::setTransform()
    { 
      w2n(0,0) = w2n(1,1) = cos(arot);
      w2n(0,1) = sin(arot);
      w2n(1,0) = -w2n(0,1);
      w2n.col(2).setZero();
      w2n.col(2) = -w2n*trans;
    }


  //
  // sets angle derivatives dR'/dth
  // 
  void Node2d::setDr()
  {
    dRdx(0,0) = dRdx(1,1) = w2n(1,0); // -sin(th)
    dRdx(0,1) = w2n(0,0);       // cos(th)
    dRdx(1,0) = -w2n(0,0);      // -cos(th)
  }


  // set up Jacobians

  void Con2dP2::setJacobians(std::vector<Node2d,Eigen::aligned_allocator<Node2d> > &nodes)
  {
    // node references
    Node2d &nr = nodes[ndr];
    Matrix<double,3,1> &tr = nr.trans;
    Node2d &n1 = nodes[nd1];
    Matrix<double,3,1> &t1 = n1.trans;

    // first get the second frame in first frame coords
    Eigen::Matrix<double,2,1> pc = nr.w2n * t1;

    // Jacobians wrt first frame parameters

    // translational part of 0p1 wrt translational vars of p0
    // this is just -R0'  [from 0t1 = R0'(t1 - t0)]
    J0.block<2,2>(0,0) = -nr.w2n.block<2,2>(0,0);


    // translational part of 0p1 wrt rotational vars of p0
    // dR'/dq * [pw - t]
    Eigen::Matrix<double,2,1> pwt;
    pwt = (t1-tr).head(2);   // transform translations

    // dx
    Eigen::Matrix<double,2,1> dp = nr.dRdx * pwt; // dR'/dq * [pw - t]
    J0.block<2,1>(0,2) = dp;

    // rotational part of 0p1 wrt translation vars of p0 => zero
    J0.block<1,2>(2,0).setZero();

    // rotational part of 0p1 wrt rotational vars of p0
    // from 0q1 = (q1-q0)
    J0(2,2) = -1.0;
    J0t = J0.transpose();

    //  cout << endl << "J0 " << ndr << endl << J0 << endl;

    // Jacobians wrt second frame parameters
    // translational part of 0p1 wrt translational vars of p1
    // this is just R0'  [from 0t1 = R0'(t1 - t0)]
    J1.block<2,2>(0,0) = nr.w2n.block<2,2>(0,0);

    // translational part of 0p1 wrt rotational vars of p1: zero
    J1.block<2,1>(0,2).setZero();

    // rotational part of 0p1 wrt translation vars of p0 => zero
    J1.block<1,2>(2,0).setZero();


    // rotational part of 0p1 wrt rotational vars of p0
    // from 0q1 = (q1-q0)
    J1(2,2) = 1.0;
    J1t = J1.transpose();

    //  cout << endl << "J1 " << nd1 << endl << J1 << endl;

  };



  // error function
  // NOTE: this is h(x) - z, not z - h(x)
  inline double Con2dP2::calcErr(const Node2d &nd0, const Node2d &nd1)
    { 
      err.block<2,1>(0,0) = nd0.w2n * nd1.trans - tmean;
      double aerr = (nd1.arot - nd0.arot) - amean;
      if (aerr > M_PI) aerr -= 2.0*M_PI;
      if (aerr < -M_PI) aerr += 2.0*M_PI;
      err(2) = aerr;

      //      cout << err.transpose() << endl;

      return err.dot(prec * err);
    }


  // error function for distance cost
  double Con2dP2::calcErrDist(const Node2d &nd0, const Node2d &nd1)
    { 
      Vector2d derr = nd0.w2n * nd1.trans - tmean;
      return derr.dot(derr);
    }


  // error measure, squared
  // assumes node transforms have already been calculated
  // <tcost> is true if we just want the distance offsets
  double SysSPA2d::calcCost(bool tcost)
  {
    double cost = 0.0;
    
    // do distance offset
    if (tcost)
      {
        for(size_t i=0; i<p2cons.size(); i++)
          {
            Con2dP2 &con = p2cons[i];
            double err = con.calcErrDist(nodes[con.ndr],nodes[con.nd1]);
            cost += err;
          }
      }

    // full cost
    else 
      {
        for(size_t i=0; i<p2cons.size(); i++)
          {
            Con2dP2 &con = p2cons[i];
            double err = con.calcErr(nodes[con.ndr],nodes[con.nd1]);
            cost += err;
          }
        errcost = cost;
      }

    return cost;
  }


  // add a node at a pose
  // <pos> is x,y,th, with th in radians
  // returns node position 
  int SysSPA2d::addNode(const Vector3d &pos, int id)
  {
    Node2d nd;
    nd.nodeId = id;

    nd.arot = pos(2);
    nd.trans.head(2) = pos.head(2);
    nd.trans(2) = 1.0;

    // add in to system
    nd.setTransform();          // set up world2node transform
    nd.setDr();
    int ndi = nodes.size();
    nodes.push_back(nd);
    return ndi;
  }

  // add a constraint
  // <nd0>, <nd1> are node id's
  // <mean> is x,y,th, with th in radians
  // <prec> is a 3x3 precision matrix (inverse covariance
  // returns true if nodes are found
  // TODO: make node lookup more efficient
  bool SysSPA2d::addConstraint(int ndi0, int ndi1, const Vector3d &mean, 
                                 const Matrix3d &prec)
  {
    int ni0 = -1, ni1 = -1;
    for (int i=0; i<(int)nodes.size(); i++)
      {
        if (nodes[i].nodeId == ndi0)
          ni0 = i;
        if (nodes[i].nodeId == ndi1)
          ni1 = i;
      }
    if (ni0 < 0 || ni1 < 0) return false;
    
    Con2dP2 con;
    con.ndr = ni0;
    con.nd1 = ni1;

    con.tmean = mean.head(2);
    con.amean = mean(2);
    con.prec = prec;
    p2cons.push_back(con);
    return true;
  }


  // Set up linear system
  // Use dense matrices

  void SysSPA2d::setupSys(double sLambda)
  {
    // set matrix sizes and clear
    // assumes scales vars are all free
    int nFree = nodes.size() - nFixed;
    A.setZero(3*nFree,3*nFree);
    B.setZero(3*nFree);
    VectorXi dcnt(nFree);
    dcnt.setZero(nFree);

    // lambda augmentation
    double lam = 1.0 + sLambda;

    // loop over P2 constraints
    for(size_t pi=0; pi<p2cons.size(); pi++)
      {
        Con2dP2 &con = p2cons[pi];
        con.setJacobians(nodes);

        // add in 4 blocks of A
        int i0 = 3*(con.ndr-nFixed); // will be negative if fixed
        int i1 = 3*(con.nd1-nFixed); // will be negative if fixed
        
        if (i0>=0)
          {
            A.block<3,3>(i0,i0) += con.J0t * con.prec * con.J0;
            dcnt(con.ndr - nFixed)++;
          }
        if (i1>=0)
          {
            dcnt(con.nd1 - nFixed)++;
            Matrix<double,3,3> tp = con.prec * con.J1;
            A.block<3,3>(i1,i1) += con.J1t * tp;
            if (i0>=0)
              {
                A.block<3,3>(i0,i1) += con.J0t * con.prec * con.J1;
                A.block<3,3>(i1,i0) += con.J1t * con.prec * con.J0;
              }
          }

        // add in 2 blocks of B
        if (i0>=0)
          B.block<3,1>(i0,0) -= con.J0t * con.prec * con.err;
        if (i1>=0)
          B.block<3,1>(i1,0) -= con.J1t * con.prec * con.err;
      } // finish P2 constraints


    // augment diagonal
    A.diagonal() *= lam;

    // check the matrix and vector
    for (int i=0; i<3*nFree; i++)
      for (int j=0; j<3*nFree; j++)
        if (std::isnan(A(i,j)) ) { printf("[SetupSys] NaN in A\n"); *(int *)0x0 = 0; }

    for (int j=0; j<3*nFree; j++)
      if (std::isnan(B[j]) ) { printf("[SetupSys] NaN in B\n"); *(int *)0x0 = 0; }

    int ndc = 0;
    for (int i=0; i<nFree; i++)
      if (dcnt(i) == 0) ndc++;

    if (ndc > 0)
      cout << "[SetupSys] " << ndc << " disconnected nodes" << endl;
  }


  // Set up sparse linear system; see setupSys for algorithm.
  // Currently doesn't work with scale variables
  void SysSPA2d::setupSparseSys(double sLambda, int iter, int sparseType)
  {
    // set matrix sizes and clear
    // assumes scales vars are all free
    int nFree = nodes.size() - nFixed;

    long long t0, t1, t2, t3;
    t0 = utime();

    if (iter == 0)
      csp.setupBlockStructure(nFree); // initialize CSparse structures
    else
      csp.setupBlockStructure(0); // zero out CSparse structures

    t1 = utime();

    VectorXi dcnt(nFree);
    dcnt.setZero(nFree);

    // lambda augmentation
    double lam = 1.0 + sLambda;

    // loop over P2 constraints
    for(size_t pi=0; pi<p2cons.size(); pi++)
      {
        Con2dP2 &con = p2cons[pi];
        con.setJacobians(nodes);

        // add in 4 blocks of A; actually just need upper triangular
        int i0 = con.ndr-nFixed; // will be negative if fixed
        int i1 = con.nd1-nFixed; // will be negative if fixed
        
        if (i0>=0)
          {
           Matrix<double,3,3> m = con.J0t*con.prec*con.J0;
            csp.addDiagBlock(m,i0);
            dcnt(con.ndr - nFixed)++;
          }
        if (i1>=0)
          {
            dcnt(con.nd1 - nFixed)++;
            Matrix<double,3,3> tp = con.prec * con.J1;
            Matrix<double,3,3> m = con.J1t * tp;
            csp.addDiagBlock(m,i1);
            if (i0>=0)
              {
                Matrix<double,3,3> m2 = con.J0t * tp;
                if (i1 < i0)
                  {
                    m = m2.transpose();
                    csp.addOffdiagBlock(m,i1,i0);
                  }
                else
                  {
                    csp.addOffdiagBlock(m2,i0,i1);
                  }
              }
          }

        // add in 2 blocks of B
        if (i0>=0)
          csp.B.block<3,1>(i0*3,0) -= con.J0t * con.prec * con.err;
        if (i1>=0)
          csp.B.block<3,1>(i1*3,0) -= con.J1t * con.prec * con.err;
      } // finish P2 constraints

    t2 = utime();

    // set up sparse matrix structure from blocks
    if (sparseType == SBA_BLOCK_JACOBIAN_PCG)
      csp.incDiagBlocks(lam);   // increment diagonal block
    else
      csp.setupCSstructure(lam,iter==0); 
    t3 = utime();

    if (verbose)
      printf("\n[SetupSparseSys] Block: %0.1f   Cons: %0.1f  CS: %0.1f\n",
           (t1-t0)*.001, (t2-t1)*.001, (t3-t2)*.001);

    int ndc = 0;
    for (int i=0; i<nFree; i++)
      if (dcnt(i) == 0) ndc++;

    if (ndc > 0)
      cout << "[SetupSparseSys] " << ndc << " disconnected nodes" << endl;
  }
  

  /// Run the LM algorithm that computes a nonlinear SPA estimate.
  /// <niter> is the max number of iterations to perform; returns the
  /// number actually performed.
  /// <lambda> is the diagonal augmentation for LM.  
  /// <useCSparse> = 0 for dense Cholesky, 1 for sparse system, 
  ///                2 for gradient system, 3 for block jacobian PCG
  /// <initTol> is the initial tolerance for CG 
  /// <maxCGiters> is max # of iterations in BPCG

  int SysSPA2d::doSPA(int niter, double sLambda, int useCSparse, double initTol,
                      int maxCGiters)
  {
    // number of nodes
    int ncams = nodes.size();

    // set number of constraints
    int ncons = p2cons.size();

    // check for fixed frames
    if (nFixed <= 0)
      {
        cout << "[doSPA2d] No fixed frames" << endl;
        return 0;
      }
    for (int i=0; i<ncams; i++)
      {
        Node2d &nd = nodes[i];
        if (i >= nFixed)
          nd.isFixed = false;
        else 
          nd.isFixed = true;
        nd.setTransform();      // set up world-to-node transform for cost calculation
        nd.setDr();         // always use local angles
      }

    // initialize vars
    if (sLambda > 0.0)          // do we initialize lambda?
      lambda = sLambda;

    double laminc = 2.0;        // how much to increment lambda if we fail
    double lamdec = 0.5;        // how much to decrement lambda if we succeed
    int iter = 0;               // iterations
    sqMinDelta = 1e-8 * 1e-8;
    double cost = calcCost();
    if (verbose)
      cout << iter << " Initial squared cost: " << cost << " which is " 
           << sqrt(cost/ncons) << " rms error" << endl;

    int good_iter = 0;
    double cumTime = 0;
    for (; iter<niter; iter++)  // loop at most <niter> times
      {
        // set up and solve linear system
        // NOTE: shouldn't need to redo all calcs in setupSys if we 
        //   got here from a bad update

        long long t0, t1, t2, t3;
        t0 = utime();
        if (useCSparse)
          setupSparseSys(lambda,iter,useCSparse); // set up sparse linear system
        else
          setupSys(lambda);     // set up linear system

        //        cout << "[SPA] Solving...";
        t1 = utime();

        // use appropriate linear solver
        if (useCSparse == SBA_BLOCK_JACOBIAN_PCG)
          {
            if (csp.B.rows() != 0)
              {
                int iters = csp.doBPCG(maxCGiters,initTol,iter);
                if (verbose)
                  cout << "[Block PCG] " << iters << " iterations" << endl;
              }
          }
#ifdef SBA_DSIF
        // PCG with incomplete Cholesky
        else if (useCSparse == 3)
          {
            int res = csp.doPCG(maxCGiters);
            if (res > 1)
              cout << "[DoSPA] Sparse PCG failed with error " << res << endl;
          }
#endif
        else if (useCSparse > 0)
          {
            if (csp.B.rows() != 0)
              {
                bool ok = csp.doChol();
                if (!ok)
                  cout << "[DoSBA] Sparse Cholesky failed!" << endl;
              }
          }

        // Dense direct Cholesky 
        else
          A.ldlt().solveInPlace(B); // Cholesky decomposition and solution

        t2 = utime();
        //        cout << "solved" << endl;

        // get correct result vector
        VectorXd &BB = useCSparse ? csp.B : B;

        // check for convergence
        // this is a pretty crummy convergence measure...
        double sqDiff = BB.squaredNorm();
        if (sqDiff < sqMinDelta) // converged, done...
          {
            if (verbose)
              cout << "Converged with delta: " << sqrt(sqDiff) << endl;
            break;
          }

        // update the frames
        int ci = 0;
        for(int i=0; i < ncams; i++)
          {
            Node2d &nd = nodes[i];
            if (nd.isFixed) continue; // not to be updated
            nd.oldtrans = nd.trans; // save in case we don't improve the cost
            nd.oldarot = nd.arot;
            nd.trans.head<2>() += BB.segment<2>(ci);

            nd.arot += BB(ci+2); 
            nd.normArot();
            nd.setTransform();  // set up projection matrix for cost calculation
            nd.setDr();         // set rotational derivatives
            ci += 3;            // advance B index
          }

        // new cost
        double newcost = calcCost();
        if (verbose)
          cout << iter << " Updated squared cost: " << newcost << " which is " 
               << sqrt(newcost/ncons) << " rms error" << endl;
        
        // check if we did good
        if (newcost < cost) // && iter != 0) // NOTE: iter==0 case is for checking
          {
            cost = newcost;
            lambda *= lamdec;   // decrease lambda
            //      laminc = 2.0;       // reset bad lambda factor; not sure if this is a good idea...
            good_iter++;
          }
        else
          {
            lambda *= laminc;   // increase lambda
            laminc *= 2.0;      // increase the increment

            // reset nodes
            for(int i=0; i<ncams; i++)
              {
                Node2d &nd = nodes[i];
                if (nd.isFixed) continue; // not to be updated
                nd.trans = nd.oldtrans;
                nd.arot = nd.oldarot;
                nd.setTransform(); // set up projection matrix for cost calculation
                nd.setDr();
              }

            cost = calcCost();  // need to reset errors
            if (verbose)
              cout << iter << " Downdated cost: " << cost << endl;
            // NOTE: shouldn't need to redo all calcs in setupSys
          }

        t3 = utime();
        if (iter == 0 && verbose)
          {
            printf("[SPA] Setup: %0.2f ms  Solve: %0.2f ms  Update: %0.2f ms\n",
                   0.001*(double)(t1-t0),
                   0.001*(double)(t2-t1),
                   0.001*(double)(t3-t2));
          }

        double dt=1e-6*(double)(t3-t0);
        cumTime+=dt;
        if (print_iros_stats){
          cerr << "iteration= " << iter
               << "\t chi2= " << cost
               << "\t time= " << dt
               << "\t cumTime= " << cumTime
               << "\t kurtChi2= " << cost
               << endl;
        }

      }

    // return number of iterations performed
    return good_iter;

  }


  /// Run the LM algorithm that computes a nonlinear SPA estimate.
  /// <window> is the number of nodes in the window, the last nodes added
  /// <niter> is the max number of iterations to perform; returns the
  ///    number actually performed.
  /// <lambda> is the diagonal augmentation for LM.  
  /// <useCSParse> = 0 for dense Cholesky, 1 for sparse Cholesky, 2 for sparse PCG

  static inline int getind(std::map<int,int> &m, int ind)
  {
    std::map<int,int>::iterator it;
    it = m.find(ind);
    if (it == m.end())
      return -1;
    else
      return it->second;
  }

  int SysSPA2d::doSPAwindowed(int window, int niter, double sLambda, int useCSparse)
  {
    // number of nodes
    int nnodes = nodes.size();
    if (nnodes < 2) return 0;

    int nlow = nnodes - window;
    if (nlow < 1) nlow = 1;     // always have one fixed node

    if (verbose)
      cout << "[SPA Window] From " << nlow << " to " << nnodes << endl;

    // number of constraints
    int ncons = p2cons.size();

    // set up SPA
    SysSPA2d spa;
    spa.verbose = verbose;

    // node, constraint vectors and index mapping
    std::vector<Node2d,Eigen::aligned_allocator<Node2d> > &wnodes = spa.nodes;
    std::vector<Con2dP2,Eigen::aligned_allocator<Con2dP2> > &wp2cons = spa.p2cons;
    std::map<int,int> inds;
    std::vector<int> rinds;     // reverse indices

    // loop through all constraints and set up fixed nodes and constraints
    for (int i=0; i<ncons; i++)
      {
        Con2dP2 &con = p2cons[i];
        if (con.ndr >= nlow || con.nd1 >= nlow)
            wp2cons.push_back(con);

        if (con.ndr >= nlow && con.nd1 < nlow) // have a winner
          {
            int j = getind(inds,con.nd1); // corresponding index
            if (j < 0)      // not present, add it in
              {
                inds.insert(std::pair<int,int>(con.nd1,wnodes.size()));
                wnodes.push_back(nodes[con.nd1]);
              }
            rinds.push_back(con.nd1);
          }
        else if (con.nd1 >= nlow && con.ndr < nlow)
          {
            int j = getind(inds,con.ndr); // corresponding index
            if (j < 0)      // not present, add it in
              {
                inds.insert(std::pair<int,int>(con.ndr,wnodes.size()));
                wnodes.push_back(nodes[con.ndr]);
              }
            rinds.push_back(con.ndr);
          }
      }

    spa.nFixed = wnodes.size();
    if (verbose)
      cout << "[SPA Window] Fixed node count: " << spa.nFixed << endl;

    // add in variable nodes
    for (int i=0; i<(int)wp2cons.size(); i++)
      {
        Con2dP2 &con = wp2cons[i];
        if (con.nd1 >= nlow && con.ndr >= nlow) // have a winner
          {
            int n0 = getind(inds,con.ndr);
            if (n0 < 0)
              {
                inds.insert(std::pair<int,int>(con.ndr,wnodes.size()));
                wnodes.push_back(nodes[con.ndr]);
                rinds.push_back(con.ndr);
              }
            int n1 = getind(inds,con.nd1);
            if (n1 < 0)
              {
                inds.insert(std::pair<int,int>(con.nd1,wnodes.size()));
                wnodes.push_back(nodes[con.nd1]);
                rinds.push_back(con.nd1);
              }
          }
      }

    if (verbose)
      {
        cout << "[SPA Window] Variable node count: " << spa.nodes.size() - spa.nFixed << endl;
        cout << "[SPA Window] Constraint count: " << spa.p2cons.size() << endl;
      }

    // new constraint indices
    for (int i=0; i<(int)wp2cons.size(); i++)
      {
        Con2dP2 &con = wp2cons[i];
        con.ndr = getind(inds,con.ndr);
        con.nd1 = getind(inds,con.nd1);
      }

    // run spa
    niter = spa.doSPA(niter,sLambda,useCSparse);
    
    // reset constraint indices
    for (int i=0; i<(int)wp2cons.size(); i++)
      {
        Con2dP2 &con = wp2cons[i];
        con.ndr = rinds[con.ndr];
        con.nd1 = rinds[con.nd1];
      }
    return niter;
  }


#ifdef SBA_DSIF
  ///
  /// Delayed Sparse Info Filter (DSIF)
  ///

  // Set up sparse linear system for Delayed Sparse Information Filter
  void SysSPA2d::setupSparseDSIF(int newnode)
  {
    // set matrix sizes and clear
    // assumes scales vars are all free
    int nFree = nodes.size() - nFixed;

    //    long long t0, t1, t2, t3;
    //    t0 = utime();

    // don't erase old stuff here, the delayed filter just grows in size
    csp.setupBlockStructure(nFree,false); // initialize CSparse structures

    //    t1 = utime();

    // loop over P2 constraints
    for(size_t pi=0; pi<p2cons.size(); pi++)
      {
        Con2dP2 &con = p2cons[pi];

        // don't consider old constraints
        if (con.ndr < newnode && con.nd1 < newnode)
          continue;

        con.setJacobians(nodes);

        // add in 4 blocks of A; actually just need upper triangular
        int i0 = con.ndr-nFixed; // will be negative if fixed
        int i1 = con.nd1-nFixed; // will be negative if fixed
        
        // DSIF will not diverge on standard datasets unless
        //   we reduce the precision of the constraints
        double fact = 1.0;
        if (i0 != i1-1) fact = 0.99; // what should we use????

        if (i0>=0)
          {
            Matrix<double,3,3> m = con.J0t*con.prec*con.J0;
            csp.addDiagBlock(m,i0);
          }
        if (i1>=0)
          {
            Matrix<double,3,3> m = con.J1t*con.prec*con.J1;
            csp.addDiagBlock(m,i1);

            if (i0>=0)
              {
                Matrix<double,3,3> m2 = con.J0t*con.prec*con.J1;
                m2 = m2 * fact * fact;
                if (i1 < i0)
                  {
                    m = m2.transpose();
                    csp.addOffdiagBlock(m,i1,i0);
                  }
                else
                  {
                    csp.addOffdiagBlock(m2,i0,i1);
                  }
              }
          }

        // add in 2 blocks of B
        if (i0>=0)
          csp.B.block<3,1>(i0*3,0) -= con.J0t * con.prec * con.err;
        if (i1>=0)
          csp.B.block<3,1>(i1*3,0) -= con.J1t * con.prec * con.err;

      } // finish P2 constraints

    //    t2 = utime();

    csp.Bprev = csp.B;          // save for next iteration

    // set up sparse matrix structure from blocks
    csp.setupCSstructure(1.0,true); 

    //    t3 = utime();

    //    printf("\n[SetupSparseSys] Block: %0.1f   Cons: %0.1f  CS: %0.1f\n",
    //           (t1-t0)*.001, (t2-t1)*.001, (t3-t2)*.001);

  }


  /// Run the Delayed Sparse Information Filter (Eustice et al.)
  /// <newnode> is the index of the first new node added since the last iteration
  void SysSPA2d::doDSIF(int newnode)
  {
    // number of nodes
    int nnodes = nodes.size();

    // set number of constraints
    int ncons = p2cons.size();

    // check for fixed frames
    if (nFixed <= 0)
      {
        cout << "[doDSIF] No fixed frames" << endl;
        return;
      }

    // check for newnode being ok
    if (newnode >= nnodes)
      {
        cout << "[doDSIF] no new nodes to add" << endl;
        return;
      }
    else                        // set up saved mean value of pose
      {
        for (int i=newnode; i<nnodes; i++)
          {
            nodes[i].oldtrans = nodes[i].trans;
            nodes[i].oldarot = nodes[i].arot;
          }
      }

    for (int i=0; i<nnodes; i++)
      {
        Node2d &nd = nodes[i];
        if (i >= nFixed)
          nd.isFixed = false;
        else 
          nd.isFixed = true;
        nd.setTransform();      // set up world-to-node transform for cost calculation
        nd.setDr();             // always use local angles
      }

    // initialize vars
    double cost = calcCost();
    if (verbose)
      cout << " Initial squared cost: " << cost << " which is " 
           << sqrt(cost/ncons) << " rms error" << endl;

    // set up and solve linear system
    long long t0, t1, t2, t3;
    t0 = utime();
    setupSparseDSIF(newnode); // set up sparse linear system

#if 0
    cout << "[doDSIF] B = " << csp.B.transpose() << endl;
    csp.uncompress(A);
    cout << "[doDSIF] A = " << endl << A << endl;
#endif

    //        cout << "[SPA] Solving...";
    t1 = utime();
    bool ok = csp.doChol();
    if (!ok)
      cout << "[doDSIF] Sparse Cholesky failed!" << endl;
    t2 = utime();
    //        cout << "solved" << endl;

    // get correct result vector
    VectorXd &BB = csp.B;

    //    cout << "[doDSIF] RES  = " << BB.transpose() << endl;

    // update the frames
    int ci = 0;
    for(int i=0; i < nnodes; i++)
      {
        Node2d &nd = nodes[i];
        if (nd.isFixed) continue; // not to be updated
        nd.trans.head(2) = nd.oldtrans.head(2)+BB.segment<2>(ci);
        nd.arot = nd.oldarot+BB(ci+2); 
        nd.normArot();
        nd.setTransform();  // set up projection matrix for cost calculation
        nd.setDr();         // set rotational derivatives
        ci += 3;            // advance B index
      }

    // new cost
    double newcost = calcCost();
    if (verbose)
      cout << " Updated squared cost: " << newcost << " which is " 
           << sqrt(newcost/ncons) << " rms error" << endl;
        
    t3 = utime();
  }



  // write out the precision matrix for CSparse
  void SysSPA2d::writeSparseA(char *fname, bool useCSparse)
  {
    ofstream ofs(fname);
    if (ofs == NULL)
      {
        cout << "Can't open file " << fname << endl;
        return;
      }

    // cameras
    if (useCSparse)
      {
        setupSparseSys(0.0,0);
        
        int *Ai = csp.A->i;
        int *Ap = csp.A->p;
        double *Ax = csp.A->x;

        for (int i=0; i<csp.csize; i++)
          for (int j=Ap[i]; j<Ap[i+1]; j++)
            if (Ai[j] <= i)
              ofs << Ai[j] << " " << i << setprecision(16) << " " << Ax[j] << endl;
      }
    else
      {
        Eigen::IOFormat pfmt(16);

        int nrows = A.rows();
        int ncols = A.cols();
    
        for (int i=0; i<nrows; i++)
          for (int j=i; j<ncols; j++)
            {
              double a = A(i,j);
              if (A(i,j) != 0.0)
                ofs << i << " " << j << setprecision(16) << " " << a << endl;
            }
      }

    ofs.close();
  }
#endif


  // return vector of connections
  // 4 entries for each connection: x,y,x',y'
  void SysSPA2d::getGraph(std::vector<float> &graph)
  {
    for (int i=0; i<(int)p2cons.size(); i++)
      {
        Con2dP2 &con = p2cons[i];
        Node2d &nd0 = nodes[con.ndr];
        Node2d &nd1 = nodes[con.nd1];
        graph.push_back(nd0.trans(0));
        graph.push_back(nd0.trans(1));
        graph.push_back(nd1.trans(0));
        graph.push_back(nd1.trans(1));
      }
  }


}  // namespace vo



///*****************************************************************************
/// end of good code

/// here we keep stuff we may want in a little bit

#if 0

// this was a failed attempt at Ryan's augmentation of the info matrix


  // Set up sparse linear system for Delayed Sparse Information Filter
  void SysSPA2d::setupSparseDSIF(int newnode)
  {
    cout << "[SetupDSIF] at " << newnode << endl;

    // set matrix sizes and clear
    // assumes scales vars are all free
    int nFree = nodes.size() - nFixed;

    //    long long t0, t1, t2, t3;
    //    t0 = utime();

    // don't erase old stuff here, the delayed filter just grows in size
    csp.setupBlockStructure(nFree,false); // initialize CSparse structures

    //    t1 = utime();

    // loop over P2 constraints
    for(size_t pi=0; pi<p2cons.size(); pi++)
      {
        Con2dP2 &con = p2cons[pi];

        // don't consider old constraints
        if (con.ndr < newnode && con.nd1 < newnode)
          continue;

        con.setJacobians(nodes);
        Matrix3d J;
        J.setIdentity();
        Vector2d dRdth = nodes[con.ndr].dRdx.transpose() * con.tmean;
        J(0,2) = dRdth(0);
        J(1,2) = dRdth(1);
        Matrix3d Jt = J.transpose();
        Vector3d u;
        u.head(2) = nodes[con.ndr].trans.head(2);
        u(2) = nodes[con.ndr].arot;
        Vector3d f;
        f.head(2) = u.head(2) + nodes[con.ndr].w2n.transpose().block<2,2>(0,0) * con.tmean;
        f(2) = u(2) + con.amean;
        if (f(2) > M_PI) f(2) -= 2.0*M_PI;
        if (f(2) < M_PI) f(2) += 2.0*M_PI;

        
        cout << "[SetupDSIF] u  = " << u.transpose() << endl;
        cout << "[SetupDSIF] f  = " << f.transpose() << endl;
        cout << "[SetupDSIF] fo = " << nodes[con.nd1].trans.transpose() << endl;


        // add in 4 blocks of A; actually just need upper triangular
        int i0 = con.ndr-nFixed; // will be negative if fixed
        int i1 = con.nd1-nFixed; // will be negative if fixed
        
        if (i0 != i1-1) continue; // just sequential nodes for now

        if (i0>=0)
          {
            Matrix<double,3,3> m = Jt*con.prec*J;
            csp.addDiagBlock(m,i0);
          }
        if (i1>=0)
          {
            Matrix<double,3,3> m = con.prec;
            csp.addDiagBlock(m,i1);

            if (i0>=0)
              {
                Matrix<double,3,3> m2 = -con.prec * J;


                if (i1 < i0)
                  {
                    m = m2.transpose();
                    csp.addOffdiagBlock(m,i1,i0);
                  }
                else
                  {
                    csp.addOffdiagBlock(m2,i0,i1);
                  }
              }
          }

        // add in 2 blocks of B
        // Jt Gamma (J u - e)
        Vector3d Juf = J*u - f;
        if (Juf(2) > M_PI) Juf(2) -= 2.0*M_PI;
        if (Juf(2) < M_PI) Juf(2) += 2.0*M_PI;
        if (i0>=0)
          {
            csp.B.block<3,1>(i0*3,0) += Jt * con.prec * Juf;
          }
        if (i1>=0)
          {
            csp.B.block<3,1>(i1*3,0) -= con.prec * Juf;
          }

      } // finish P2 constraints

    //    t2 = utime();

    csp.Bprev = csp.B;          // save for next iteration

    // set up sparse matrix structure from blocks
    csp.setupCSstructure(1.0,true); 

    //    t3 = utime();

    //    printf("\n[SetupSparseSys] Block: %0.1f   Cons: %0.1f  CS: %0.1f\n",
    //           (t1-t0)*.001, (t2-t1)*.001, (t3-t2)*.001);

  }


  /// Run the Delayed Sparse Information Filter (Eustice et al.)
  /// <newnode> is the index of the first new node added since the last iteration
  /// <useCSparse> is true for sparse Cholesky.
  void SysSPA2d::doDSIF(int newnode, bool useCSparse)
  {
    // number of nodes
    int nnodes = nodes.size();

    // set number of constraints
    int ncons = p2cons.size();

    // check for fixed frames
    if (nFixed <= 0)
      {
        cout << "[doDSIF] No fixed frames" << endl;
        return;
      }

    // check for newnode being ok
    if (newnode >= nnodes)
      {
        cout << "[doDSIF] no new nodes to add" << endl;
        return;
      }

    nFixed = 0;

    for (int i=0; i<nnodes; i++)
      {
        Node2d &nd = nodes[i];
        if (i >= nFixed)
          nd.isFixed = false;
        else 
          nd.isFixed = true;
        nd.setTransform();      // set up world-to-node transform for cost calculation
        nd.setDr();             // always use local angles
      }

    // initialize vars
    double cost = calcCost();
    if (verbose)
      cout << " Initial squared cost: " << cost << " which is " 
           << sqrt(cost/ncons) << " rms error" << endl;

    if (newnode == 1)
      {
        // set up first system with node 0
        csp.setupBlockStructure(1,false); // initialize CSparse structures
        Matrix3d m;
        m.setIdentity();
        m = m*1000000;
        csp.addDiagBlock(m,0);
        Vector3d u;
        u.head(2) = nodes[0].trans.head(2);
        u(2) = nodes[0].arot;
        csp.B.head(3) = u*1000000;
        csp.Bprev = csp.B;              // save for next iteration
        cout << "[doDSIF] B = " << csp.B.transpose() << endl;    
      }

    // set up and solve linear system
    // NOTE: shouldn't need to redo all calcs in setupSys if we 
    //   got here from a bad update

    long long t0, t1, t2, t3;
    t0 = utime();
    if (useCSparse)
      setupSparseDSIF(newnode); // set up sparse linear system
    else
      {}

#if 1
    cout << "[doDSIF] B = " << csp.B.transpose() << endl;
    csp.uncompress(A);
    cout << "[doDSIF] A = " << endl << A << endl;
#endif

    //        cout << "[SPA] Solving...";
    t1 = utime();
    if (useCSparse)
      {
        bool ok = csp.doChol();
        if (!ok)
          cout << "[doDSIF] Sparse Cholesky failed!" << endl;
      }
    else
      A.ldlt().solveInPlace(B); // Cholesky decomposition and solution
    t2 = utime();
    //        cout << "solved" << endl;

    // get correct result vector
    VectorXd &BB = useCSparse ? csp.B : B;

    cout << "[doDSIF] RES  = " << BB.transpose() << endl;

    // update the frames
    int ci = 0;
    for(int i=0; i < nnodes; i++)
      {
        Node2d &nd = nodes[i];
        if (nd.isFixed) continue; // not to be updated
        nd.trans.head<2>() = BB.segment<2>(ci);
        nd.arot = BB(ci+2); 
        nd.normArot();
        nd.setTransform();  // set up projection matrix for cost calculation
        nd.setDr();         // set rotational derivatives
        ci += 3;            // advance B index
      }

    // new cost
    double newcost = calcCost();
    if (verbose)
      cout << " Updated squared cost: " << newcost << " which is " 
           << sqrt(newcost/ncons) << " rms error" << endl;
        
    t3 = utime();
  }


//// *********************************************
//// This version is following the measurement update rule in the SEIF paper.
//// Doesn't work

  // Set up sparse linear system for Delayed Sparse Information Filter
  void SysSPA2d::setupSparseDSIF(int newnode)
  {
    cout << "[SetupDSIF] at " << newnode << endl;

    // set matrix sizes and clear
    // assumes scales vars are all free
    int nFree = nodes.size() - nFixed;

    //    long long t0, t1, t2, t3;
    //    t0 = utime();

    // don't erase old stuff here, the delayed filter just grows in size
    csp.setupBlockStructure(nFree,false); // initialize CSparse structures

    //    t1 = utime();

    // loop over P2 constraints
    for(size_t pi=0; pi<p2cons.size(); pi++)
      {
        Con2dP2 &con = p2cons[pi];

        // don't consider old constraints
        if (con.ndr < newnode && con.nd1 < newnode)
          continue;

        con.setJacobians(nodes);

        // add in 4 blocks of A; actually just need upper triangular
        int i0 = con.ndr-nFixed; // will be negative if fixed
        int i1 = con.nd1-nFixed; // will be negative if fixed
        
        if (i0 != i1-1) continue; // just sequential nodes for now

        if (i0>=0)
          {
            Matrix<double,3,3> m = con.J0t*con.prec*con.J0;
            csp.addDiagBlock(m,i0);
          }
        if (i1>=0)
          {
            Matrix<double,3,3> m = con.J1t*con.prec*con.J1;
            csp.addDiagBlock(m,i1);

            if (i0>=0)
              {
                Matrix<double,3,3> m2 = con.J0t*con.prec*con.J1;
                if (i1 < i0)
                  {
                    m = m2.transpose();
                    csp.addOffdiagBlock(m,i1,i0);
                  }
                else
                  {
                    csp.addOffdiagBlock(m2,i0,i1);
                  }
              }
          }

        // add in 2 blocks of B
        // (J u + e)^T G J

        if (i0>=0)
          {
            Vector3d u;
            u.head(2) = nodes[con.ndr].trans.head(2);
            u(2) = nodes[con.ndr].arot;
            Vector3d bm = con.err + con.J0 * u;
            csp.B.block<3,1>(i0*3,0) += (bm.transpose() * con.prec * con.J0).transpose();
          }
        if (i1>=0)
          {
            Vector3d u;
            u.head(2) = nodes[con.nd1].trans.head(2);
            u(2) = nodes[con.nd1].arot;
            Vector3d bm = con.err + con.J1 * u;
            csp.B.block<3,1>(i1*3,0) += (bm.transpose() * con.prec * con.J1).transpose();
          }

      } // finish P2 constraints

    //    t2 = utime();

    csp.Bprev = csp.B;          // save for next iteration

    // set up sparse matrix structure from blocks
    csp.setupCSstructure(1.0,true); 

    //    t3 = utime();

    //    printf("\n[SetupSparseSys] Block: %0.1f   Cons: %0.1f  CS: %0.1f\n",
    //           (t1-t0)*.001, (t2-t1)*.001, (t3-t2)*.001);

  }


  /// Run the Delayed Sparse Information Filter (Eustice et al.)
  /// <newnode> is the index of the first new node added since the last iteration
  /// <useCSparse> is true for sparse Cholesky.
  void SysSPA2d::doDSIF(int newnode, bool useCSparse)
  {
    // number of nodes
    int nnodes = nodes.size();

    // set number of constraints
    int ncons = p2cons.size();

    // check for fixed frames
    if (nFixed <= 0)
      {
        cout << "[doDSIF] No fixed frames" << endl;
        return;
      }

    // check for newnode being ok
    if (newnode >= nnodes)
      {
        cout << "[doDSIF] no new nodes to add" << endl;
        return;
      }

    for (int i=0; i<nnodes; i++)
      {
        Node2d &nd = nodes[i];
        if (i >= nFixed)
          nd.isFixed = false;
        else 
          nd.isFixed = true;
        nd.setTransform();      // set up world-to-node transform for cost calculation
        nd.setDr();             // always use local angles
      }

    // initialize vars
    double cost = calcCost();
    if (verbose)
      cout << " Initial squared cost: " << cost << " which is " 
           << sqrt(cost/ncons) << " rms error" << endl;

    // set up and solve linear system
    // NOTE: shouldn't need to redo all calcs in setupSys if we 
    //   got here from a bad update

    long long t0, t1, t2, t3;
    t0 = utime();
    if (useCSparse)
      setupSparseDSIF(newnode); // set up sparse linear system
    else
      {}

#if 1
    cout << "[doDSIF] B = " << csp.B.transpose() << endl;
    csp.uncompress(A);
    cout << "[doDSIF] A = " << endl << A << endl;
#endif

    //        cout << "[SPA] Solving...";
    t1 = utime();
    if (useCSparse)
      {
        bool ok = csp.doChol();
        if (!ok)
          cout << "[doDSIF] Sparse Cholesky failed!" << endl;
      }
    else
      A.ldlt().solveInPlace(B); // Cholesky decomposition and solution
    t2 = utime();
    //        cout << "solved" << endl;

    // get correct result vector
    VectorXd &BB = useCSparse ? csp.B : B;

    cout << "[doDSIF] RES  = " << BB.transpose() << endl;

    // update the frames
    int ci = 0;
    for(int i=0; i < nnodes; i++)
      {
        Node2d &nd = nodes[i];
        if (nd.isFixed) continue; // not to be updated
        nd.trans.head<2>() = BB.segment<2>(ci);
        nd.arot = BB(ci+2); 
        nd.normArot();
        nd.setTransform();  // set up projection matrix for cost calculation
        nd.setDr();         // set rotational derivatives
        ci += 3;            // advance B index
      }

    // new cost
    double newcost = calcCost();
    if (verbose)
      cout << " Updated squared cost: " << newcost << " which is " 
           << sqrt(newcost/ncons) << " rms error" << endl;
        
    t3 = utime();

  }  // namespace sba


  /// remove node with id
  /// <id> is a node id
  void SysSPA2d::removeNode(int id)
  {
    int ind = -1;
    for (int i=0; i<(int)nodes.size(); i++)
      {
        if (nodes[i].nodeId == ind)
          ind = i;
      }
    if (ind < 0) return;

    // remove node
    nodes.erase(nodes.begin() + ind);

    // remove all constraints referring to node
    // and adjust indices of all nodes with indices
    // greater than 'ind'
    int i=0;
    while (i <(int)p2cons.size())
      {
        std::vector<Con2dP2,Eigen::aligned_allocator<Con2dP2> >::iterator iter = p2cons.begin() + i;
        if (iter->ndr == ind || iter->nd1 == ind)
          {
            p2cons.erase(iter);
          }
        else
          {
            if (iter->ndr > ind) iter->ndr--;
            if (iter->nd1 > ind) iter->nd1--;
            i++;
          }
      }
  }

  /// remove all constraints between ids
  // <nd0>, <nd1> are node id's
  bool SysSPA2d::removeConstraint(int ndi0, int ndi1)
  {
    int ni0 = -1, ni1 = -1;
    for (int i=0; i<(int)nodes.size(); i++)
      {
        if (nodes[i].nodeId == ndi0)
          ni0 = i;
        if (nodes[i].nodeId == ndi1)
          ni1 = i;
      }
    if (ni0 < 0 || ni1 < 0) return false;

    int i=0;
    while (i <(int)p2cons.size())
      {
        std::vector<Con2dP2,Eigen::aligned_allocator<Con2dP2> >::iterator iter = p2cons.begin() + i;
        if (iter->ndr == ni0 && iter->nd1 == ni1)
          {
            p2cons.erase(iter);
          }
        else
          {
            i++;
          }
      }

    return true;
  }



#endif


