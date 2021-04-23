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
// Sparse Pose Adjustment classes and functions
//

//
// Ok, we're going with 2-pose constraints, and added scale variables.
// Format for files:
// Format for files:
//   #nodes #scales #p2p_constraints #scale_constraints
//   x y z rx ry rz   ; first node, vector part of unit quaternion
//   ...
//   p1 p2 Lambda_12  ; first p2p constraint
//   ...
//   p1 p2 s1 K_12 w  ; first scale constraint, 
//                    ; (p1-p2)^2 = s1*K_12 with weight w


#include <stdio.h>
#include "sparse_bundle_adjustment/sba.h"
#include <Eigen/Cholesky>
#include <chrono>

using namespace Eigen;
using namespace std;

#include <iostream>
#include <iomanip>
#include <fstream>
#include <utility>

// elapsed time in microseconds
static long long utime()
{
  auto duration = std::chrono::system_clock::now().time_since_epoch();
  return std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
}

namespace sba
{
  // reads in a file of pose constraints
  bool readP2File(char *fname, SysSPA spa)
  {
    // Format for files:
    //   #nodes #scales #p2p_constraints #scale_constraints
    //   x y z rx ry rz   ; first node, vector part of unit quaternion
    //   ...
    //   p1 p2 Lambda_12  ; first p2p constraint
    //   ...
    //   p1 p2 s1 K_12 w  ; first scale constraint, 
    //                    ; (p1-p2)^2 = s1*K_12 with weight w

    ifstream ifs(fname);
    if (!ifs)
      {
        cout << "Can't open file " << fname << endl;
        return false;
      }
    ifs.precision(10);          // what is this for???

    // read header
    string line;
    if (!getline(ifs,line) || line != "# P2 Constraint File")
    {
      cout << "Bad header" << endl;
      return false;
    }
    cout << "Found P2 constraint file" << endl;

    // read number of cameras, scalers, and p2p constraints
    int ncams, nss, nscs, np2s;
    if (!(ifs >> ncams >> nss >> np2s >> nscs))
    {
      cout << "Bad entity count" << endl;  
      return false;
    }
    cout << "Number of cameras: " << ncams 
         << "  Number of scalers: " << nss
         << "  Number of p2p constraints: " << np2s
         << "  Number of scale constraints: " << nscs << endl;
    
    cout << "Reading in camera data..." << flush;
    std::vector<Node,Eigen::aligned_allocator<Node> > &nodes = spa.nodes;
    Node nd;

    for (int i=0; i<ncams; i++)
      {
        double v1,v2,v3;
        if (!(ifs >> v1 >> v2 >> v3))
          {
            cout << "Bad node translation params at number " << i << endl;
            return false;
          }
        nd.trans = Vector4d(v1,v2,v3,1.0);

        if (!(ifs >> v1 >> v2 >> v3))
          {
            cout << "Bad node rotation quaternion at number " << i << endl;
            return false;
          }
        Matrix3d m;
        nd.qrot = Quaternion<double>(v1,v2,v2,0.0);
        nd.normRot();           // normalize to unit vector and compute w
        
        nodes.push_back(nd);
      }
    cout << "done" << endl;

    // read in p2p constraints
    cout << "Reading in constraint data..." << flush;
    std::vector<ConP2,Eigen::aligned_allocator<ConP2> > &cons = spa.p2cons;
    ConP2 con;

    for (int i=0; i<np2s; i++)
      {
        int p1, p2;
        double vz[6];           // mean
        double vv[36];          // prec

        if (!(ifs >> p1 >> p2))
          {
            cout << "Bad node indices at constraint " << i << endl;
            return false;
          }

        for (int i=0; i<6; i++)
          {
            if (!(ifs >> vz[i]))
              {
                cout << "Bad constraint mean at constraint " << i << endl;
                return false;
              }
          }

        for (int i=0; i<36; i++)
          {
            if (!(ifs >> vv[i]))
              {
                cout << "Bad constraint precision at constraint " << i << endl;
                return false;
              }
          }

        con.ndr = p1;
        con.nd1 = p2;

        //        con.setMean(Matrix<double,6,1>(vz));
        con.prec = Matrix<double,6,6>(vv);

        cons.push_back(con);
      }


    // read in scale constraints
    cout << "Reading in scale constraint data..." << flush;
    std::vector<ConScale,Eigen::aligned_allocator<ConScale> > &scons = spa.scons;
    ConScale scon;

    for (int i=0; i<nscs; i++)
      {
        int p1, p2, sv;
        double ks, w;

        if (!(ifs >> p1 >> p2 >> sv >> ks >> w))
          {
            cout << "Bad scale constraint at constraint " << i << endl;
            return false;
          }
        
        scon.nd0 = p1;
        scon.nd1 = p2;
        scon.sv  = sv;
        scon.ks  = ks;
        scon.w   = w;

        scons.push_back(scon);
      }

    return true;
  }

// Do we want to normalize quaternion vectors based on the sign of w?
#define NORMALIZE_Q

  // set up Jacobians
  // see Konolige RSS 2010 submission for details

  void ConP2::setJacobians(std::vector<Node,Eigen::aligned_allocator<Node> > &nodes)
  {
    // node references
    Node &nr = nodes[ndr];
    Matrix<double,4,1> &tr = nr.trans;
    Quaternion<double> &qr = nr.qrot;
    Node &n1 = nodes[nd1];
    Matrix<double,4,1> &t1 = n1.trans;
    Quaternion<double> &q1 = n1.qrot;

    // first get the second frame in first frame coords
    Eigen::Matrix<double,3,1> pc = nr.w2n * t1;

    // Jacobians wrt first frame parameters

    // translational part of 0p1 wrt translational vars of p0
    // this is just -R0'  [from 0t1 = R0'(t1 - t0)]
    J0.block<3,3>(0,0) = -nr.w2n.block<3,3>(0,0);


    // translational part of 0p1 wrt rotational vars of p0
    // dR'/dq * [pw - t]
    Eigen::Matrix<double,3,1> pwt;
    pwt = (t1-tr).head(3);   // transform translations

    // dx
    Eigen::Matrix<double,3,1> dp = nr.dRdx * pwt; // dR'/dq * [pw - t]
    J0.block<3,1>(0,3) = dp;
    // dy
    dp = nr.dRdy * pwt; // dR'/dq * [pw - t]
    J0.block<3,1>(0,4) = dp;
    // dz
    dp = nr.dRdz * pwt; // dR'/dq * [pw - t]
    J0.block<3,1>(0,5) = dp;

    // rotational part of 0p1 wrt translation vars of p0 => zero
    J0.block<3,3>(3,0).setZero();

    // rotational part of 0p1 wrt rotational vars of p0
    // from 0q1 = qpmean * s0' * q0' * q1

    // dqdx
    Eigen::Quaternion<double> qr0, qr1, qrn, qrd;
    qr1.coeffs() = q1.coeffs();
    qrn.coeffs() = Vector4d(-qpmean.w(),-qpmean.z(),qpmean.y(),qpmean.x());  // qpmean * ds0'/dx
    qr0.coeffs() = Vector4d(-qr.x(),-qr.y(),-qr.z(),qr.w());
    qr0 = qr0*qr1;              // rotate to zero mean
    qrd = qpmean*qr0;           // for normalization check
    qrn = qrn*qr0;

#ifdef NORMALIZE_Q
    if (qrd.w() < 0.0)
      qrn.vec() = -qrn.vec();
#endif

    J0.block<3,1>(3,3) = qrn.vec();

    // dqdy
    qrn.coeffs() = Vector4d(qpmean.z(),-qpmean.w(),-qpmean.x(),qpmean.y());  // qpmean * ds0'/dy
    qrn = qrn*qr0;

#ifdef NORMALIZE_Q
    if (qrd.w() < 0.0)
      qrn.vec() = -qrn.vec();
#endif

    J0.block<3,1>(3,4) = qrn.vec();

    // dqdz
    qrn.coeffs() = Vector4d(-qpmean.y(),qpmean.x(),-qpmean.w(),qpmean.z());  // qpmean * ds0'/dz
    qrn = qrn*qr0;

#ifdef NORMALIZE_Q
    if (qrd.w() < 0.0)
      qrn.vec() = -qrn.vec();
#endif

    J0.block<3,1>(3,5) = qrn.vec();

    // transpose
    J0t = J0.transpose();

    //  cout << endl << "J0 " << ndr << endl << J0 << endl;

    // Jacobians wrt second frame parameters
    // translational part of 0p1 wrt translational vars of p1
    // this is just R0'  [from 0t1 = R0'(t1 - t0)]
    J1.block<3,3>(0,0) = nr.w2n.block<3,3>(0,0);

    // translational part of 0p1 wrt rotational vars of p1: zero
    J1.block<3,3>(0,3).setZero();

    // rotational part of 0p1 wrt translation vars of p0 => zero
    J1.block<3,3>(3,0).setZero();


    // rotational part of 0p1 wrt rotational vars of p0
    // from 0q1 = q0'*s1*q1

    Eigen::Quaternion<double> qrc;
    qrc.coeffs() = Vector4d(-qr.x(),-qr.y(),-qr.z(),qr.w());
    qrc = qpmean*qrc*qr1;       // mean' * qr0' * qr1
    qrc.normalize();

    //    cout << endl << "QRC  : " << qrc.coeffs().transpose() << endl;

    double dq = 1.0e-8;
    double wdq = 1.0 - dq*dq;
    qr1.coeffs() = Vector4d(dq,0,0,wdq);
    //    cout << "QRC+x: " << (qrc*qr1).coeffs().transpose() << endl;    
    //    cout << "QRdx:  " << ((qrc*qr1).coeffs().transpose() - qrc.coeffs().transpose())/dq << endl;

    // dqdx
    qrn.coeffs() = Vector4d(1,0,0,0);
    qrn = qrc*qrn;

    //    cout << "J1dx:  " << qrn.coeffs().transpose() << endl;

#ifdef NORMALIZE_Q
    if (qrc.w() < 0.0)
      qrn.vec() = -qrn.vec();
#endif

    J1.block<3,1>(3,3) = qrn.vec();

    // dqdy
    qrn.coeffs() = Vector4d(0,1,0,0);
    qrn = qrc*qrn;

#ifdef NORMALIZE_Q
    if (qrc.w() < 0.0)
      qrn.vec() = -qrn.vec();
#endif

    J1.block<3,1>(3,4) = qrn.vec();

    // dqdz
    qrn.coeffs() = Vector4d(0,0,1,0);
    qrn = qrc*qrn;

#ifdef NORMALIZE_Q
    if (qrc.w() < 0.0)
      qrn.vec() = -qrn.vec();
#endif

    J1.block<3,1>(3,5) = qrn.vec();

    // transpose
    J1t = J1.transpose();

    //  cout << endl << "J1 " << nd1 << endl << J1 << endl;

  };



  // set up Jacobians
  // see Konolige RSS 2010 submission for details

  void ConScale::setJacobians(std::vector<Node,Eigen::aligned_allocator<Node> > &nodes)
  {
    // node references
    Node &n0 = nodes[nd0];
    Matrix<double,4,1> &t0 = n0.trans;
    Node &n1 = nodes[nd1];
    Matrix<double,4,1> &t1 = n1.trans;

    Eigen::Matrix<double,3,1> td = (t1-t0).head(3);

    // Jacobians wrt first frame parameters
    //  (ti - tj)^2 - a*kij

    // scale error wrt translational vars t0
    J0 = -2.0 * td;

    // scale error wrt translational vars t1
    J1 =  2.0 * td;

    // scale error wrt scale variable is just -kij
  };



  // 
  // This hasn't been tested, and is probably wrong.
  //

  void ConP3P::setJacobians(std::vector<Node,Eigen::aligned_allocator<Node> > nodes)
  {
    // node references
    Node nr = nodes[ndr];
    Matrix<double,4,1> &tr = nr.trans;
    Quaternion<double> &qr = nr.qrot;
    Node n1 = nodes[nd1];
    Matrix<double,4,1> &t1 = n1.trans;
    Quaternion<double> &q1 = n1.qrot;
    Node n2 = nodes[nd2];
    Matrix<double,4,1> &t2 = n2.trans;
    Quaternion<double> &q2 = n2.qrot;

    // calculate J10 -> d(0p1)/d(p0)
    // rows are indexed by 0p1 parameters
    // cols are indexed by p0 parameters

    // translational part of 0p1 wrt translational vars of p0
    // this is just -R0'  [from 0t1 = R0'(t1 - t0)]
    J10.block<3,3>(0,0) = -nr.w2n.block<3,3>(0,0);

    // translational part of 0p1 wrt rotational vars of p0
    // dR'/dq * [pw - t]
    Matrix<double,3,1> pwt = (t1-tr).head(3);
    Matrix<double,3,1> dp = nr.dRdx * pwt; // dR'/dqx * [pw - t]
    J10.block<3,1>(0,3) = dp;
    dp = nr.dRdy * pwt; // dR'/dqy * [pw - t]
    J10.block<3,1>(0,4) = dp;
    dp = nr.dRdz * pwt; // dR'/dqz * [pw - t]
    J10.block<3,1>(0,5) = dp;

    // rotational part of 0p1 wrt translation vars of p0 => zero
    J10.block<3,3>(3,0).setZero();

    // rotational part of 0p1 wrt rotational vars of p0
    // from 0q1 = q0'*q1

    // dqdx
    double wn = -1.0/qr.w();  // need to switch params for small qr(3)
    dp[0] = wn*t1[0]*qr.x() - t1[3];
    dp[1] = wn*t1[1]*qr.x() + t1[2];
    dp[2] = wn*t1[2]*qr.x() - t1[1];
    J10.block<3,1>(3,3) = dp;

    // dqdy
    dp[0] = wn*t1[0]*qr.y() - t1[2];
    dp[1] = wn*t1[1]*qr.y() - t1[3];
    dp[2] = wn*t1[2]*qr.y() + t1[0];
    J10.block<3,1>(3,4) = dp;

    // dqdz
    dp[0] = wn*t1[0]*qr.z() + t1[1];
    dp[1] = wn*t1[1]*qr.z() - t1[0];
    dp[2] = wn*t1[2]*qr.z() - t1[3];
    J10.block<3,1>(3,5) = dp;

    // whew, J10 done...

    // J20 is similar, just using p2
    // translational part of 0p2 wrt translational vars of p0
    // this is just -R0'  [from 0t2 = R0'(t2 - t0)]
    J20.block<3,3>(0,0) = -nr.w2n.block<3,3>(0,0);

    // translational part of 0p2 wrt rotational vars of p0
    // dR'/dq * [pw - t]
    pwt = (t2-tr).head(3);
    dp = nr.dRdx * pwt; // dR'/dqx * [pw - t]
    J20.block<3,1>(0,3) = dp;
    dp = nr.dRdy * pwt; // dR'/dqy * [pw - t]
    J20.block<3,1>(0,4) = dp;
    dp = nr.dRdz * pwt; // dR'/dqz * [pw - t]
    J20.block<3,1>(0,5) = dp;

    // rotational part of 0p1 wrt translation vars of p0 => zero
    J20.block<3,3>(3,0).setZero();

    // rotational part of 0p2 wrt rotational vars of p0
    // from 0q1 = q0'*q2

    // dqdx
    wn = -1.0/qr.w();  // need to switch params for small qr(3)
    dp[0] = wn*q2.x()*qr.x() - q2.w();
    dp[1] = wn*q2.y()*qr.x() + q2.z();
    dp[2] = wn*q2.z()*qr.x() - q2.y();
    J20.block<3,1>(3,3) = dp;

    // dqdy
    dp[0] = wn*q2.x()*qr.y() - q2.z();
    dp[1] = wn*q2.y()*qr.y() - q2.w();
    dp[2] = wn*q2.z()*qr.y() + q2.x();
    J20.block<3,1>(3,4) = dp;

    // dqdz
    dp[0] = wn*q2.x()*qr.z() + q2.y();
    dp[1] = wn*q2.y()*qr.z() - q2.x();
    dp[2] = wn*q2.z()*qr.z() - q2.w();
    J20.block<3,1>(3,5) = dp;

    // calculate J11 -> d(0p1)/d(p1)
    // rows are indexed by 0p1 parameters
    // cols are indexed by p1 parameters

    // translational part of 0p1 wrt translational vars of p1
    // this is just R0'  [from 0t1 = R0'(t1 - t0)]
    J11.block<3,3>(0,0) = nr.w2n.block<3,3>(0,0);

    // translational part of 0p1 wrt rotational vars of p1 => zero
    J11.block<3,3>(0,3).setZero();

    // rotational part of 0p1 wrt translation vars of p1 => zero
    J11.block<3,3>(3,0).setZero();

    // rotational part of 0p1 wrt rotational vars of p1
    // from 0q1 = q0'*q1

    // dqdx1
    wn = 1.0/q1.w();  // need to switch params for small q1(3)
    dp[0] = wn*qr.x()*q1.x() + qr.w();
    dp[1] = wn*qr.y()*q1.x() - qr.z();
    dp[2] = wn*qr.z()*q1.x() + qr.y();
    J11.block<3,1>(3,3) = dp;

    // dqdy1
    dp[0] = wn*qr.x()*q1.y() + qr.z();
    dp[1] = wn*qr.y()*q1.y() + qr.w();
    dp[2] = wn*qr.z()*q1.y() - qr.x();
    J11.block<3,1>(3,4) = dp;

    // dqdz1
    dp[0] = wn*qr.x()*q1.z() - qr.y();
    dp[1] = wn*qr.y()*q1.z() + qr.x();
    dp[2] = wn*qr.z()*q1.z() + qr.w();
    J11.block<3,1>(3,5) = dp;

    // whew, J11 done...    

    // J22 is similar to J11
    // rows are indexed by 0p2 parameters
    // cols are indexed by p2 parameters

    // translational part of 0p2 wrt translational vars of p2
    // this is just R0'  [from 0t2 = R0'(t2 - t0)]
    J22.block<3,3>(0,0) = nr.w2n.block<3,3>(0,0);

    // translational part of 0p2 wrt rotational vars of p2 => zero
    J22.block<3,3>(0,3).setZero();

    // rotational part of 0p2 wrt translation vars of p2 => zero
    J22.block<3,3>(3,0).setZero();

    // rotational part of 0p2 wrt rotational vars of p2
    // from 0q2 = q0'*q2

    // dqdx2
    wn = 1.0/q2.w();  // need to switch params for small q2.w()
    dp[0] = wn*qr.x()*q2.x() + qr.w();
    dp[1] = wn*qr.y()*q2.x() - qr.z();
    dp[2] = wn*qr.z()*q2.x() + qr.y();
    J22.block<3,1>(3,3) = dp;

    // dqdy1
    dp[0] = wn*qr.x()*q2.y() + qr.z();
    dp[1] = wn*qr.y()*q2.y() + qr.w();
    dp[2] = wn*qr.z()*q2.y() - qr.x();
    J22.block<3,1>(3,4) = dp;

    // dqdz1
    dp[0] = wn*qr.x()*q2.z() - qr.y();
    dp[1] = wn*qr.y()*q2.z() + qr.x();
    dp[2] = wn*qr.z()*q2.z() + qr.w();
    J22.block<3,1>(3,5) = dp;

    // whew, J22 done...    

  };


  // error function
  inline double ConP2::calcErr(const Node &nd0, const Node &nd1)
    { 
      Quaternion<double> q0p,q1;
      q0p.vec()   = -nd0.qrot.coeffs().head(3); // invert quaternion
      q0p.w()     =  nd0.qrot.w();
      q1          =  nd1.qrot;
      err.block<3,1>(0,0) = nd0.w2n * nd1.trans - tmean;

      //      cout << endl;
      //      cout << q0p.coeffs().transpose() << endl;
      //      cout << q1.coeffs().transpose() << endl;
      //      cout << qpmean.coeffs().transpose() << endl;

      q1 = qpmean * q0p * q1;

      //      cout << q1.coeffs().transpose() << endl << endl;

// this seems to mess up convergence...
#ifdef NORMALIZE_Q                              
      if (q1.w() < 0.0)
        err.block<3,1>(3,0) = -q1.vec(); // normalized
      else
#endif
        err.block<3,1>(3,0) = q1.vec(); // do we have to normalize w???
      //      cout << endl << "Error: " << err.transpose() << endl << endl;
      return err.dot(prec * err);
    }


  // error function for distance cost
  double ConP2::calcErrDist(const Node &nd0, const Node &nd1)
    { 
      Vector3d derr;
      Quaternion<double> q0p,q1;
      q0p.vec()   = -nd0.qrot.vec(); // invert quaternion
      q0p.w()     =  nd0.qrot.w();
      q1          =  nd1.qrot;
      derr = nd0.w2n * nd1.trans - tmean;
      return derr.dot(derr);
    }


  // error function
  inline double ConScale::calcErr(const Node &nd0, const Node &nd1, double alpha)
    { 
      err = (nd1.trans - nd0.trans).squaredNorm();
      err -= ks*alpha;
      //      cout << "Scale err: " << err << endl;
      return w*err*err;
    }


  // Adds a node to the system. 
  // \return the index of the node added.
  int SysSPA::addNode(Eigen::Matrix<double,4,1> &trans, 
                      Eigen::Quaternion<double> &qrot,
                      bool isFixed)
  {
    Node nd;
    nd.trans = trans;
    nd.qrot = qrot;
    nd.isFixed = isFixed;
    nd.setTransform(); // set up world2node transform
    nd.setDr(true); // set rotational derivatives
    // Should this be local or global?
    nd.normRot();//Local();
    nodes.push_back(nd);
    return nodes.size()-1;
  }


  // add a constraint
  // <nd0>, <nd1> are node id's
  // <mean> is x,y,th, with th in radians
  // <prec> is a 3x3 precision matrix (inverse covariance
  // returns true if nodes are found
  bool SysSPA::addConstraint(int nd0, int nd1,
                             Eigen::Vector3d &tmean,
                             Eigen::Quaterniond &qpmean,
                             Eigen::Matrix<double,6,6> &prec)
  {
    if (nd0 >= (int)nodes.size() || nd1 >= (int)nodes.size()) 
      return false;
    
    ConP2 con;
    con.ndr = nd0;
    con.nd1 = nd1;

    con.tmean = tmean;
    Quaternion<double> qr;
    qr = qpmean;
    qr.normalize();
    con.qpmean = qr.inverse(); // inverse of the rotation measurement
    con.prec = prec;            

    p2cons.push_back(con);
    return true;
  }


  // error measure, squared
  // assumes node transforms have already been calculated
  // <tcost> is true if we just want the distance offsets
  double SysSPA::calcCost(bool tcost)
  {
    double cost = 0.0;
    
    // do distance offset
    if (tcost)
      {
        for(size_t i=0; i<p2cons.size(); i++)
          {
            ConP2 &con = p2cons[i];
            double err = con.calcErrDist(nodes[con.ndr],nodes[con.nd1]);
            cost += err;
          }
      }

    // full cost
    else 
      {
        for(size_t i=0; i<p2cons.size(); i++)
          {
            ConP2 &con = p2cons[i];
            double err = con.calcErr(nodes[con.ndr],nodes[con.nd1]);
            cost += err;
          }
        if (scons.size() > 0)       // can have zero scale constraints
          for(size_t i=0; i<scons.size(); i++)
            {
              ConScale &con = scons[i];
              double err = con.calcErr(nodes[con.nd0],nodes[con.nd1],scales[con.sv]);
              cost += err;
            }
      }

    return cost;
  }


  // Set up linear system, from RSS submission (konolige 2010)
  // This is a relatively compact version of the algorithm! 
  // but not Jacobians

  void SysSPA::setupSys(double sLambda)
  {
    // set matrix sizes and clear
    // assumes scales vars are all free
    int nFree = nodes.size() - nFixed;
    int nscales = scales.size();
    A.setZero(6*nFree+nscales,6*nFree+nscales);
    B.setZero(6*nFree+nscales);
    VectorXi dcnt(nFree);
    dcnt.setZero(nFree);

    // lambda augmentation
    double lam = 1.0 + sLambda;

    // loop over P2 constraints
    for(size_t pi=0; pi<p2cons.size(); pi++)
      {
        ConP2 &con = p2cons[pi];
        con.setJacobians(nodes);

        // add in 4 blocks of A; actually just need upper triangular
        // i0 < i1
        int i0 = 6*(con.ndr-nFixed); // will be negative if fixed
        int i1 = 6*(con.nd1-nFixed); // will be negative if fixed
        
        if (i0>=0)
          {
            A.block<6,6>(i0,i0) += con.J0t * con.prec * con.J0;
            dcnt(con.ndr - nFixed)++;
          }
        if (i1>=0)
          {
            dcnt(con.nd1 - nFixed)++;
            Matrix<double,6,6> tp = con.prec * con.J1;
            A.block<6,6>(i1,i1) += con.J1t * tp;
            if (i0>=0)
              {
                A.block<6,6>(i0,i1) += con.J0t * con.prec * con.J1;
                A.block<6,6>(i1,i0) += con.J1t * con.prec * con.J0;
              }
          }

        // add in 2 blocks of B
        if (i0>=0)
          B.block<6,1>(i0,0) -= con.J0t * con.prec * con.err;
        if (i1>=0)
          B.block<6,1>(i1,0) -= con.J1t * con.prec * con.err;
      } // finish P2 constraints

    // loop over Scale constraints
    if (scons.size() > 0)       // could be zero
      for(size_t pi=0; pi<scons.size(); pi++)
        {
          ConScale &con = scons[pi];
          con.setJacobians(nodes);
        // add in 4 blocks of A for t0, t1; actually just need upper triangular
        // i0 < i1
        int i0 = 6*(con.nd0-nFixed); // will be negative if fixed
        int i1 = 6*(con.nd1-nFixed); // will be negative if fixed
        
        if (i0>=0)
          {
            A.block<3,3>(i0,i0) += con.w * con.J0 * con.J0.transpose();
          }
        if (i1>=0)
          {
            A.block<3,3>(i1,i1) += con.w * con.J1 * con.J1.transpose();
            if (i0>=0)
              {
                A.block<3,3>(i0,i1) += con.w * con.J0 * con.J1.transpose();
                A.block<3,3>(i1,i0) = A.block<3,3>(i0,i1).transpose();
              }
          }

        // add in 2 blocks of B
        if (i0>=0)
          B.block<3,1>(i0,0) -= con.w * con.J0 * con.err;
        if (i1>=0)
          B.block<3,1>(i1,0) -= con.w * con.J1 * con.err;

        // scale variable, add in 5 blocks; actually just need upper triangular
        int is = 6*nFree+con.sv;
        A(is,is) += con.w * con.ks * con.ks;
        if (i0>=0)
          {
            A.block<3,1>(i0,is) += con.w * con.J0 * -con.ks;
            A.block<1,3>(is,i0) = A.block<3,1>(i0,is).transpose();
          }
        if (i1>=0)
          {
            A.block<3,1>(i1,is) += con.w * con.J1 * -con.ks;
            A.block<1,3>(is,i1) = A.block<3,1>(i1,is).transpose();
          }

        // add in scale block of B
        B(is) -= con.w * (-con.ks) * con.err;

      } // finish Scale constraints


    // augment diagonal
    A.diagonal() *= lam;

    // check the matrix and vector
    for (int i=0; i<6*nFree; i++)
      for (int j=0; j<6*nFree; j++)
        if (std::isnan(A(i,j)) ) { printf("[SetupSys] NaN in A\n"); *(int *)0x0 = 0; }

    for (int j=0; j<6*nFree; j++)
      if (std::isnan(B[j]) ) { printf("[SetupSys] NaN in B\n"); *(int *)0x0 = 0; }

    int ndc = 0;
    for (int i=0; i<nFree; i++)
      if (dcnt(i) == 0) ndc++;

    if (ndc > 0)
      cout << "[SetupSys] " << ndc << " disconnected nodes" << endl;
  }


  // Set up sparse linear system; see setupSys for algorithm.
  // Currently doesn't work with scale variables
  void SysSPA::setupSparseSys(double sLambda, int iter, int sparseType)
  {
    // set matrix sizes and clear
    // assumes scales vars are all free
    int nFree = nodes.size() - nFixed;

    //    long long t0, t1, t2, t3;
    //    t0 = utime();

    if (iter == 0)
      csp.setupBlockStructure(nFree); // initialize CSparse structures
    else
      csp.setupBlockStructure(0); // zero out CSparse structures

    //    t1 = utime();

    VectorXi dcnt(nFree);
    dcnt.setZero(nFree);

    // lambda augmentation
    double lam = 1.0 + sLambda;

    // loop over P2 constraints
    for(size_t pi=0; pi<p2cons.size(); pi++)
      {
        ConP2 &con = p2cons[pi];
        con.setJacobians(nodes);

        // add in 4 blocks of A; actually just need upper triangular
        int i0 = con.ndr-nFixed; // will be negative if fixed
        int i1 = con.nd1-nFixed; // will be negative if fixed
        
        if (i0>=0)
          {
           Matrix<double,6,6> m = con.J0t*con.prec*con.J0;
            csp.addDiagBlock(m,i0);
            dcnt(con.ndr - nFixed)++;
          }
        if (i1>=0)
          {
            dcnt(con.nd1 - nFixed)++;
            Matrix<double,6,6> tp = con.prec * con.J1;
            Matrix<double,6,6> m = con.J1t * tp;
            csp.addDiagBlock(m,i1);
            if (i0>=0)
              {
                Matrix<double,6,6> m2 = con.J0t * tp;
                if (i1 < i0)
                  {
                    m = m2.transpose();
                    csp.addOffdiagBlock(m,i1,i0);
                  }
                else
                  csp.addOffdiagBlock(m2,i0,i1);
              }
          }

        // add in 2 blocks of B
        if (i0>=0)
          csp.B.block<6,1>(i0*6,0) -= con.J0t * con.prec * con.err;
        if (i1>=0)
          csp.B.block<6,1>(i1*6,0) -= con.J1t * con.prec * con.err;
      } // finish P2 constraints

    //    t2 = utime();

    // set up sparse matrix structure from blocks
    if (sparseType == SBA_BLOCK_JACOBIAN_PCG)
      csp.incDiagBlocks(lam);   // increment diagonal block
    else
      csp.setupCSstructure(lam,iter==0); 

    //    t3 = utime();

    //    printf("\n[SetupSparseSys] Block: %0.1f   Cons: %0.1f  CS: %0.1f\n",
    //           (t1-t0)*.001, (t2-t1)*.001, (t3-t2)*.001);

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
  /// <useCSparse> is true for sparse Cholesky.
  ///                2 for gradient system, 3 for block jacobian PCG
  /// <initTol> is the initial tolerance for CG 
  /// <maxCGiters> is max # of iterations in BPCG

  int SysSPA::doSPA(int niter, double sLambda, int useCSparse, double initTol,
                      int maxCGiters)
  {
    Node::initDr();
    int nFree = nodes.size() - nFixed; // number of free nodes

    // number of nodes
    int ncams = nodes.size();
    // number of scale variables
    int nscales = scales.size();

    // save old scales
    vector<double> oldscales;   
    oldscales.resize(nscales);
    
    // initialize vars
    if (sLambda > 0.0)          // do we initialize lambda?
      lambda = sLambda;

    // set number of constraints
    int ncons = p2cons.size();

    // check for fixed frames
    for (int i=0; i<ncams; i++)
      {
        Node &nd = nodes[i];
        if (i >= nFixed)
          nd.isFixed = false;
        else 
          nd.isFixed = true;
        nd.setTransform();      // set up world-to-node transform for cost calculation
        nd.setDr(true);         // always use local angles
      }

    // initialize vars
    double laminc = 2.0;        // how much to increment lambda if we fail
    double lamdec = 0.5;        // how much to decrement lambda if we succeed
    int iter = 0;               // iterations
    sqMinDelta = 1e-8 * 1e-8;
    double cost = calcCost();
    if (verbose)
      cout << iter << " Initial squared cost: " << cost << " which is " 
           << sqrt(cost/ncons) << " rms error" << endl; 

    int good_iter = 0;
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
        else if (useCSparse > 0)
        {
            bool ok = csp.doChol();
            if (!ok)
              cout << "[DoSPA] Sparse Cholesky failed!" << endl;
        }
        else
          A.ldlt().solveInPlace(B); // Cholesky decomposition and solution

        // get correct result vector
        VectorXd &BB = useCSparse ? csp.B : B;

        // check for convergence
        // this is a pretty crummy convergence measure...
        double sqDiff = BB.squaredNorm();
        if (sqDiff < sqMinDelta) // converged, done...
        {
          break;
        }

        // update the frames
        int ci = 0;
        for(int i=0; i < ncams; i++)
        {
            Node &nd = nodes[i];
            if (nd.isFixed) continue; // not to be updated
            nd.oldtrans = nd.trans; // save in case we don't improve the cost
            nd.oldqrot = nd.qrot;
            nd.trans.head<3>() += BB.segment<3>(ci);

            Quaternion<double> qr;
            qr.vec() = BB.segment<3>(ci+3); 
            qr.w() = sqrt(1.0 - qr.vec().squaredNorm());

            Quaternion<double> qrn,qrx;
            qrn = nd.qrot;
            qr = qrn*qr;        // post-multiply
            qr.normalize();
            if (qr.w() < 0.0)
              nd.qrot.coeffs() = -qr.coeffs();
            else
              nd.qrot.coeffs() = qr.coeffs();

            nd.setTransform();  // set up projection matrix for cost calculation
            nd.setDr(true);     // set rotational derivatives
            ci += 6;            // advance B index
        }

        // update the scales
        ci = 6*nFree;       // head of scale vars
        if (nscales > 0)        // could be empty
          for(int i=0; i < nscales; i++)
          {
              oldscales[i] = scales[i];
              scales[i] += B(ci);
              ci++;
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
                Node &nd = nodes[i];
                if (nd.isFixed) continue; // not to be updated
                nd.trans = nd.oldtrans;
                nd.qrot = nd.oldqrot;
                nd.setTransform(); // set up projection matrix for cost calculation
                nd.setDr(true);
            }

            // reset scales
            if (nscales > 0)    // could be empty
              for(int i=0; i < nscales; i++)
                scales[i] = oldscales[i];


            cost = calcCost();  // need to reset errors
            if (verbose)
              cout << iter << " Downdated cost: " << cost << endl;
            // NOTE: shouldn't need to redo all calcs in setupSys
        }
      }

    // return number of iterations performed
    return good_iter;

  }


  // write out the precision matrix for CSparse
  void SysSPA::writeSparseA(char *fname, bool useCSparse)
  {
    ofstream ofs(fname);
    if (!ofs)
      {
        cout << "Can't open file " << fname << endl;
        return;
      }

    // cameras
    if (useCSparse)
      {
        setupSparseSys(0.0,0,useCSparse);
        
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


  // Set up spanning tree initialization
  void SysSPA::spanningTree(int node)
  {
    int nnodes = nodes.size();

    // set up an index from nodes to their constraints
    vector<vector<int> > cind;
    cind.resize(nnodes);

    for(size_t pi=0; pi<p2cons.size(); pi++)
      {
        ConP2 &con = p2cons[pi];
        int i0 = con.ndr;
        int i1 = con.nd1;
        cind[i0].push_back(i1);
        cind[i1].push_back(i0);        
      }

    // set up breadth-first algorithm
    VectorXd dist(nnodes);
    dist.setConstant(1e100);
    if (node >= nnodes)
      node = 0;
    dist[node] = 0.0;
    std::multimap<double,int> open;  // open list, priority queue - can have duplicates
    open.emplace(0.0,node);

    // do breadth-first computation
    while (!open.empty())
      {
        // get top node, remove it
        int ni = open.begin()->second;
        double di = open.begin()->first;
        open.erase(open.begin());
        if (di > dist[ni]) continue; // already dealt with

        // update neighbors
        Node &nd = nodes[ni];
        Matrix<double,3,4> n2w;
        transformF2W(n2w,nd.trans,nd.qrot); // from node to world coords

        vector<int> &nns = cind[ni];
        for (int i=0; i<(int)nns.size(); i++)
          {
            ConP2 &con = p2cons[nns[i]];
            double dd = con.tmean.norm(); // incremental distance
            // neighbor node index
            int nn = con.nd1;
            if (nn == ni)
              nn = con.ndr;
            Node &nd2 = nodes[nn];
            Vector3d tmean = con.tmean;
            Quaterniond qpmean = con.qpmean;
            if (nn == con.ndr)       // wrong way, reverse
              {
                qpmean = qpmean.inverse();
                tmean = nd.qrot.toRotationMatrix().transpose()*nd2.qrot.toRotationMatrix()*tmean;
              }
                
            if (dist[nn] > di + dd) // is neighbor now closer?
              {
                // set priority queue
                dist[nn] = di+dd;
                open.emplace(di+dd,nn);
                // update initial pose
                Vector4d trans;
                trans.head(3) = tmean;
                trans(3) = 1.0;
                nd2.trans.head(3) = n2w*trans;
                nd2.qrot = qpmean*nd.qrot;
                nd2.normRot();
                nd2.setTransform();
                nd2.setDr(true);
              }
          }
      }
    
  }
  

}  // namespace sba
