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
// block preconditioned conjugate gradient
// 6x6 blocks at present, should be templatized
//

#ifndef _BPCG_H_
#define _BPCG_H_

#ifndef EIGEN_USE_NEW_STDVECTOR
#define EIGEN_USE_NEW_STDVECTOR
#endif // EIGEN_USE_NEW_STDVECTOR

#include <vector>
#include <map>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/StdVector>

using namespace Eigen;
using namespace std;

//typedef Matrix<double,6,1> Vector6d;
//typedef Vector6d::AlignedMapType AV6d;

namespace sba
{
  /// Let's try templated versions

  template <int N>
    class jacobiBPCG 
    {
    public:
      jacobiBPCG() { residual = 0.0; };
      int doBPCG(int iters, double tol,
                 vector< Matrix<double,N,N>, aligned_allocator<Matrix<double,N,N> > > &diag,
                 vector< map<int,Matrix<double,N,N>, less<int>, aligned_allocator<Matrix<double,N,N> > > > &cols,
                 VectorXd &x,
                 VectorXd &b,
                 bool abstol = false,
                 bool verbose = false
                 );

      // uses internal linear storage for Hessian
      int doBPCG2(int iters, double tol,
                 vector< Matrix<double,N,N>, aligned_allocator<Matrix<double,N,N> > > &diag,
                 vector< map<int,Matrix<double,N,N>, less<int>, aligned_allocator<Matrix<double,N,N> > > > &cols,
                 VectorXd &x,
                 VectorXd &b,
                 bool abstol = false,
                 bool verbose = false
                 );

      double residual;

    private:
      void mMV(vector< Matrix<double,N,N>, aligned_allocator<Matrix<double,N,N> > > &diag,
               vector< map<int,Matrix<double,N,N>, less<int>, aligned_allocator<Matrix<double,N,N> > > > &cols,
               const VectorXd &vin,
               VectorXd &vout);
 
      // uses internal linear storage
      void mMV2(vector< Matrix<double,N,N>, aligned_allocator<Matrix<double,N,N> > > &diag,
                const VectorXd &vin,
                VectorXd &vout);
 
      void mD(vector< Matrix<double,N,N>, aligned_allocator<Matrix<double,N,N> > > &diag,
              VectorXd &vin,
              VectorXd &vout);

      vector<int> vcind, vrind;
      vector< Matrix<double,N,N>, aligned_allocator<Matrix<double,N,N> > > vcols;
    };

  template <int N>
    void jacobiBPCG<N>::mMV(vector< Matrix<double,N,N>, aligned_allocator<Matrix<double,N,N> > > &diag,
                        vector< map<int,Matrix<double,N,N>, less<int>, aligned_allocator<Matrix<double,N,N> > > > &cols,
                        const VectorXd &vin,
                        VectorXd &vout)
    {
    // loop over all entries
      if (cols.size() > 0)
        for (int i=0; i<(int)cols.size(); i++)
          {
            vout.segment<N>(i*N) = diag[i]*vin.segment<N>(i*N); // only works with cols ordering

            map<int,Matrix<double,N,N>, less<int>, 
              aligned_allocator<Matrix<double,N,N> > > &col = cols[i];
            if (col.size() > 0)
              {
                typename map<int,Matrix<double,N,N>, less<int>, // need "typename" here, barf
                  aligned_allocator<Matrix<double,N,N > > >::iterator it;
                for (it = col.begin(); it != col.end(); it++)
                  {
                    int ri = (*it).first; // get row index
                    const Matrix<double,N,N> &M = (*it).second; // matrix
                    vout.segment<N>(i*N)  += M.transpose()*vin.segment<N>(ri*N);
                    vout.segment<N>(ri*N) += M*vin.segment<N>(i*N);
                  }
              }
          }
    }

  //
  // matrix-vector multiply with linear storage
  //

  template <int N>
    void jacobiBPCG<N>::mMV2(vector< Matrix<double,N,N>, aligned_allocator<Matrix<double,N,N> > > &diag,
                             const VectorXd &vin,
                             VectorXd &vout)
    {
      // linear storage for matrices
      // loop over off-diag entries
      if (diag.size() > 0)
        for (int i=0; i<(int)diag.size(); i++)
          vout.segment<N>(i*N) = diag[i]*vin.segment<N>(i*N); // only works with cols ordering

      for (int i=0; i<(int)vcind.size(); i++)
        {
          int ri = vrind[i];
          int ii = vcind[i];
          const Matrix<double,N,N> &M = vcols[i];
          vout.segment<N>(ri*N) += M*vin.segment<N>(ii*N);
          vout.segment<N>(ii*N) += M.transpose()*vin.segment<N>(ri*N);
        }
    }




  template <int N>
    void jacobiBPCG<N>::mD(vector< Matrix<double,N,N>, aligned_allocator<Matrix<double,N,N> > > &diag,
            VectorXd &vin,
            VectorXd &vout)
    {
      // loop over diag entries
      for (int i=0; i<(int)diag.size(); i++)
        vout.segment<N>(i*N) = diag[i]*vin.segment<N>(i*N);
    }


  template <int N>
    int jacobiBPCG<N>::doBPCG(int iters, double tol,
	    vector< Matrix<double,N,N>, aligned_allocator<Matrix<double,N,N> > > &diag,
	    vector< map<int,Matrix<double,N,N>, less<int>, aligned_allocator<Matrix<double,N,N> > > > &cols,
	    VectorXd &x,
	    VectorXd &b,
	    bool abstol,
	    bool verbose)
    {
      // set up local vars
      VectorXd r,d,q,s;
      int n = diag.size();
      int n6 = n*N;
      r.setZero(n6);
      d.setZero(n6);
      q.setZero(n6);
      s.setZero(n6);

      // set up Jacobi preconditioner
      vector< Matrix<double,N,N>, aligned_allocator<Matrix<double,N,N> > > J;
      J.resize(n);
      for (int i=0; i<n; i++)
        J[i] = diag[i].inverse();

      int i;
      r = b;
      mD(J,r,d);
      double dn = r.dot(d);
      double d0 = tol*dn;
      if (abstol)               // change tolerances
        {
          if (residual > d0) d0 = residual;
        }

      for (i=0; i<iters; i++)
        {
          if (verbose && 0)
            cout << "[BPCG] residual[" << i << "]: " << dn << " < " << d0 << endl;
          if (dn < d0) break;	// done
          mMV(diag,cols,d,q);
          double a = dn / d.dot(q);
          x += a*d;
          // TODO: reset residual here every 50 iterations
          r -= a*q;
          mD(J,r,s);
          double dold = dn;
          dn = r.dot(s);
          double ba = dn / dold;
          d = s + ba*d;
        }

  
      if (verbose)
        cout << "[BPCG] residual[" << i << "]: " << dn << endl;
      residual = dn/2.0;
      return i;
    }


  // uses internal linear storage for Hessian
  template <int N>
    int jacobiBPCG<N>::doBPCG2(int iters, double tol,
	    vector< Matrix<double,N,N>, aligned_allocator<Matrix<double,N,N> > > &diag,
	    vector< map<int,Matrix<double,N,N>, less<int>, aligned_allocator<Matrix<double,N,N> > > > &cols,
	    VectorXd &x,
	    VectorXd &b,
	    bool abstol,
	    bool verbose)
    {
      // set up local vars
      VectorXd r,d,q,s;
      int n = diag.size();
      int n6 = n*N;
      r.setZero(n6);
      d.setZero(n6);
      q.setZero(n6);
      s.setZero(n6);

      vcind.clear();
      vrind.clear();
      vcols.clear();

      // set up alternate rep for sparse matrix
      for (int i=0; i<(int)cols.size(); i++)
        {
          map<int,Matrix<double,N,N>, less<int>, 
            aligned_allocator<Matrix<double,N,N> > > &col = cols[i];
          if (col.size() > 0)
            {
              typename map<int,Matrix<double,N,N>, less<int>, 
                aligned_allocator<Matrix<double,N,N> > >::iterator it;
              for (it = col.begin(); it != col.end(); it++)
                {
                  int ri = (*it).first; // get row index
                  vrind.push_back(ri);
                  vcind.push_back(i);
                  vcols.push_back((*it).second);
                }
            }
        }

      // set up Jacobi preconditioner
      vector< Matrix<double,N,N>, aligned_allocator<Matrix<double,N,N> > > J;
      J.resize(n);
      for (int i=0; i<n; i++)
        J[i] = diag[i].inverse();

      int i;
      r = b;
      mD(J,r,d);
      double dn = r.dot(d);
      double d0 = tol*dn;
      if (abstol)               // change tolerances
        {
          if (residual > d0) d0 = residual;
        }

      for (i=0; i<iters; i++)
        {
          if (verbose && 0)
            cout << "[BPCG] residual[" << i << "]: " << dn << " < " << d0 << endl;
          if (dn < d0) break;	// done
          mMV2(diag,d,q);
          double a = dn / d.dot(q);
          x += a*d;
          // TODO: reset residual here every 50 iterations
          r -= a*q;
          mD(J,r,s);
          double dold = dn;
          dn = r.dot(s);
          double ba = dn / dold;
          d = s + ba*d;
        }

  
      if (verbose)
        cout << "[BPCG] residual[" << i << "]: " << dn << endl;
      residual = dn/2.0;
      return i;
    }


#if 0
//
// matrix multiply of compressed column storage + diagonal blocks by a vector
//

void
mMV(vector< Matrix<double,6,6>, aligned_allocator<Matrix<double,6,6> > > &diag,
    vector< map<int,Matrix<double,6,6>, less<int>, aligned_allocator<Matrix<double,6,6> > > > &cols,
    const VectorXd &vin,
    VectorXd &vout);

//
// jacobi-preconditioned block conjugate gradient
// returns number of iterations

int
bpcg_jacobi(int iters, double tol,
	    vector< Matrix<double,6,6>, aligned_allocator<Matrix<double,6,6> > > &diag,
	    vector< map<int,Matrix<double,6,6>, less<int>, aligned_allocator<Matrix<double,6,6> > > > &cols,
	    VectorXd &x,
	    VectorXd &b,
	    bool abstol = false,
	    bool verbose = false
         );

int
bpcg_jacobi_dense(int iters, double tol,
		  MatrixXd &M,
		  VectorXd &x,
		  VectorXd &b);

//
// jacobi-preconditioned block conjugate gradient
// returns number of iterations

int
bpcg_jacobi3(int iters, double tol,
	    vector< Matrix<double,3,3>, aligned_allocator<Matrix<double,3,3> > > &diag,
	    vector< map<int,Matrix<double,3,3>, less<int>, aligned_allocator<Matrix<double,3,3> > > > &cols,
	    VectorXd &x,
	    VectorXd &b,
	    bool abstol = false,
	    bool verbose = false
         );

int
bpcg_jacobi_dense3(int iters, double tol,
		  MatrixXd &M,
		  VectorXd &x,
		  VectorXd &b);

#endif

}  // end namespace sba

#endif // BPCG_H
