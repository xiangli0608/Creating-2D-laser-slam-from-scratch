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
// Interface to CSparse
//

#ifndef _CSPARSE_H_
#define _CSPARSE_H_

#ifndef EIGEN_USE_NEW_STDVECTOR
#define EIGEN_USE_NEW_STDVECTOR
#endif // EIGEN_USE_NEW_STDVECTOR

#include <stdio.h>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/StdVector>

// CSparse header
extern "C" {
#include "suitesparse/cs.h"
}
#include <vector>
#include <map>

// Cholmod header, other header files brought in
#ifdef SBA_CHOLMOD
#include "suitesparse/cholmod.h"
#endif

// these are for SparseLib and IML, testing the Delayed State Filter
#ifdef SBA_DSIF
#include "SparseLib/compcol_double.h" // Compressed column matrix header
#include "SparseLib/mvblasd.h"  // MV_Vector level 1 BLAS
#include "SparseLib/icpre_double.h" // Diagonal preconditioner
#include "SparseLib/cg.h"       // IML++ CG template
#endif

// block jacobian PCG
#include "sparse_bundle_adjustment/bpcg/bpcg.h"

using namespace Eigen;
using namespace std;

namespace sba
{
  class CSparse
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // needed for 16B alignment

    // constructor
    CSparse();

    // destructor
    ~CSparse();

    // storage of diagonal blocks
    vector< Matrix<double,6,6>, aligned_allocator<Matrix<double,6,6> > > diag;
    // compressed column storage of blocks
    vector< map<int,Matrix<double,6,6>, less<int>, 
                aligned_allocator<Matrix<double,6,6> > > > cols;

    void setupBlockStructure(int n); // size of rows/cols of A (in blocks)
    
    // add in blocks
    inline void addDiagBlock(Matrix<double,6,6> &m, int n)
      { diag[n]+=m; };
    void incDiagBlocks(double lam);
    void addOffdiagBlock(Matrix<double,6,6> &m, int ii, int jj);

    // set up compressed column structure; <init> true if first time
    // <diaginc> is the diagonal multiplier for LM
    void setupCSstructure(double diaginc, bool init=false); 

    // write cs structure into a dense Eigen matrix
    void uncompress(MatrixXd &m);

    // parameters
    int asize, csize;           // matrix A is asize x asize (blocks), csize x csize (elements)
    int nnz;                    // number of non-zeros in A

    // CSparse structures
    cs *A;                      // linear problem matrix

    // RHS Eigen vector
    VectorXd B;

    // which algorithm?
    bool useCholmod;

    // doing the Cholesky with CSparse or Cholmod
    bool doChol();              // solve in place with RHS B

    // doing the BPCG
    // max iterations <iter>, ending toleranace <tol>, current sba iteration <sba_iter>
    int doBPCG(int iters, double tol, int sba_iter);
    // CG structure for 6x6 matrices
    jacobiBPCG<6> bpcg;

#ifdef SBA_CHOLMOD
    // CHOLMOD structures
    bool chInited;
    cholmod_sparse *chA;        // linear problem matrix
    cholmod_common *chc;
    cholmod_common Common;
#endif

  };


  class CSparse2d
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // needed for 16B alignment

    // constructor
    CSparse2d();

    // destructor
    ~CSparse2d();

    // storage of diagonal blocks
    vector< Matrix<double,3,3>, aligned_allocator<Matrix<double,3,3> > > diag;
    // compressed column storage of blocks
    vector< map<int,Matrix<double,3,3>, less<int>, 
                aligned_allocator<Matrix<double,3,3> > > > cols;

    void setupBlockStructure(int n, bool eraseit = true); // size of rows/cols of A (in blocks)
    
    // add in blocks
    inline void addDiagBlock(Matrix<double,3,3> &m, int n)
      { diag[n]+=m; };
    void addOffdiagBlock(Matrix<double,3,3> &m, int ii, int jj);
    void incDiagBlocks(double lam);

    // set up compressed column structure; <init> true if first time
    // <diaginc> is the diagonal multiplier for LM
    void setupCSstructure(double diaginc, bool init=false); 

    // write cs structure into a dense Eigen matrix
    void uncompress(MatrixXd &m);

    // parameters
    int asize, csize;           // matrix A is asize x asize (blocks), csize x csize (elements)
    int nnz;                    // number of non-zeros in A

    // CSparse2d structures
    cs *A, *AF;                 // linear problem matrices, A upper diagonal, AF symmetric

    // RHS Eigen vector
    VectorXd B, Bprev;

    // which algorithm?
    bool useCholmod;

    // doing the Cholesky
    bool doChol();              // solve in place with RHS B

    // doing PCG with incomplete Cholesky preconditioner
    // returns 0 on success, 1 on not achieving tolerance, >1 on other errors
    int doPCG(int iters);

    // doing the BPCG
    // max iterations <iter>, ending toleranace <tol>    
    int doBPCG(int iters, double tol, int sba_iter);
    // CG structure for 3x3 matrices
    jacobiBPCG<3> bpcg;

#ifdef SBA_CHOLMOD
    // CHOLMOD structures
    bool chInited;
    cholmod_sparse *chA;        // linear problem matrix
    cholmod_common *chc;
    cholmod_common Common;
#endif

  };


} // end namespace sba

#endif  // _CSPARSE_H
