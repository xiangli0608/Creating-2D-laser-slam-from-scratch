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

#include <stdio.h>
#include "sparse_bundle_adjustment/csparse.h"

using namespace Eigen;

#include <iostream>
#include <iomanip>
#include <fstream>
#include <algorithm>

using namespace std;


//
// Efficient interface to the CSparse sparse solver
//
// Basic idea is to store 6x6 blocks in a sparse structure,
//   and when we're done forming the A matrix, transfer to
//   a CSparse compressed structure.  The latter only has
//   to be initialized once at the start of the nonlinear iterations.
//

//
// CSparse holds its sparse NxN arrays in compressed column format:
//
//  col_ptr has N+1 entries, one for each column, and a final NNZ entry.
//    each entry is the index in row_ptr of where the column starts.
//  row_ind has NNZ entries, each one a row index for an element.
//  vals    has NNZ entries corresponding to the row_ptr entries
//  

namespace sba
{

  CSparse::CSparse()
  {
    A = NULL;
#ifdef SBA_CHOLMOD
    chA = NULL;
    chInited = false;
#endif
    useCholmod = false;
    asize = 0;
    csize = 0;
    nnz = 0;
  }

  CSparse::~CSparse()
  {
    if (A) cs_spfree(A);        // free any previous structure
  }


  // 6x6 sparse jacobian blocks
  void CSparse::setupBlockStructure(int n)
  {
    if (n > 0)                  // set up initial structure
      {
        diag.clear();
        diag.resize(n);
        cols.clear();
        cols.resize(n);
        asize = n;
        csize = 6*n;
      }

    // zero out entries
    B.setZero(csize);
    for (int i=0; i<(int)diag.size(); i++)
      diag[i].setZero();
    for (int i=0; i<(int)cols.size(); i++)
      {
        map<int,Matrix<double,6,6>, less<int>, 
          aligned_allocator<Matrix<double,6,6> > > &col = cols[i];
        if (col.size() > 0)
          {
            map<int,Matrix<double,6,6>, less<int>, 
              aligned_allocator<Matrix<double,6,6> > >::iterator it;
            for (it = col.begin(); it != col.end(); it++)
              it->second.setZero();
          }
      }
  }

  void CSparse::incDiagBlocks(double lam)
  {
    for (int i=0; i<(int)diag.size(); i++)
      diag[i].diagonal() *= lam;
  }

  // add an off-diagonal block
  void CSparse::addOffdiagBlock(Matrix<double,6,6> &m, int ii, int jj)
  {
    // get column 
    map<int,Matrix<double,6,6>, less<int>, 
      aligned_allocator<Matrix<double,6,6> > > &col = cols[jj];
    // find current matrix
    map<int,Matrix<double,6,6>, less<int>, 
      aligned_allocator<Matrix<double,6,6> > >::iterator it;
    it = col.find(ii);
    if (it == col.end())        // didn't find it
      col.insert(pair<int,Matrix<double,6,6> >(ii,m));
    else                        // found it, add
      it->second += m;
  }


  // set up CSparse structure; <init> true if first time
  // <diaginc> is the diagonal multiplier for LM

  // this version sets upper triangular matrix,
  void CSparse::setupCSstructure(double diaginc, bool init)
  {
#ifdef SBA_CHOLMOD
    if (useCholmod) {
      cholmod_start(&Common); // this is finished in doChol()
      Common.print = 0;
    }
#endif

    // reserve space and set things up
    if (init || useCholmod)
      {
        // count entries for cs allocation
        nnz = 21*asize;         // diagonal entries, just upper triangle
        for (int i=0; i<(int)cols.size(); i++)
          {
            map<int,Matrix<double,6,6>, less<int>, 
              aligned_allocator<Matrix<double,6,6> > > &col = cols[i];
            nnz += 36 * col.size(); // 6x6 matrix
          }

#ifdef SBA_CHOLMOD
        if (useCholmod)
          {
            //            cholmod_start(&Common); // this is finished in doChol()
            //            if (chA) 
            //              cholmod_free_sparse(&chA, &Common);
            chA = cholmod_allocate_sparse(csize,csize,nnz,true,true,1,CHOLMOD_REAL,&Common);
          }
        else
#endif
          {
            if (A) cs_spfree(A);    // free any previous structure
            A = cs_spalloc(csize,csize,nnz,1,0); // allocate sparse matrix
          }

        // now figure out the column pointers
        int colp = 0;           // index of where the column starts in Ai
        int *Ap, *Ai;
#ifdef SBA_CHOLMOD
        if (useCholmod)
          {
            Ap = (int *)chA->p; // column pointer
            Ai = (int *)chA->i; // row indices
          }
        else
#endif
          {
            Ap = A->p;          // column pointer
            Ai = A->i;          // row indices
          }
        for (int i=0; i<(int)cols.size(); i++)
          {
            // column i entries
            map<int,Matrix<double,6,6>, less<int>, 
              aligned_allocator<Matrix<double,6,6> > > &col = cols[i];

            // do this for 6 columns
            for (int k=0; k<6; k++)
              {
                *Ap++ = colp;
                int row;

                // iterate over the map
                if (col.size() > 0)
                  {
                    // map iterator
                    map<int,Matrix<double,6,6>, less<int>, 
                      aligned_allocator<Matrix<double,6,6> > >::iterator it;

                    // iterate over block column entries
                    for (it = col.begin(); it != col.end(); it++)
                      {
                        row = 6*it->first; // which row we're at
                        for (int j=0; j<6; j++)
                          Ai[colp++] = row++;
                      }
                  }

                // add in diagonal entries
                row = 6*i;
                for (int kk=0; kk<k+1; kk++)
                  Ai[colp++] = row++;
              }
           }        
          *Ap = nnz;            // last entry
       }



     // now put the entries in place
     int colp = 0;              // index of where the column starts in Ai
     double *Ax;
#ifdef SBA_CHOLMOD
     if (useCholmod)
       Ax = (double *)chA->x;   // values
     else
#endif
       Ax = A->x;               // values
     for (int i=0; i<(int)cols.size(); i++)
       {
         // column i entries
         map<int,Matrix<double,6,6>, less<int>, 
           aligned_allocator<Matrix<double,6,6> > > &col = cols[i];

         // do this for 6 columns
         for (int k=0; k<6; k++)
           {
             // iterate over the map
             if (col.size() > 0)
               {
                 // map iterator
                 map<int,Matrix<double,6,6>, less<int>, 
                   aligned_allocator<Matrix<double,6,6> > >::iterator it;

                 // iterate over block column entries
                 for (it = col.begin(); it != col.end(); it++)
                   {
                     Matrix<double,6,6> &m = it->second;
                     for (int j=0; j<6; j++)
                       Ax[colp++] = m(j,k);
                   }
               } 

             // add in diagonal entries
             Matrix<double,6,6> &m = diag[i]; // diagonal block
             for (int kk=0; kk<k+1; kk++)
               Ax[colp++] = m(kk,k);
             Ax[colp-1] *= diaginc; // increment diagonal for LM
           }
       }      

  }


  // uncompress the compressed A into a dense Eigen matrix
  void CSparse::uncompress(MatrixXd &m)
  {
    if (!A) return;
    m.setZero(csize,csize);
    
    int *Ap = A->p;             // column pointer
    int *Ai = A->i;             // row indices
    double *Ax = A->x;          // values;

    for (int i=0; i<csize; i++)
      {
        int rbeg = Ap[i];
        int rend = Ap[i+1];
        if (rend > rbeg)
          for (int j=rbeg; j<rend; j++)
            m(Ai[j],i) = Ax[j];
      }
  }

  // solve in place, returns RHS B
  bool CSparse::doChol()
  {
#ifdef SBA_CHOLMOD
    if (useCholmod)
      {
        cholmod_dense *x, b, *R, *R2;
        cholmod_factor *L ;
        double *Xx, *Rx, *bb;
        double one [2], minusone [2];
        one [0] = 1 ;
        one [1] = 0 ;
        minusone [0] = -1 ;
        minusone [1] = 0 ;

        //        cholmod_start (&Common) ;    // start it up ???
        cholmod_print_sparse (chA, (char *)"A", &Common) ; // print simple stats
        b.nrow = csize;
        b.ncol = 1;
        b.d = csize;                // leading dimension (???)
        b.nzmax = csize;
        b.xtype = CHOLMOD_REAL;
        b.dtype = CHOLMOD_DOUBLE;
        b.x = B.data();
        //cout << "CHOLMOD analyze..." << flush;
        L = cholmod_analyze (chA, &Common) ; // analyze 
        //cout << "factorize..." << flush;
        cholmod_factorize (chA, L, &Common) ; // factorize 
        //cout << "solve..." << flush;
        x = cholmod_solve (CHOLMOD_A, L, &b, &Common) ; // solve Ax=b
        //        cholmod_print_factor (L, (char *)"L", &Common) ;

        //cout << "refine" << endl;
        // one step of iterative refinement, cheap
	/* Ax=b was factorized and solved, R = B-A*X */
	R = cholmod_copy_dense (&b, &Common) ;
	cholmod_sdmult(chA, 0, minusone, one, x, R, &Common) ;
	/* R2 = A\(B-A*X) */
	R2 = cholmod_solve (CHOLMOD_A, L, R, &Common) ;
	/* compute X = X + A\(B-A*X) */
	Xx = (double *)x->x ;
	Rx = (double *)R2->x ;
	for (int i=0 ; i<csize ; i++)
	{
          Xx[i] = Xx[i] + Rx[i] ;
	}
	cholmod_free_dense (&R2, &Common) ;
	cholmod_free_dense (&R, &Common) ;

        bb = B.data();
        for (int i=0; i<csize; i++) // transfer answer
          *bb++ = *Xx++;
        cholmod_free_factor (&L, &Common) ; // free matrices 
        cholmod_free_dense (&x, &Common) ;
        cholmod_free_sparse(&chA, &Common);
        cholmod_finish (&Common) ;   // finish it ???

        return true;
      }
    else
#endif
      {
        // using order 0 here (natural order); 
        // may be better to use "1" for large problems
        int order = 0;
        if (csize > 400) order = 1;
        bool ok = (bool)cs_cholsol(order,A,B.data()); // do the CSparse thang
        return ok;
      }
  }


  // 
  // block jacobian PCG
  // max iterations <iter>, ending toleranace <tol>
  //

  int 
  CSparse::doBPCG(int iters, double tol, int sba_iter)
  {
    int n = B.rows();
    VectorXd x;
    x.setZero(n);
    bool abstol = false;
    if (sba_iter > 0) abstol = true;
    int ret;
    ret = bpcg.doBPCG2(iters, tol, diag, cols, x, B, abstol);
    B = x;			// transfer result data
    return ret;
  }


  //
  // 2d version
  //

  CSparse2d::CSparse2d()
  {
    A = NULL;
    AF = NULL;
#ifdef SBA_CHOLMOD
    chA = NULL;
    chInited = false;
    Common.print=0;
#endif
    useCholmod = false;
    asize = 0;
    csize = 0;
    nnz = 0;
    Bprev.resize(0);
  }

  CSparse2d::~CSparse2d()
  {
    if (A) cs_spfree(A);        // free any previous structure
    if (AF) cs_spfree(AF);      // free any previous structure
  }


  // 3x3 sparse jacobian blocks
  void CSparse2d::setupBlockStructure(int n, bool eraseit)
  {
    if (n > 0)                  // set up initial structure
      {
        diag.resize(n);
        cols.resize(n);
	if (eraseit)
	  for (int i=0; i<(int)cols.size(); i++)
	    {
	      map<int,Matrix<double,3,3>, less<int>, 
		aligned_allocator< Matrix <double,3,3> > > &col = cols[i];
              col.clear();
            }
        asize = n;
        csize = 3*n;
      }

    if (eraseit)
      {
	// zero out entries
	B.setZero(csize);
	for (int i=0; i<(int)diag.size(); i++)
	  diag[i].setZero();
	for (int i=0; i<(int)cols.size(); i++)
	  {
	    map<int,Matrix<double,3,3>, less<int>, 
	      aligned_allocator<Matrix<double,3,3> > > &col = cols[i];
            if (col.size() > 0)
	      {
		map<int,Matrix<double,3,3>, less<int>, 
		  aligned_allocator<Matrix<double,3,3> > >::iterator it;
                for (it = col.begin(); it != col.end(); it++)
		  it->second.setZero();
              }
          }
      }

    else			// here we just resize B, saving the old parts
      {
	B.setZero(csize);
	if (Bprev.size() > 0)
	  B.head(Bprev.size()) = Bprev;
      }
  }


  // add an off-diagonal block
  void CSparse2d::addOffdiagBlock(Matrix<double,3,3> &m, int ii, int jj)
  {
    // get column 
    map<int,Matrix<double,3,3>, less<int>, 
      aligned_allocator<Matrix<double,3,3> > > &col = cols[jj];
    // find current matrix
    map<int,Matrix<double,3,3>, less<int>, 
      aligned_allocator<Matrix<double,3,3> > >::iterator it;
    it = col.find(ii);
    if (it == col.end())        // didn't find it
      col.insert(pair<int,Matrix<double,3,3> >(ii,m));
    else                        // found it, add
      it->second += m;
  }


  void CSparse2d::incDiagBlocks(double lam)
  {
    for (int i=0; i<(int)diag.size(); i++)
      diag[i].diagonal() *= lam;
  }


  // set up CSparse2d structure; <init> true if first time
  // <diaginc> is the diagonal multiplier for LM

  // this version only sets upper triangular matrix,
  //   should set whole thing

  void CSparse2d::setupCSstructure(double diaginc, bool init)
  {
#ifdef SBA_CHOLMOD
    if (useCholmod) {
      cholmod_start(&Common); // this is finished in doChol()
      Common.print = 0;
    }
#endif

    // reserve space and set things up
    if (init || useCholmod)
      {
        if (A) cs_spfree(A);    // free any previous structure

        // count entries for cs allocation
        nnz = 6*asize;         // diagonal entries, just upper triangle
        for (int i=0; i<(int)cols.size(); i++)
          {
            map<int,Matrix<double,3,3>, less<int>, 
              aligned_allocator<Matrix<double,3,3> > > &col = cols[i];
            nnz += 9 * col.size(); // 3x3 matrix
          }
#ifdef SBA_CHOLMOD
        if (useCholmod)
          {
            //            cholmod_start(&Common); // this is finished in doChol()
            //            if (chA) 
            //              cholmod_free_sparse(&chA, &Common);
            chA = cholmod_allocate_sparse(csize,csize,nnz,true,true,1,CHOLMOD_REAL,&Common);
          }
        else
#endif
	  {
	    A = cs_spalloc(csize,csize,nnz,1,0); // allocate sparse matrix
	  }
        
        // now figure out the column pointers
        int colp = 0;           // index of where the column starts in Ai
        int *Ap, *Ai;
#ifdef SBA_CHOLMOD
        if (useCholmod)
          {
            Ap = (int *)chA->p; // column pointer
            Ai = (int *)chA->i; // row indices
          }
        else
#endif
          {
            Ap = A->p;          // column pointer
            Ai = A->i;          // row indices
          }

        for (int i=0; i<(int)cols.size(); i++)
          {
            // column i entries
            map<int,Matrix<double,3,3>, less<int>, 
              aligned_allocator<Matrix<double,3,3> > > &col = cols[i];

            // do this for 3 columns
            for (int k=0; k<3; k++)
              {
                *Ap++ = colp;
                int row;

                // iterate over the map
                if (col.size() > 0)
                  {
                    // map iterator
                    map<int,Matrix<double,3,3>, less<int>, 
                      aligned_allocator<Matrix<double,3,3> > >::iterator it;

                    // iterate over block column entries
                    for (it = col.begin(); it != col.end(); it++)
                      {
                        row = 3*it->first; // which row we're at
                        for (int j=0; j<3; j++)
                          Ai[colp++] = row++;
                      }
                  }

                // add in diagonal entries
                row = 3*i;
                for (int kk=0; kk<k+1; kk++)
                  Ai[colp++] = row++;
              }
           }        
          *Ap = nnz;       // last entry
       }

     // now put the entries in place
     int colp = 0;           // index of where the column starts in Ai
     double *Ax;
#ifdef SBA_CHOLMOD
     if (useCholmod)
       Ax = (double *)chA->x;   // values
     else
#endif
       Ax = A->x;               // values

     for (int i=0; i<(int)cols.size(); i++)
       {
         // column i entries
         map<int,Matrix<double,3,3>, less<int>, 
           aligned_allocator<Matrix<double,3,3> > > &col = cols[i];

         // do this for 3 columns
         for (int k=0; k<3; k++)
           {
             // iterate over the map
             if (col.size() > 0)
               {
                 // map iterator
                 map<int,Matrix<double,3,3>, less<int>, 
                   aligned_allocator<Matrix<double,3,3> > >::iterator it;

                 // iterate over block column entries
                 for (it = col.begin(); it != col.end(); it++)
                   {
                     Matrix<double,3,3> &m = it->second;
                     for (int j=0; j<3; j++)
                       Ax[colp++] = m(j,k);
                   }
               } 

             // add in diagonal entries
             Matrix<double,3,3> &m = diag[i]; // diagonal block
             for (int kk=0; kk<k+1; kk++)
               Ax[colp++] = m(kk,k);
             Ax[colp-1] *= diaginc; // increment diagonal for LM
           }
       }      

     // make symmetric from upper diagonal
     // this could be more efficient if AT were eliminated and AF were fixed
     // oops - chol_sol only needs upper diag
  }


  // uncompress the compressed A into a dense Eigen matrix
  void CSparse2d::uncompress(MatrixXd &m)
  {
    if (!A) return;
    m.setZero(csize,csize);
    
    int *Ap = A->p;             // column pointer
    int *Ai = A->i;             // row indices
    double *Ax = A->x;          // values;

    for (int i=0; i<csize; i++)
      {
        int rbeg = Ap[i];
        int rend = Ap[i+1];
        if (rend > rbeg)
          for (int j=rbeg; j<rend; j++)
            m(Ai[j],i) = Ax[j];
      }
  }

  // solve in place, returns RHS B
  bool CSparse2d::doChol()
  {
#ifdef SBA_CHOLMOD
    if (useCholmod)
      {
        cholmod_dense *x, b, *R, *R2;
        cholmod_factor *L ;
        double *Xx, *Rx, *bb;
        double one [2], minusone [2];
        one [0] = 1 ;
        one [1] = 0 ;
        minusone [0] = -1 ;
        minusone [1] = 0 ;

        //        cholmod_start (&Common) ;    // start it up ???
        cholmod_print_sparse (chA, (char *)"A", &Common) ; // print simple stats
        b.nrow = csize;
        b.ncol = 1;
        b.d = csize;                // leading dimension (???)
        b.nzmax = csize;
        b.xtype = CHOLMOD_REAL;
        b.dtype = CHOLMOD_DOUBLE;
        b.x = B.data();
        //cout << "CHOLMOD analyze..." << flush;
        L = cholmod_analyze (chA, &Common) ; // analyze 
        //cout << "factorize..." << flush;
        cholmod_factorize (chA, L, &Common) ; // factorize 
        //cout << "solve..." << flush;
        x = cholmod_solve (CHOLMOD_A, L, &b, &Common) ; // solve Ax=b
        //        cholmod_print_factor (L, (char *)"L", &Common) ;

        //cout << "refine" << endl;
        // one step of iterative refinement, cheap
	/* Ax=b was factorized and solved, R = B-A*X */
	R = cholmod_copy_dense (&b, &Common) ;
	cholmod_sdmult(chA, 0, minusone, one, x, R, &Common) ;
	/* R2 = A\(B-A*X) */
	R2 = cholmod_solve (CHOLMOD_A, L, R, &Common) ;
	/* compute X = X + A\(B-A*X) */
	Xx = (double *)x->x ;
	Rx = (double *)R2->x ;
	for (int i=0 ; i<csize ; i++)
	{
          Xx[i] = Xx[i] + Rx[i] ;
	}
	cholmod_free_dense (&R2, &Common) ;
	cholmod_free_dense (&R, &Common) ;

        bb = B.data();
        for (int i=0; i<csize; i++) // transfer answer
          *bb++ = *Xx++;
        cholmod_free_factor (&L, &Common) ; // free matrices 
        cholmod_free_dense (&x, &Common) ;
        cholmod_free_sparse(&chA, &Common);
        cholmod_finish (&Common) ;   // finish it ???

        return true;
      }
    else
#endif
      {

	// using order 0 here (natural order); 
	// may be better to use "1" for large problems (AMD)
	int order = 0;
	if (csize > 100) order = 1;
	bool ok = (bool)cs_cholsol(order,A,B.data()); // do the CSparse2d thang
	return ok;
      }
  }


  // 
  // block jacobian PCG
  // max iterations <iter>, ending toleranace <tol>
  //

  int 
  CSparse2d::doBPCG(int iters, double tol, int sba_iter)
  {
    int n = B.rows();
    VectorXd x;
    x.setZero(n);
    bool abstol = false;
    if (sba_iter > 0) abstol = true;
    int ret;
    ret = bpcg.doBPCG2(iters, tol, diag, cols, x, B, abstol);
    B = x;			// transfer result data
    return ret;
  }


#ifdef SBA_DSIF
  // solve in place, returns RHS B
  // PCG algorithm from SparseLib++, IML++
  int CSparse2d::doPCG(int iters)
  {
    // first convert sparse matrix to SparseLib++ format
    // do we need just upper triangular here???

    // add in lower triangular part
    cs *AT;
    AT = cs_transpose(A,1);
    cs_fkeep (AT, &dropdiag, NULL); // drop diagonal entries from AT 
    if (AF) cs_spfree(AF);      // free any previous structure
    AF = cs_add (A, AT, 1, 1);  // AF = A+AT
    cs_spfree (AT);

    // convert to SparseLib objects
    CompCol_Mat_double Ap(csize, csize, AF->nzmax, AF->x, AF->i, AF->p); 
    VECTOR_double b(B.data(),csize);   // Create rhs
    VECTOR_double x(csize, 0.0); // solution vectors

    // perform PCG
    int res;
    double tol = 1e-6;
    ICPreconditioner_double D(Ap); // Create diagonal preconditioner
    res = CG(Ap,x,b,D,iters,tol); // Solve system

    for (int i=0; i<csize; i++)
      B[i] = x[i];

    //    cout << "CG flag = " << res << endl;
    //    cout << "iterations performed: " << iters << endl;
    //    cout << "tolerance achieved  : " << tol << endl;
    //    cout << x << endl;

    return res;
  }
#endif


} // end namespace sba
