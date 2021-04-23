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

#include "cholmod.h"
#include "sparse_bundle_adjustment/sba.h"
#include <time.h>
#define CPUTIME ((double) (clock ( )) / CLOCKS_PER_SEC)

int main (void)
{
    cholmod_sparse *A ;
    cholmod_dense *x, *b, *r ;
    cholmod_factor *L ;
    double one [2] = {1,0}, m1 [2] = {-1,0} ;	    /* basic scalars */
    cholmod_common c ;
    cholmod_start (&c) ;			    /* start CHOLMOD */
    A = cholmod_read_sparse (stdin, &c) ;	    /* read in a matrix */
    cholmod_print_sparse (A, "A", &c) ;		    /* print the matrix */
    if (A == NULL || A->stype == 0)		    /* A must be symmetric */
    {
	cholmod_free_sparse (&A, &c) ;
	cholmod_finish (&c) ;
	return (0) ;
    }
    b = cholmod_ones (A->nrow, 1, A->xtype, &c) ;   /* b = ones(n,1) */
    double t0 = CPUTIME;
    L = cholmod_analyze (A, &c) ;		    /* analyze */
    cholmod_factorize (A, L, &c) ;		    /* factorize */
    x = cholmod_solve (CHOLMOD_A, L, b, &c) ;	    /* solve Ax=b */
    double t1 = CPUTIME;
    printf("Time: %12.4f \n", t1-t0);
    r = cholmod_copy_dense (b, &c) ;		    /* r = b */
    cholmod_sdmult (A, 0, m1, one, x, r, &c) ;	    /* r = r-Ax */
    printf ("norm(b-Ax) %8.1e\n",
	    cholmod_norm_dense (r, 0, &c)) ;	    /* print norm(r) */
    cholmod_free_factor (&L, &c) ;		    /* free matrices */
    cholmod_free_sparse (&A, &c) ;
    cholmod_free_dense (&r, &c) ;
    cholmod_free_dense (&x, &c) ;
    cholmod_free_dense (&b, &c) ;
    cholmod_finish (&c) ;			    /* finish CHOLMOD */
    return (0) ;
}
