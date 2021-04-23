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
// Functions defined in various test setup files
//

#ifndef _SBA_SETUP_H_
#define _SBA_SETUP_H_

#include "sparse_bundle_adjustment/sba.h"
#include "sparse_bundle_adjustment/spa2d.h"
using namespace Eigen;
using namespace sba;
using namespace frame_common;

#include <iostream>
#include <fstream>
#include <vector>

using namespace std;

// elapsed time in microseconds
long long utime();
// set up spiral system
void 
spiral_setup(SysSBA &sba, CamParams &cpars, vector<Matrix<double,6,1>,Eigen::aligned_allocator<Matrix<double,6,1> > > &cps,
	     double s_near, double s_far,
	     double ptsize, double kfang, double initang, double cycles, 
	     double inoise, double pnoise, double qnoise);

void
sphere_setup(SysSBA &sba, CamParams &cpars, vector<Matrix<double,6,1>, Eigen::aligned_allocator<Matrix<double,6,1> > > &cps,
	     int ncams,		// number of cameras
	     int ncpts,		// number of new pts per camera
	     int nccs,		// number of camera connections per camera
             double inoise, double pnoise, double qnoise);


void
spa_spiral_setup(SysSPA &spa, bool use_cross_links,
                 vector<Matrix<double,6,1>, Eigen::aligned_allocator<Matrix<double,6,1> > > &cps,
                 Matrix<double,6,6> prec, Matrix<double,6,6> vprec,
                 Matrix<double,6,6> a10prec, Matrix<double,6,6> a15prec,
                 double kfang, double initang, double cycles, 
                 double pnoise, double qnoise, double snoise, // measurement noise (m,deg), scale noise (percent)
                 double mpnoise, double mqnoise); // initial displacement (m,deg)


void
spa2d_spiral_setup(SysSPA2d &spa, 
                 vector<Matrix<double,3,1>, Eigen::aligned_allocator<Matrix<double,3,1> > > &cps,
                 Matrix<double,3,3> prec, Matrix<double,3,3> vprec,
                 Matrix<double,3,3> a10prec, Matrix<double,3,3> a15prec,
                 double kfang, double initang, double cycles, 
                 double pnoise, double qnoise, double snoise, double dpnoise, double dqnoise);


#endif  // _VO_SETUP_H_
