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
// Visual Odometry classes and functions
//

#include "sparse_bundle_adjustment/sba.h"
#include <chrono>

using namespace Eigen;
using namespace std;

//#define DEBUG

// elapsed time in microseconds
static long long utime()
{
  auto duration = std::chrono::system_clock::now().time_since_epoch();
  return std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
}


// some LAPACK Cholesky routines
#ifdef __cplusplus
extern "C" {
#endif

#define F77_FUNC(func)    func ## _
/* Cholesky decomposition, linear system solution and matrix inversion */
extern int F77_FUNC(dpotf2)(char *uplo, int *n, double *a, int *lda, int *info); /* unblocked Cholesky */
extern int F77_FUNC(dpotrf)(char *uplo, int *n, double *a, int *lda, int *info); /* block version of dpotf2 */
extern int F77_FUNC(dpotrs)(char *uplo, int *n, int *nrhs, double *a, int *lda, double *b, int *ldb, int *info);
extern int F77_FUNC(dpotri)(char *uplo, int *n, double *a, int *lda, int *info);

#ifdef __cplusplus
}
#endif



namespace sba
{
  // SBA system functions
  
  // Adds a node to the system. 
  // \return the index of the node added.
  int SysSBA::addNode(Eigen::Matrix<double,4,1> &trans, 
                      Eigen::Quaternion<double> &qrot,
                      const fc::CamParams &cp,
                      bool isFixed)
  {
    Node nd;
    nd.trans = trans;
    nd.qrot = qrot;
    nd.isFixed = isFixed;
    nd.setKcam(cp); // set up node2image projection
    nd.setTransform(); // set up world2node transform
    nd.setDr(true); // set rotational derivatives
    nd.setProjection();
    // Should this be local or global?
    nd.normRot();//Local();
    nodes.push_back(nd);
    return nodes.size()-1;
  }

  // Adds a point to the system.
  // \return the index of the point added.
  int SysSBA::addPoint(Point p)
  {
    tracks.push_back(Track(p));
    return tracks.size()-1;
  }
  

  // Add a projection between point and camera, in setting up the system;
  // <ci> is camera/node index, <pi> point index, <q> is image coordinates.
  // Stereo is whether the point is stereo or not (true is stereo, false is 
  // monocular).
  bool SysSBA::addProj(int ci, int pi, Eigen::Vector3d &q, bool stereo)
  {
    // NOTE: should check bounds on ci, pi, and prjs
    // get track
    
    if (tracks[pi].projections.count(ci) > 0)
    {
      if (tracks[pi].projections[ci].kp == q)
        return true;
      return false;
    }
    tracks[pi].projections[ci] = Proj(ci, q, stereo);

#if 0
    /// NOTE
    Eigen::Matrix3d covar;
    covar.setIdentity();
    covar(0,0) = 0.4;
    covar(1,1) = 0.4;
    covar(2,2) = 4.0;
    tracks[pi].projections[ci].setCovariance(covar);
#endif

    return true;
  }

  // Add a projection between point and camera, in setting up the system;
  // <ci> is camera/node index, <pi> point index, <q> is image coordinates.
  // This function explicitly sets up a monocular projection. If creating 
  // projections from existing projections, use addProj().
  bool SysSBA::addMonoProj(int ci, int pi, Eigen::Vector2d &q)
  {
    if (tracks[pi].projections.count(ci) > 0)
    {
      if (tracks[pi].projections[ci].kp.head(2) == q)
        return true;
      return false;
    }
    tracks[pi].projections[ci] = Proj(ci, q);
    return true;
  }
  
  // Add a projection between point and camera, in setting up the system;
  // <ci> is camera/node index, <pi> point index, <q> is image coordinates.
  // This function explicitly sets up a stereo projection.
  bool SysSBA::addStereoProj(int ci, int pi, Eigen::Vector3d &q)
  {
    if (tracks[pi].projections.count(ci) > 0)
    {
      if (tracks[pi].projections[ci].kp == q)
        return true;
      return false;
    }
    tracks[pi].projections[ci] = Proj(ci, q, true);

#if 0
    /// NOTE
    Eigen::Matrix3d covar;
    covar.setIdentity();
    covar(0,0) = 0.4;
    covar(1,1) = 0.4;
    covar(2,2) = 4.0;
    tracks[pi].projections[ci].setCovariance(covar);
#endif

    return true;
  }
  
  // Sets the covariance matrix of a projection.
  void SysSBA::setProjCovariance(int ci, int pi, Eigen::Matrix3d &covar)
  {
    // TODO Check if the projection exists instead.
    tracks[pi].projections[ci].setCovariance(covar);
  }
  
  // Add a point-plane match, forward and backward.
  void SysSBA::addPointPlaneMatch(int ci0, int pi0, Eigen::Vector3d normal0, int ci1, int pi1, Eigen::Vector3d normal1)
  {
    Point pt0 = tracks[pi0].point;
    Point pt1 = tracks[pi1].point;
    
    // works best with single constraint
#if 1
    // Forward: point 0 into camera 1.
    Vector3d proj_forward;
    proj_forward = tracks[pi1].projections[ci1].kp;
    //nodes[ci1].projectStereo(pt0, proj_forward);
    addStereoProj(ci1, pi0, proj_forward);
    
    Proj &forward_proj = tracks[pi0].projections[ci1];
    forward_proj.pointPlane = true;
    forward_proj.plane_point = pt1.head<3>();
    forward_proj.plane_local_normal = normal1;
    forward_proj.plane_point_index = pi1;
    forward_proj.plane_node_index = ci0;
#endif
    
#if 0
 // Peter: to avoid removeFrame() removing pi1 because it only has a single projection (according to projections of pi1)
   // Note that if we do point to plane projections both directions this should not be done
   Vector3d proj_fake;
   proj_fake = tracks[pi0].projections[ci0].kp;
   addStereoProj(ci0, pi1, proj_fake);
   Proj &fake_proj = tracks[pi1].projections[ci0];
   fake_proj.isValid = false;
#endif
    
#if 0
    // Backward: point 1 into camera 0. 
    Vector3d proj_backward;
    //nodes[ci0].projectStereo(pt1, proj_backward);
    proj_backward = tracks[pi0].projections[ci0].kp;
    addStereoProj(ci0, pi1, proj_backward);
    
    Proj &backward_proj = tracks[pi1].projections[ci0];
    backward_proj.pointPlane = true;
    backward_proj.plane_point = pt0.head<3>();
    backward_proj.plane_local_normal = normal0;
    backward_proj.plane_point_index = pi0;
    backward_proj.plane_node_index = ci1;
#endif
  }

  // Update the normals for point-plane matches.
  void SysSBA::updateNormals()
  {
    for(size_t i=0; i<tracks.size(); i++)
      {
        ProjMap &prjs = tracks[i].projections;
        if (prjs.size() == 0) continue;

        for(ProjMap::iterator itr = prjs.begin(); itr != prjs.end(); itr++)
          {
            Proj &prj = itr->second;      
            if (!prj.pointPlane || !prj.isValid) continue;
            
            prj.plane_point = tracks[prj.plane_point_index].point.head<3>();
            
            // Rotation between nodes into the projection's image plane
            Quaterniond qrot = nodes[prj.ndi].qrot;
            // Vector4d trans;
            
            // transformN2N(trans, qrot, nodes[prj.plane_node_index], nodes[itr->first]);
            
            prj.plane_normal = qrot.toRotationMatrix() * prj.plane_local_normal;
            //printf("Global normal: %f %f %f\n", prj.plane_normal.x(), prj.plane_normal.y(), prj.plane_normal.z()); 
            
            // Update projections
            //nodes[prj.ndi].projectStereo(tracks[prj.plane_point_index].point, prj.kp);
            Point &pt0 = tracks[i].point;
            Vector3d &plane_point = prj.plane_point;
            Vector3d &plane_normal = prj.plane_normal;
            Vector3d w = pt0.head<3>()-plane_point;
            Vector3d projpt = pt0.head<3>() - (w.dot(plane_normal))*plane_normal;
            
          }
      }
  }

  // help function
  int SysSBA::countProjs()
  {
    int tot = 0;
    for(size_t i=0; i<tracks.size(); i++)
      {
        ProjMap &prjs = tracks[i].projections;
        tot += prjs.size();
      }
    return tot;
  }


  // error measure, squared
  // assumes node projection matrices have already been calculated
  double SysSBA::calcCost()
  {
    double cost = 0.0;
    for(size_t i=0; i<tracks.size(); i++)
      {
        ProjMap &prjs = tracks[i].projections;
        if (prjs.size() == 0) continue;
        for(ProjMap::iterator itr = prjs.begin(); itr != prjs.end(); itr++)
          {
            Proj &prj = itr->second;      
            if (!prj.isValid) continue;
            double err = prj.calcErr(nodes[prj.ndi],tracks[i].point,huber);
            cost += err;
          }
      }
    return cost;
  }

  // error measure, squared
  // assumes node projection matrices have already been calculated
  // doesn't count projections with errors higher than <dist>
  double SysSBA::calcCost(double dist)
  {
    double cost = 0.0;
    dist = dist*dist;           // square it
    
    for(size_t i=0; i<tracks.size(); i++)
      {
        ProjMap &prjs = tracks[i].projections;
        if (prjs.size() == 0) continue;
        for(ProjMap::iterator itr = prjs.begin(); itr != prjs.end(); itr++)
          {
            Proj &prj = itr->second;      
            if (!prj.isValid) continue;
            double err = prj.calcErr(nodes[prj.ndi],tracks[i].point);
            if (err < dist)
              cost += err;
          }
      }

    return cost;
  }


  // error measure, RMS
  // assumes node projection matrices have already been calculated
  // doesn't count projections with errors higher than <dist>
  double SysSBA::calcRMSCost(double dist)
  {
    double cost = 0.0;
    dist = dist*dist;           // square it
    int nprjs = 0;
    
    for(size_t i=0; i<tracks.size(); i++)
      {
        ProjMap &prjs = tracks[i].projections;
        if (prjs.size() == 0) continue;
        for(ProjMap::iterator itr = prjs.begin(); itr != prjs.end(); itr++)
          {
            Proj &prj = itr->second;      
            if (!prj.isValid) continue;
            double err = prj.calcErr(nodes[prj.ndi],tracks[i].point,huber);
            if (err < dist)
            {
              cost += err;
              nprjs++;
            }
          }
      }
      
    return sqrt(cost/(double)nprjs);
  }


  // Average reprojection error
  // assumes projection errors already calculated
  double SysSBA::calcAvgError()
  {
    double cost = 0.0;
    int nprjs = 0;
    
    for(size_t i=0; i<tracks.size(); i++)
      {
        ProjMap &prjs = tracks[i].projections;
        if (prjs.size() == 0) continue;
        for(ProjMap::iterator itr = prjs.begin(); itr != prjs.end(); itr++)
          {
            Proj &prj = itr->second;      
            if (!prj.isValid) continue;
            prj.calcErr(nodes[prj.ndi],tracks[i].point,huber);
            cost += prj.getErrNorm();
            nprjs++;
          }
      }
    
    return cost/(double)nprjs;
  }


  // number of points that have negative Z
  int SysSBA::numBadPoints()
  {
    int count = 0;
    
    for(size_t i=0; i<tracks.size(); i++)
      {
        ProjMap &prjs = tracks[i].projections;
        if (prjs.size() == 0) continue;
        for(ProjMap::iterator itr = prjs.begin(); itr != prjs.end(); itr++)
          {
            Proj &prj = itr->second;      
            if (!prj.isValid) continue;
            prj.calcErr(nodes[prj.ndi],tracks[i].point);
            if (prj.err[0] == 0.0 && prj.err[1] == 0.0 && prj.err[2] == 0.0)
              count++;
          }
          
          /*if (tracks[i].point.z() < 0)
            count++;*/
      }

    return count;
  }


  // 
  // find the number of outlying projections
  //
  int SysSBA::countBad(double dist)
  {
    dist = dist*dist;           // square it
#ifdef HUBER
    // convert to Huber distance
    double b2 = HUBER*HUBER;
    dist = 2*b2*(sqrt(1+dist/b2)-1);
#endif
    int n=0;
    for (int i=0; i<(int)tracks.size(); i++)
      {
        ProjMap &prjs = tracks[i].projections;
        if (prjs.size() == 0) continue;
        for(ProjMap::iterator itr = prjs.begin(); itr != prjs.end(); itr++)
          {
            Proj &prj = itr->second;      
            if (!prj.isValid) continue;
            if (prj.getErrSquaredNorm() >= dist)
              n++;
          }
      }

    return n;
  }


  // 
  // remove outlying projections
  //
  int SysSBA::removeBad(double dist)
  {
    dist = dist*dist;           // square it
    int nbad = 0;
    
    for (int i=0; i<(int)tracks.size(); i++)
      {
        ProjMap &prjs = tracks[i].projections;
        if (prjs.size() == 0) continue;
        for(ProjMap::iterator itr = prjs.begin(); itr != prjs.end(); itr++)
          {
            Proj &prj = itr->second;      
            if (!prj.isValid) continue; // This line was added. Do we care?
            if (prj.getErrSquaredNorm() >= dist)
            {
              prj.isValid = false;
              nbad++;
            }
          }
      }
    return nbad;
  }

  //
  // reduce tracks by eliminating bad projections and single tracks
  //
  int SysSBA::reduceTracks()
  {
    int ret = 0;
    for (int i=0; i<(int)tracks.size(); i++)
    {
      ProjMap &prjs = tracks[i].projections;
      int ngood = 0;
      // increment is IN for loop, since after erasing an element, the old iterator is invalid
      // => incrementing from old iterator results in undefined behgaviour!
      for(ProjMap::iterator itr = prjs.begin(); itr != prjs.end(); /*no increment here!!!*/)
      {
        Proj &prj = itr->second;
        if (prj.isValid)
        {
          ngood++;
          ++itr;
        }
        else
        {
          prjs.erase(itr++); // Erase bad projections
        }
      }
      // Clear out tracks with too few good projections.
      if (ngood < 2)
      {
        prjs.clear();
        ret++;
      }
    }
    return ret; // Returns the number of tracks cleared.
  }

  // print some stats about the system
  void SysSBA::printStats()
  {
    int ncams = nodes.size();
    vector<map<int,int>, Eigen::aligned_allocator<map<int,int> > > conns; // connections between cameras - key is camera, val is count
    conns.resize(ncams);
    VectorXi dcnt(ncams);
    dcnt.setZero(ncams);

    int nbad = 0;
    int n2 = 0;
    int n1 = 0;
    int n0 = 0;
    int npts = tracks.size();

    int nprjs = 0;
    int nhs = 0;                // number of Hessian elements

    if (tracks.size() > 0)    // have a stereo system, maybe
      {
        for (int i=0; i<npts; i++)
          {
            int s = (int)tracks[i].projections.size();
            nprjs += s;
            nhs += s*(s+1)/2;
          }

        cout << "[SBAsys] Cameras: " << ncams << "   Points: " << npts 
             << "   Projections: " << nprjs << "  Hessian elements: " << nhs << endl;
        cout << "[SBAsys] Average " << (double)nprjs/(double)npts 
             << " projections per track and " << (double)nprjs/(double)ncams 
             << " projections per camera" << endl;

        cout << "[SBAsys] Checking camera order in projections..." << flush;
        for (int i=0; i<npts; i++)
          {
            // track
            ProjMap &prjs = tracks[i].projections;
            int n = 0;
            int j = 0;
            
            for(ProjMap::iterator itr = prjs.begin(); itr != prjs.end(); itr++)
              {
                j++;
                Proj &prj = itr->second;
                // projection
                int cami = prj.ndi;
                if (prj.isValid)
                  dcnt(cami)++;

                // check validity
                if (!prj.isValid) 
                  {
                    nbad++;
                    continue;
                  }
                n++;

                // add to camera connection matrix
                if (j < (int)prjs.size()-1)
                  {
                    map<int,int> &pm = conns[cami];
                    map<int,int>::iterator it;
                    for(ProjMap::iterator itr2 = itr; itr2 != prjs.end(); itr2++)
                      {
                        Proj &prj2 = itr2->second;
                        int cami2 = prj2.ndi;
                        map<int,int> &pm2 = conns[cami2];
                        it = pm.find(cami2);
                        if (it == pm.end()) // not here yet
                          {
                            pm[cami2] = 1; // create entry
                            pm2[cami] = 1;
                          }
                        else
                          {
                            it->second++; // increment
                            pm2.find(cami)->second++;
                          }
                      }
                  }
              }

            // stats on tracks
            if (n == 2)
              n2++;
            if (n == 1)
              n1++;
            if (n == 0)
              n0++;
          }
      }

    cout << "ok" << endl;

    // projection stats
    cout << "[SBAsys] Number of invalid projections: " << nbad << endl;
    cout << "[SBAsys] Number of null tracks: " << n0 << "  Number of single tracks: " << n1 << "   Number of double tracks: " << n2 << endl;

    // connections
    int dnz = 0;
    int dn5 = 0;
    for (int i=0; i<ncams; i++)
      {
        if (dcnt(i) == 0) dnz++;
        if (dcnt(i) < 5) dn5++;
      }
    cout << "[SBAsys] Number of disconnected cameras: " << dnz << endl;
    cout << "[SBAsys] Number of cameras with <5 projs: " << dn5 << endl;


#if 1
    // connection stats
    int nconns = 0;
    int ntot = 0;
    int nmin = 1000000;
    int nmax = 0;
    int nc5 = 0;
    int nc10 = 0;
    int nc20 = 0;
    int nc50 = 0;
    n0 = 0;
    n1 = 1;
    for (int i=0; i<ncams; i++)
      {
        int nc = conns[i].size();
        nconns += nc;
        if (nc == 0) n0++;
        if (nc == 1) n1++;
        map<int,int>::iterator it;      
        for (it = conns[i].begin(); it != conns[i].end(); it++)
          {
            int np = (*it).second;
            ntot += np;
            if (np < nmin) nmin = np;
            if (np > nmax) nmax = np;
            if (np < 5) nc5++;
            if (np < 10) nc10++;
            if (np < 20) nc20++;
            if (np < 50) nc50++;
          }
      }

    cout << "[SBAsys] Number of camera connections: " << nconns/2 << "  which is " 
         << (double)nconns/(double)ncams << " connections per camera and " 
         << (nconns+ncams)*100.0/(ncams*ncams) << " percent matrix fill" << endl;
    cout << "[SBAsys] Min connection points: " << nmin << "  Max connection points: " 
         << nmax << "  Average connection points: " << (double)ntot/(double)nconns << endl;
    cout << "[SBAsys] Total connection projections: " << ntot << endl;
    cout << "[SBAsys] Connections with less than 5 points: " << nc5/2 << "   10 pts: " << nc10/2
         << "   20 pts: " << nc20/2 << "   50 pts: " << nc50/2 << endl;
#endif

  }

  vector<map<int,int> > SysSBA::generateConns_()
  {
    int ncams = nodes.size();
    vector<map<int,int> > conns; // connections between cameras - key is camera, val is count
    conns.resize(ncams);

    // reset bit maps
    connMat.resize(ncams);
    for (int i=0; i<ncams; i++)
      {
        vector<bool> &bm = connMat[i];
        bm.assign(ncams,false);
      }

    // set up sparse connectivity matrix
    for (int i=0; i<(int)tracks.size(); i++)
      {
        int j = 0;
        ProjMap &prjs = tracks[i].projections;
        for(ProjMap::iterator itr = prjs.begin(); itr != prjs.end(); itr++)
          {
            Proj &prj = itr->second;
            j++; 
            int cami = prj.ndi;

            // check validity
            if (!prj.isValid) 
              continue;

            // add to camera connection matrix
            if (j < (int)prjs.size()-1)
              {
                map<int,int> &pm = conns[cami];
                map<int,int>::iterator it;
                for(ProjMap::iterator itr2 = itr; itr2 != prjs.end(); itr2++)
                  {
                    Proj &prj2 = itr2->second;
                    int cami2 = prj2.ndi;
                    map<int,int> &pm2 = conns[cami2];
                    it = pm.find(cami2);
                    if (it == pm.end()) // not here yet
                      {
                        pm[cami2] = 1; // create entry
                        pm2[cami] = 1;
                      }
                    else
                      {
                        it->second++; // increment
                        pm2.find(cami)->second++;
                      }
                  }
              }
          }
      }
      
    return conns;
  }


  // set the connectivity matrix based on a minimum number of points
  // greedy algorithm, heads with lowest and preserves connectivity
  void SysSBA::setConnMat(int minpts)
  {
    // data structures
    int ncams = nodes.size();
    // connections between cameras - key is camera, val is count
    vector<map<int,int> > conns = generateConns_();

    // get ordered list of connections
    multimap<int,pair<int,int> > weakcs;
    for (int i=0; i<ncams; i++)
      {
        map<int,int> cs = conns[i];
        map<int,int>::iterator it;
        for (it = cs.begin(); it != cs.end(); it++)
          {
            if (it->second < minpts && it->first > i) // upper triangle
              weakcs.insert(pair<int,pair<int,int> >(it->second, pair<int,int>(i,it->first)));
          }
      }
    
    cout << "[SetConnMat] Found " << weakcs.size() << " connections with < " 
         << minpts << " points" << endl;

    // greedily delete connections that don't disconnect the graph
    int n = 0;
    multimap<int,pair<int,int> >::iterator it;    
    for (it = weakcs.begin(); it != weakcs.end(); it++)
      {
        int c0 = it->second.first;
        int c1 = it->second.second;
        if (conns[c0].size() > 1 && conns[c1].size() > 1)
          {
            n++;
            conns[c0].erase(conns[c0].find(c1)); // erase entry
            conns[c1].erase(conns[c1].find(c0)); // erase entry
            // set correct bits
            connMat[c0][c1] = true;
            connMat[c1][c0] = true;
          }
      }

    cout << "[SetConnMat] Erased " << n << " connections" << endl;

  }


  // set the connectivity matrix based on a spanning tree
  // greedy algorithm, strings together best matches first
  void SysSBA::setConnMatReduced(int maxconns)
  {
    // data structures
    int ncams = nodes.size();
    // connections between cameras - key is camera, val is count
    vector<map<int,int> > conns = generateConns_();
    
    // get ordered list of connections
    multimap<int,pair<int,int> > weakcs;
    for (int i=0; i<ncams; i++)
      {
        map<int,int> cs = conns[i];
        map<int,int>::iterator it;
        for (it = cs.begin(); it != cs.end(); it++)
          {
            if (it->first > i) // upper triangle, order by biggest matches first
              weakcs.insert(pair<int,pair<int,int> >(-it->second, pair<int,int>(i,it->first)));
          }
       }
    
    // greedily add connections to build a spanning graph
    int n = 0;
    vector<int> found;
    found.assign(ncams,0);
    multimap<int,pair<int,int> >::iterator it;    
    for (it = weakcs.begin(); it != weakcs.end(); it++)
      {
        int c0 = it->second.first;
        int c1 = it->second.second;
        if (found[c0] < maxconns || found[c1] < maxconns)
          {
            n++;
            // set correct bits
            found[c0]++;
            found[c1]++;
            connMat[c0][c1] = false; // assign this connection
            connMat[c1][c0] = false;
          }
      }

    cout << "[SetConnMat] Found " << n << " connections in spanning tree" << endl;
  }


  // helper fn
  // split into random tracks
  void
  SysSBA::tsplit(int tri, int len)
  {
    ProjMap prjs = tracks[tri].projections;
    tracks[tri].projections.clear();

    // first reset current track
    int i=0;
    if ((int)prjs.size() == len+1) len = len+1; // get rid of single tracks
    
    // Goes through existing projections and adds them back to the track up
    // until len elements are added back.
    while (prjs.size() > 0 && i < len)
      {
        // Pick a random projection to add back.
        ProjMap::iterator randomitr = prjs.begin();
        std::advance( randomitr, rand() % prjs.size());
        Proj &prj = randomitr->second;

        // Add it back to the original track.
        addProj(prj.ndi, tri, prj.kp, prj.stereo);
        prjs.erase(randomitr);
        i++;
      }

    // Creates new tracks for remaining elements in the track.
    int pti = tracks.size();
    while (prjs.size() > 0)
      {
        i = 0;
        if ((int)prjs.size() == len+1) len = len+1; // get rid of single tracks
        while (prjs.size() > 0 && i < len)
          {
            // Pick a random projection to add to a new track.
            ProjMap::iterator randomitr = prjs.begin();
            std::advance( randomitr, rand() % prjs.size());
            Proj &prj = randomitr->second;
            
            // Add it to the new track and erase it from the list of projections
            // remaining.
            addProj(prj.ndi, pti, prj.kp, prj.stereo);
            prjs.erase(randomitr);
            i++;
          }
        tracks[pti].point = tracks[tri].point;
        pti++;
      }
  }


  // get rid of long tracks
  // currently not very smart, just orders them by length and gets rid of top ones
  // or else splits them
  int SysSBA::reduceLongTracks(double len)
  {
    int ilen = len;
    // data structures
    int npts = tracks.size();

#if 1  // this algorithm splits tracks
    // algorithm: for a long track, break it into enough pieces to get it under 
    // the required length.  Randomzed point picking.
    srand(time(NULL));
    int nn = 0;
    for (int i=0; i<npts; i++)
      {
        if ((int)tracks[i].projections.size() > ilen)
          {
            // make new set of proj's
            nn++;
            int ts = tracks[i].projections.size()+1;
            int tn = ts/ilen;
            tsplit(i,ts/tn);
          }
      }
    
    return nn;

#else  // this throws them out
  
    // Has not been changed from old API. :(
    
    multimap<int,int> ordps;    // ordered tracks
    
    // order them, largest first
    for (int i=0; i<npts; i++)
      if ((int)tracks[i].size() > ilen)
        ordps.insert(pair<int,int>(-(int)tracks[i].size(), i));
    
    // remove long tracks
    //    int nn = npts*pct;
    int nn = ordps.size();
    map<int,int>::iterator it = ordps.begin();
    vector<int> rem;
    for (int i=0; i<nn; i++, it++)
      rem.push_back(it->second);
    sort(rem.begin(),rem.end()); // sort into ascending order
    cout << "Finished finding " << rem.size() << " tracks" << endl;

    std::vector<Point, Eigen::aligned_allocator<Point> > pts;
    std::vector< std::vector<MonoProj, Eigen::aligned_allocator<MonoProj> >, 
      Eigen::aligned_allocator<std::vector<MonoProj, Eigen::aligned_allocator<MonoProj> > > > trs;

    // delete elements into new vectors
    int n = 0;                  // index into rem()
    int ii = 0;
    for (int i=0; i<npts; i++)
      {
        if (rem.size()>n && i == rem[n])        // skip this element
          {
            n++;
            continue;
          }
        pts.push_back(points[i]);
        vector<MonoProj, Eigen::aligned_allocator<MonoProj> > &prjs = tracks[i];
        /*for (int j=0; j<(int)prjs.size(); j++)
          prjs[j].pti = ii; */
        trs.push_back(tracks[i]);
        ii++;
      }

    // transfer vectors
    points.resize(pts.size());
    tracks.resize(pts.size());
    points = pts;
    tracks = trs;
#endif
    

    return nn;
  }


  // delete any track that doesn't reduce the connection number between two nodes below
  //   a minimum
  // greedy algorithm, heads with largest (smallest???) tracks
  int SysSBA::remExcessTracks(int minpts)
  {
    // data structures
    //int ncams = nodes.size();
    int npts = tracks.size();
    // connections between cameras - key is camera, val is count
    vector<map<int,int> > conns = generateConns_();
    
    // get ordered list of tracks
    multimap<int,int> ordtrs;
    for (int i=0; i<npts; i++)
      ordtrs.insert(pair<int,int>((int)tracks[i].projections.size(),i));
    vector<int> remtrs;         // tracks to delete

    // greedily delete tracks that don't bring connection size below a minimum
    multimap<int,int>::reverse_iterator it;    
    for (it = ordtrs.rbegin(); it != ordtrs.rend(); it++)
      {
        int tri = it->second;
        ProjMap &prjs = tracks[tri].projections;
        bool isgood = true;
        for(ProjMap::iterator itr = prjs.begin(); itr != prjs.end(); itr++)
          {
            Proj &prj = itr->second;
          
            int c0 = prj.ndi;
            for(ProjMap::iterator itr2 = itr; itr2 != prjs.end(); itr2++)
              {
                Proj &prj2 = itr2->second;
                int c1 = prj2.ndi;
                if (conns[c0].find(c1)->second <= minpts) // can't reduce this connection
                  {
                    isgood = false;
                    break;
                  }
              }
            if (!isgood) break;
          }

        if (isgood)             // found a deletable track, change connection values
          {
            for(ProjMap::iterator itr = prjs.begin(); itr != prjs.end(); itr++)
              {
                Proj &prj = itr->second;
                int c0 = prj.ndi;
                for(ProjMap::iterator itr2 = itr; itr2 != prjs.end(); itr2++)
                  {
                    Proj &prj2 = itr2->second;
                    int c1 = prj2.ndi;
                    conns[c0].find(c1)->second--;
                    conns[c1].find(c0)->second--;
                  }
              }
            remtrs.push_back(tri); // save for deletion
          }
      }

    int hms = 0;
    for (int i=0; i<(int)remtrs.size(); i++)
      {
        int s = tracks[remtrs[i]].projections.size();
        hms += s*(s+1)/2;
      }

    cout << "[RemExcessTracks] Can erase " << remtrs.size() << " tracks with " << hms << " H entries" << endl;
    
    std::sort(remtrs.begin(),remtrs.end()); // sort into ascending order

    std::vector<Track, Eigen::aligned_allocator<Track> > trs;

    // delete elements into new vectors
    int n = 0;                  // index into rem()
    int ii = 0;
    for (int i=0; i<npts; i++)
      {
        if ((int)remtrs.size()>n && i == remtrs[n]) // skip this element
          {
            n++;
            continue;
          }
        // vector<MonoProj, Eigen::aligned_allocator<MonoProj> > &prjs = tracks[i];
        /*for (int j=0; j<(int)prjs.size(); j++)
          prjs[j].pti = ii; */
        trs.push_back(tracks[i]);
        ii++;
      }

    cout << "[RemExcessTracks] Erased " << n << " tracks" << endl;

    // transfer vectors
    tracks.resize(trs.size());
    tracks = trs;

    return remtrs.size();
  }
  
  // Set up linear system, from Engels and Nister 2006, Table 1, steps 3 and 4
  // This is a relatively compact version of the algorithm! 
  // Assumes camera transforms and derivatives have already been computed,
  // but not projection Jacobians

void SysSBA::setupSys(double sLambda)
  {
    // set matrix sizes and clear (step 3)
    int nFree = nodes.size() - nFixed;
    A.setZero(6*nFree,6*nFree);
    B.setZero(6*nFree);
    VectorXi dcnt(nFree);
    dcnt.setZero(nFree);

    // lambda augmentation
    double lam = 1.0 + sLambda;

    // loop over tracks (step 4)
    for(size_t pi=0; pi<tracks.size(); pi++)
      {
        ProjMap &prjs = tracks[pi].projections;
        if (prjs.size() < 1) continue; // this catches some problems with bad tracks

	// Jacobian product storage
	if (prjs.size() > jps.size())
	  jps.resize(prjs.size());

	// local storage
        Matrix3d Hpp;
        Hpp.setZero();            // zero it out
        Vector3d bp;
        bp.setZero();
      
        // "compute derivates" of step 4
        // assume error has already been calculated in the cost function
	int ii=0;
        for(ProjMap::iterator itr = prjs.begin(); itr != prjs.end(); itr++, ii++)
          {
            Proj &prj = itr->second;
            if (!prj.isValid) continue;
            int ci = (prj.ndi - nFixed) * 6; // index of camera params (6DOF)
                                             // NOTE: assumes fixed cams are at beginning
            prj.setJacobians(nodes[prj.ndi],tracks[pi].point,&jps[ii]); // calculate derivatives
            Hpp += prj.jp->Hpp; // add in JpT*Jp
            bp  -= prj.jp->Bp; // subtract JcT*f from bp; compute transpose twice???

            if (!nodes[prj.ndi].isFixed)  // if not a fixed camera, do more
              {
                dcnt(prj.ndi - nFixed)++;
                // NOTE: A is symmetric, only need the upper/lower triangular part
                A.block<6,6>(ci,ci) += prj.jp->Hcc; // add JcT*Jc to A; diagonal augmented????
                B.block<6,1>(ci,0) -= prj.jp->JcTE;
              }
          }

        // Augment Hpp, invert it and save Hpp' * bp
        // Hmm, shouldn't need this (augmented diagonal at end), 
        //   but seems to work better...
        Hpp.diagonal() *= lam;
        Matrix3d Hppi = Hpp.inverse(); // Which inverse should we use???? Note Hpp is symmetric
        Vector3d &tp = tps[pi];
        tp = Hppi * bp;           

        // "outer product of track" in Step 4
        for(ProjMap::iterator itr = prjs.begin(); itr != prjs.end(); itr++)
          {
            Proj &prj = itr->second;
            if (!prj.isValid) continue;
            if (nodes[prj.ndi].isFixed) continue; // skip fixed cameras
            int ci = (prj.ndi - nFixed) * 6; // index of camera params (6DOF)
                                             // NOTE: assumes fixed cams are at beginning
            B.block<6,1>(ci,0) -= prj.jp->Hpc.transpose() * tp; // Hpc * tp subtracted from B
            prj.Tpc = prj.jp->Hpc.transpose() * Hppi;

            // iterate over nodes left on the track, plus yourself
            for(ProjMap::iterator itr2 = itr; itr2 != prjs.end(); itr2++)
              {
                Proj &prj2 = itr2->second;
                if (!prj2.isValid) continue;
                if (nodes[prj2.ndi].isFixed) continue; // skip fixed cameras
                int ci2 = (prj2.ndi - nFixed) * 6; // index of camera params (6DOF)
                                               // NOTE: assumes fixed cams are at beginning
                // NOTE: this only does upper triangular part
                A.block<6,6>(ci,ci2) -= prj.Tpc * prj2.jp->Hpc; // Tpc * Hpc2 subtracted from A(c,c2)
                // lower triangular part - this can be dropped for CSparse, uses ~30% of setup time
                if (ci != ci2)
                  A.block<6,6>(ci2,ci) = A.block<6,6>(ci,ci2).transpose();
              }
          } // finish outer product

      } // finish track
    
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
  

  // Set up linear system, from Engels and Nister 2006, Table 1, steps 3 and 4
  // This is a relatively compact version of the algorithm! 
  // Assumes camera transforms and derivatives have already been computed,
  // but not projection Jacobians
  // CSparse version

  void SysSBA::setupSparseSys(double sLambda, int iter, int sparseType)
  {
    // set matrix sizes and clear (step 3)
    int nFree = nodes.size() - nFixed;
    if (nFree < 0) nFree = 0;

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

    // use connection matrix?
    bool useConnMat = connMat.size() > 0;
    int nskip = 0;

    // loop over tracks (step 4)
    for(size_t pi=0; pi<tracks.size(); pi++)
      {
        ProjMap &prjs = tracks[pi].projections;
        if (prjs.size() < 1) continue; // this catches some problems with bad tracks

	// set up vector storage of Jacobian products
	if (prjs.size() > jps.size())
	  jps.resize(prjs.size());

	// local storage
        Matrix3d Hpp;
        Hpp.setZero();            // zero it out
        Vector3d bp;
        bp.setZero();
      
        // "compute derivates" of step 4
        // assume error has already been calculated in the cost function
	int ii=0;
        for(ProjMap::iterator itr = prjs.begin(); itr != prjs.end(); itr++, ii++)
          {
            Proj &prj = itr->second;
            if (!prj.isValid) continue;
            int ni = prj.ndi - nFixed;
            int ci = ni * 6;    // index of camera params (6DOF)
                                             // NOTE: assumes fixed cams are at beginning
            prj.setJacobians(nodes[prj.ndi],tracks[pi].point,&jps[ii]); // calculate derivatives
            Hpp += prj.jp->Hpp; // add in JpT*Jp
            bp  -= prj.jp->Bp; // subtract JcT*f from bp; compute transpose twice???

            if (!nodes[prj.ndi].isFixed)  // if not a fixed camera, do more
              {
                dcnt(prj.ndi - nFixed)++;
                // NOTE: A is symmetric, only need the upper/lower triangular part
                //                jctjc.diagonal() *= lam;  // now done at end
                csp.addDiagBlock(prj.jp->Hcc,ni);
                csp.B.block<6,1>(ci,0) -= prj.jp->JcTE;
              }
          }

        // Augment Hpp, invert it and save Hpp' * bp
        // Hmm, shouldn't need this (augmented diagonal at end), 
        //   but seems to work better...
        Hpp.diagonal() *= lam;
        Matrix3d Hppi = Hpp.inverse(); // Which inverse should we use???? Note Hpp is symmetric; but this is not a bottleneck
        Vector3d &tp = tps[pi];
        tp = Hppi * bp;      

        // "outer product of track" in Step 4
        for(ProjMap::iterator itr = prjs.begin(); itr != prjs.end(); itr++)
          {
            Proj &prj = itr->second;
            if (!prj.isValid) continue;
            if (nodes[prj.ndi].isFixed) continue; // skip fixed cameras
            int ni = prj.ndi - nFixed;
            int ci = ni * 6;    // index of camera params (6DOF)
                                // NOTE: assumes fixed cams are at beginning
            csp.B.block<6,1>(ci,0) -= prj.jp->Hpc.transpose() * tp; // Hpc * tp subtracted from B
            prj.Tpc = prj.jp->Hpc.transpose() * Hppi;

            // iterate over nodes left on the track, plus yourself
            if (sparseType != SBA_GRADIENT)
              for(ProjMap::iterator itr2 = itr; itr2 != prjs.end(); itr2++)
                {
                  Proj &prj2 = itr2->second;
                  if (!prj2.isValid) continue;
                  if (nodes[prj2.ndi].isFixed) continue; // skip fixed cameras
                  int ni2 = prj2.ndi - nFixed; // NOTE: assumes fixed cams are at beginning
                  if (useConnMat && connMat[prj.ndi][prj2.ndi]) // check connection matrix filter
                    {
                      nskip++;
                      continue;
                    }
                  Matrix<double,6,6> m = -prj.Tpc * prj2.jp->Hpc;
                  if (ni == ni2)
                    csp.addDiagBlock(m,ni);
                  else
                    csp.addOffdiagBlock(m,ni,ni2);
                }
            else                // gradient calculation
              {
                Matrix<double,6,6> m = -prj.Tpc * prj.jp->Hpc;
                csp.addDiagBlock(m,ni);
              }

          } // finish outer product

      } // finish track

    //    cout << "[SetupSparseSys] Skipped conns: " << nskip << endl;

    t2 = utime();

    // set up sparse matrix structure from blocks
    if (sparseType == SBA_BLOCK_JACOBIAN_PCG)
      csp.incDiagBlocks(lam);	// increment diagonal block
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
      cout << "[SetupSys] " << ndc << " disconnected nodes" << endl;

  } // finish all tracks
  


  /// Run the LM algorithm that computes a nonlinear SBA estimate.
  /// <niter> is the max number of iterations to perform; returns the
  /// number actually performed.
  /// <useCSparse> = 0 for dense Cholesky, 1 for sparse system, 
  ///                2 for gradient system, 3 for block jacobian PCG
  /// <initTol> is the initial tolerance for CG 
  int SysSBA::doSBA(int niter, double sLambda, int useCSparse, double initTol, int maxCGiter)
  {
    // set aux buffer
    oldpoints.clear();
    oldpoints.resize(tracks.size());
    

    // storage
    int npts = tracks.size();
    int ncams = nodes.size();
    tps.resize(npts);

    // set number of projections
    int nprjs = 0;
    for(size_t i=0; i<tracks.size(); i++)
    {
      oldpoints[i] = tracks[i].point;
      nprjs += tracks[i].projections.size();
    }
    
    if (nprjs == 0 || npts == 0 || ncams == 0)
    {
      return -1;
    }

    // initialize vars
    if (sLambda > 0.0)          // do we initialize lambda?
      lambda = sLambda;

    // check for fixed frames
    if (nFixed <= 0)
      {
        cout << "[doSBA] No fixed frames" << endl;
        return 0;
      }
    for (int i=0; i<ncams; i++)
      {
        Node &nd = nodes[i];
        if (i >= nFixed)
          nd.isFixed = false;
        else 
          nd.isFixed = true;
        nd.setTransform(); // set up projection matrix for cost calculation
        nd.setProjection();
        nd.setDr(useLocalAngles);
      }

    // initialize vars
    double laminc = 2.0;        // how much to increment lambda if we fail
    double lamdec = 0.5;        // how much to decrement lambda if we succeed
    int iter = 0;               // iterations
    sqMinDelta = 1e-8 * 1e-8;
    updateNormals();
    double cost = calcCost();

    if (verbose > 0)
      {
	      cout << iter << " Initial squared cost: " << cost << " which is " 
	           << sqrt(cost/nprjs) << " rms pixel error and " 
	           << calcAvgError() << " average reproj error; " 
	           << numBadPoints() << " bad points" << endl;
      }

    for (; iter<niter; iter++)  // loop at most <niter> times
      {
        // set up and solve linear system
        // NOTE: shouldn't need to redo all calcs in setupSys if we 
        //   got here from a bad update

        // If we have point-plane matches, should update normals here.
        updateNormals();
        
        t0 = utime();
        if (useCSparse)
          setupSparseSys(lambda,iter,useCSparse); // sparse version
        else
          setupSys(lambda);     // set up linear system

        //        if (iter == 0)
        //          cout << endl << A << endl << endl;
        //        cout << "[SBA] Solving...";

#if 0
        int xs = B.size();
        char fn[2048];
        sprintf(fn,"A%d.txt",xs);
        printf("Writing file %s\n",fn);
        FILE *fd = fopen(fn,"w");
        fprintf(fd,"%d %d\n",xs,xs);
        for (int ii=0; ii<xs; ii++)
          for (int jj=0; jj<xs; jj++)
            fprintf(fd,"%.16g\n",A(ii,jj));
        fclose(fd);

        sprintf(fn,"B%d.txt",xs);
        fd = fopen(fn,"w");
        fprintf(fd,"%d\n",xs);
        for (int ii=0; ii<xs; ii++)
          fprintf(fd,"%.16g\n",B(ii));
        fclose(fd);
#endif

        t1 = utime();
	
	// use appropriate linear solver
	if (useCSparse == SBA_BLOCK_JACOBIAN_PCG)
	  {
            if (csp.B.rows() != 0)
	      {
		int iters = csp.doBPCG(maxCGiter,initTol,iter);
		cout << "[Block PCG] " << iters << " iterations" << endl;
	      }
	  }
        else if (useCSparse > 0)
          {
            if (csp.B.rows() != 0)
	      {
		bool ok = csp.doChol();
		if (!ok)
		  cout << "[DoSBA] Sparse Cholesky failed!" << endl;
	      }
          }
        else
          {
#if 1
            A.llt().solveInPlace(B); // Cholesky decomposition and solution
#else
            printf("\nDoing dpotrf/dpotrs\n");
            double *a = A.data();
            int info, m = B.size();
            double *x = B.data();
            int nrhs = 1;
            F77_FUNC(dpotrf)("U", (int *)&m, a, (int *)&m, (int *)&info);
            F77_FUNC(dpotrs)("U", (int *)&m, (int *)&nrhs, a, (int *)&m, x, (int *)&m, &info);
#endif
          }
        t2 = utime();
        //        printf("Matrix size: %d  Time: %d\n", B.size(), t2-t1);

        //        cout << "solved" << endl;

        // get correct result vector
        VectorXd &BB = useCSparse ? csp.B : B;

        // check for convergence
        // this is a pretty crummy convergence measure...
        double sqDiff = BB.squaredNorm();
        if (sqDiff < sqMinDelta) // converged, done...
          {
	          if (verbose > 0)
	            cout << "Converged with delta: " << sqrt(sqDiff) << endl;
                  break;
          }

        // update the cameras
        int ci = 0;
        for(vector<Node, Eigen::aligned_allocator<Node> >::iterator itr = nodes.begin(); itr != nodes.end(); itr++)
          {
            Node &nd = *itr;
            if (nd.isFixed) continue; // not to be updated
            nd.oldtrans = nd.trans; // save in case we don't improve the cost
            nd.oldqrot = nd.qrot;
            nd.trans.head<3>() += BB.segment<3>(ci);

            if (useLocalAngles)
              {
                Quaternion<double> qr;
                qr.vec() = BB.segment<3>(ci+3); 
                //                double sn = qr.vec().squaredNorm();
                //                if (sn > 0.01)  // usually indicates something has gone wrong
                //                  qr.vec() = qr.vec() / (sqrt(sn) * 10.0);
                qr.w() = sqrt(1.0 - qr.vec().squaredNorm());
                qr = nd.qrot*qr; // post-multiply, because we pre-multiply the transpose for Jacobian
                qr.normalize();
                nd.qrot = qr;
              }
            else
              {
                nd.qrot.coeffs().head<3>() += BB.segment<3>(ci+3); 
                nd.normRot();
              }

            nd.setTransform();  // set up projection matrix for cost calculation
            nd.setProjection();
            nd.setDr(useLocalAngles); // set rotational derivatives
            ci += 6;
          }

        // update the points (step 7)
        // loop over tracks
        int pi = 0;             // point index
        for(vector<Track, Eigen::aligned_allocator<Track> >::iterator itr = tracks.begin();
            itr != tracks.end(); itr++, pi++)
          {
            ProjMap &prjs = itr->projections;
            if (prjs.size() < 1) continue;
            Vector3d tp = tps[pi]; // copy to preserve the original
            // loop over cameras in each track
            for(ProjMap::iterator pitr = prjs.begin(); pitr != prjs.end(); pitr++)
              {
                Proj &prj = pitr->second;
                if (!prj.isValid) continue;
                if (nodes[prj.ndi].isFixed) continue; // only update with non-fixed cameras
                int ci = (prj.ndi - nFixed) * 6; // index of camera params (6DOF)
                                                // NOTE: assumes fixed cams are at beginning
                tp -= prj.Tpc.transpose() * BB.segment<6>(ci);
              }  
            // update point
            oldpoints[pi] = tracks[pi].point; // save for backing out
            tracks[pi].point.head(3) += tp;
          }

        t3 = utime();

        // new cost
        updateNormals();
        double newcost = calcCost();

        // average reprojection error (for Lourakis test)
        // write some stats
	      if (verbose > 0)
	        cout << iter << " Updated squared cost: " << newcost << " which is " 
	             << sqrt(newcost/(double)nprjs) << " rms pixel error and " 
	             << calcAvgError() << " average reproj error; " 
	             << numBadPoints() << " bad points" << endl;        


        // check if we did good
        if (newcost < cost) // && iter != 0) // NOTE: iter==0 case is for checking
          {
            cost = newcost;
            lambda *= lamdec;   // decrease lambda
            //      laminc = 2.0;       // reset bad lambda factor; not sure if this is a good idea...
          }
        else
          {
            lambda *= laminc;   // increase lambda
            laminc *= 2.0;      // increase the increment
            // reset points
            for(int i=0; i < (int)tracks.size(); i++)
            {
              tracks[i].point = oldpoints[i];
            }
            // reset cams
            for(int i=0; i < (int)nodes.size(); i++)
              {
                Node &nd = nodes[i];
                if (nd.isFixed) continue; // not to be updated
                nd.trans = nd.oldtrans;
                nd.qrot = nd.oldqrot;
                nd.setTransform(); // set up projection matrix for cost calculation
                nd.setProjection();
                nd.setDr(useLocalAngles);
              }
              
            updateNormals();
            cost = calcCost();  // need to reset errors
	          if (verbose > 0)
	            cout << iter << " Downdated cost: " << cost << endl;
                  // NOTE: shouldn't need to redo all calcs in setupSys
          }

        t4 = utime();
        if (iter == 0 && verbose > 0)
          printf("\n[SBA] Cost: %0.2f ms  Setup: %0.2f ms  Solve: %0.2f ms  Update: %0.2f ms  Total: %0.2f ms\n\n",
                 0.001*(double)(t4-t3),
                 0.001*(double)(t1-t0),
                 0.001*(double)(t2-t1),
                 0.001*(double)(t3-t2),
                 0.001*(double)(t4-t0));

      }

    // return number of iterations performed
    return iter;
  }


  /// merge tracks based on identity pairs
  /// this can be expensive
  void SysSBA::mergeTracks(std::vector<std::pair<int,int> > &prs)
  {
    // first form an ordered index of tracks
    int ntrs = tracks.size();
    vector<int> tris;
    tris.resize(ntrs);

    for (int i=0; i<ntrs; i++)
      tris[i] = i;
    
    // go over pairs and insert equalities
    for (int i=0; i<(int)prs.size(); i++)
      {
        pair<int,int> &pr = prs[i];
        int p0 = min(pr.first,pr.second);
        int p1 = max(pr.first,pr.second);
        tris[p1] = p0;
      }

    // do transitive closure
    for (int i=0; i<ntrs; i++)
      tris[i] = tris[tris[i]];

    // fix up tracks, monocular
    if (tracks.size() > 0)
      {
        for (int i=0; i<(int)tracks.size(); i++)
          {
            if (tris[i] == i) continue;

            ProjMap &tr0 = tracks[tris[i]].projections;
            ProjMap &tr1 = tracks[i].projections;

            for(ProjMap::iterator itr1 = tr1.begin(); itr1 != tr1.end(); itr1++)
              {
                Proj &prj = itr1->second;
                int ci = prj.ndi;
                
                // Insert the projection into the original track
                tr0[ci] = prj;
              }
            tr1.clear();
          }

        // fill up holes, reset point indices
        int n = 0;
        for (int i=0; i<(int)tracks.size(); i++)
          {
            if (tris[i] != i) continue;
            if (n == i) continue;
            tracks[n] = tracks[i];
            tracks[n].point = tracks[i].point;
            
            // We don't really do anything here? Should this be uncommented?
            // std::ProjMap &tr = tracks[n].projections;
            /*for (int j=0; j<(int)tr.size(); j++)
              tr[j].pti = n; */
            n++;
          }
        tracks.resize(n);
      }
  }


  /// merge 2 tracks
  /// leave 2nd track null; eventually need to clean up null tracks
  /// returns merged track index if successful, -1 if tracks are redundant 
  ///     (same cam found on both with different keypts)
  int SysSBA::mergeTracksSt(int tri0, int tri1)
  {
    // loop through track tr1, adding projections to tr0 and checking for redundancy
    ProjMap tr0 = tracks[tri0].projections; // duplicate this track, for undo
    ProjMap &tr1 = tracks[tri1].projections;
    
    for(ProjMap::iterator itr = tr1.begin(); itr != tr1.end(); itr++)
      {
        Proj &prj = itr->second;
        bool ok = addProj(prj.ndi, tri0, prj.kp, prj.stereo);
        if (!ok)
          {
            tracks[tri0].projections = tr0; // reset to original track
            return -1;
          }
      }

    tr1.clear();
    return tri0;
  }


} // end namespace sba
