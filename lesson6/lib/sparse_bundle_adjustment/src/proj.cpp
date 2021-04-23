#include <sparse_bundle_adjustment/proj.h>

namespace sba
{
  Proj::Proj(int ci, Eigen::Vector3d &q, bool stereo)
      : ndi(ci), kp(q), stereo(stereo), 
        isValid(true), useCovar(false), pointPlane(false) {}
      
  Proj::Proj(int ci, Eigen::Vector2d &q) 
      : ndi(ci), kp(q(0), q(1), 0), 
        stereo(false), isValid(true), useCovar(false), pointPlane(false) {}
  
  Proj::Proj() 
      : ndi(0), kp(0, 0, 0), 
        stereo(false), isValid(false), useCovar(false), pointPlane(false) {}

  void Proj::setJacobians(const Node &nd, const Point &pt, JacobProds *jpp)
  {
    if (stereo)
      setJacobiansStereo_(nd, pt, jpp);
    else
      setJacobiansMono_(nd, pt, jpp);
  }
  
  double Proj::calcErr(const Node &nd, const Point &pt, const double huber)
  {
    if (stereo)
      return calcErrStereo_(nd, pt, huber);
    else
      return calcErrMono_(nd, pt, huber);
  }
  
  double Proj::getErrNorm()
  {
    if (stereo)
      return err.norm();
    else
      return err.head<2>().norm();
  }
  
  double Proj::getErrSquaredNorm()
  {
    if (stereo)
      return err.squaredNorm();
    else
      return err.head<2>().squaredNorm();
  }
  
  void Proj::setCovariance(const Eigen::Matrix3d &covar)
  {
    useCovar = true;
    covarmat = covar;
  }
  
  void Proj::clearCovariance()
  {
    useCovar = false;
  }

  void Proj::setJacobiansMono_(const Node &nd, const Point &pt, JacobProds *jpp)
  {
    // first get the world point in camera coords
    Eigen::Matrix<double,3,1> pc = nd.w2n * pt;

    /// jacobian with respect to frame; uses dR'/dq from Node calculation
    Eigen::Matrix<double,2,6> jacc;
    
    /// jacobian with respect to point
    Eigen::Matrix<double,2,3> jacp;

    // Jacobians wrt camera parameters
    // set d(quat-x) values [ pz*dpx/dx - px*dpz/dx ] / pz^2
    double px = pc(0);
    double py = pc(1);
    double pz = pc(2);
    double ipz2 = 1.0/(pz*pz);
    if (std::isnan(ipz2) ) { printf("[SetJac] infinite jac\n");  *(int *)0x0 = 0; }

    double ipz2fx = ipz2*nd.Kcam(0,0); // Fx
    double ipz2fy = ipz2*nd.Kcam(1,1); // Fy
    // scale quaternion derivative to match the translational ones
    double ipz2fxq = qScale*ipz2fx;
    double ipz2fyq = qScale*ipz2fy;
    Eigen::Matrix<double,3,1> pwt;

    // check for local vars
    pwt = (pt-nd.trans).head<3>(); // transform translations, use differential rotation

    // dx
    Eigen::Matrix<double,3,1> dp = nd.dRdx * pwt; // dR'/dq * [pw - t]
    jacc(0,3) = (pz*dp(0) - px*dp(2))*ipz2fxq;
    jacc(1,3) = (pz*dp(1) - py*dp(2))*ipz2fyq;
    // dy
    dp = nd.dRdy * pwt; // dR'/dq * [pw - t]
    jacc(0,4) = (pz*dp(0) - px*dp(2))*ipz2fxq;
    jacc(1,4) = (pz*dp(1) - py*dp(2))*ipz2fyq;
    // dz
    dp = nd.dRdz * pwt; // dR'/dq * [pw - t]
    jacc(0,5) = (pz*dp(0) - px*dp(2))*ipz2fxq;
    jacc(1,5) = (pz*dp(1) - py*dp(2))*ipz2fyq;

    // set d(t) values [ pz*dpx/dx - px*dpz/dx ] / pz^2
    dp = -nd.w2n.col(0);        // dpc / dx
    jacc(0,0) = (pz*dp(0) - px*dp(2))*ipz2fx;
    jacc(1,0) = (pz*dp(1) - py*dp(2))*ipz2fy;
    dp = -nd.w2n.col(1);        // dpc / dy
    jacc(0,1) = (pz*dp(0) - px*dp(2))*ipz2fx;
    jacc(1,1) = (pz*dp(1) - py*dp(2))*ipz2fy;
    dp = -nd.w2n.col(2);        // dpc / dz
    jacc(0,2) = (pz*dp(0) - px*dp(2))*ipz2fx;
    jacc(1,2) = (pz*dp(1) - py*dp(2))*ipz2fy;

    // Jacobians wrt point parameters
    // set d(t) values [ pz*dpx/dx - px*dpz/dx ] / pz^2
    dp = nd.w2n.col(0); // dpc / dx
    jacp(0,0) = (pz*dp(0) - px*dp(2))*ipz2fx;
    jacp(1,0) = (pz*dp(1) - py*dp(2))*ipz2fy;
    dp = nd.w2n.col(1); // dpc / dy
    jacp(0,1) = (pz*dp(0) - px*dp(2))*ipz2fx;
    jacp(1,1) = (pz*dp(1) - py*dp(2))*ipz2fy;
    dp = nd.w2n.col(2); // dpc / dz
    jacp(0,2) = (pz*dp(0) - px*dp(2))*ipz2fx;
    jacp(1,2) = (pz*dp(1) - py*dp(2))*ipz2fy;

#ifdef DEBUG
    for (int i=0; i<2; i++)
      for (int j=0; j<6; j++)
        if (std::isnan(jacc(i,j)) ) { printf("[SetJac] NaN in jacc(%d,%d)\n", i, j);  *(int *)0x0 = 0; }
#endif
    
    // Set Hessians + extras.
    jpp->Hpp = jacp.transpose() * jacp;
    jpp->Hcc = jacc.transpose() * jacc;
    jpp->Hpc = jacp.transpose() * jacc;
    jpp->JcTE = jacc.transpose() * err.head<2>();
    jpp->Bp = jacp.transpose() * err.head<2>();

    jp = jpp;
  }

  // calculate error of a projection
  // we should do something about negative Z
  double Proj::calcErrMono_(const Node &nd, const Point &pt, double huber)
  {
    Eigen::Vector3d p1 = nd.w2i * pt; 
    err(2) = 0.0;
    if (p1(2) <= 0.0) 
    {
#ifdef DEBUG
      printf("[CalcErr] negative Z! Node %d \n",ndi);
      if (std::isnan(err[0]) || std::isnan(err[1]) ) printf("[CalcErr] NaN!\n");
#endif
      err = Eigen::Vector3d(0.0,0.0,0.0);
      return 0.0;
    }
    else
      err.head<2>() = p1.head<2>()/p1(2); 

    err -= kp;

    // pseudo-Huber weighting
    // C(e) = 2*s^2*[sqrt(1+(e/s)^2)-1]
    // w = sqrt(C(norm(e)))/norm(e)

    if (huber > 0)
      {
        double b2 = huber*huber; // kernel width
        double e2 = err.head<2>().squaredNorm();
        if (e2 > b2)
          {
            double c = 2.0*huber*sqrt(e2) - b2;
            double w = sqrt(c/e2);
            err.head<2>() *= w; // weight the error
            //            std::cout << "Huber weight: " << w << "  Err sq: " << e2 << std::endl;
          }


//         double b2 = HUBER*HUBER;        // kernel width
//         double e2 = err.squaredNorm();
//         e2 = std::max(e2,1e-22);    // can't have a zero here
//         double w = 2*b2*(sqrt(1+e2/b2)-1);
//         w = sqrt(w/e2);
//         err.head<2>() *= w;         // weight the error
      }

    return err.head<2>().squaredNorm(); 
  }


  void Proj::setJacobiansStereo_(const Node &nd, const Point &pt, JacobProds *jpp)
  {
    // first get the world point in camera coords
    Eigen::Matrix<double,3,1> pc = nd.w2n * pt;

    /// jacobian with respect to point
    Eigen::Matrix<double,3,3> jacp;
    
    /// jacobian with respect to frame; uses dR'/dq from Node calculation
    Eigen::Matrix<double,3,6> jacc;

    // Jacobians wrt camera parameters
    // set d(quat-x) values [ pz*dpx/dx - px*dpz/dx ] / pz^2
    double px = pc(0);
    double py = pc(1);
    double pz = pc(2);
    double ipz2 = 1.0/(pz*pz);
    if (std::isnan(ipz2) ) { printf("[SetJac] infinite jac\n");  *(int *)0x0 = 0; }

    double ipz2fx = ipz2*nd.Kcam(0,0); // Fx
    double ipz2fy = ipz2*nd.Kcam(1,1); // Fy
    double b      = nd.baseline; // stereo baseline
    // scale quaternion derivative to match the translational ones
    double ipz2fxq = qScale*ipz2fx;
    double ipz2fyq = qScale*ipz2fy;
    Eigen::Matrix<double,3,1> pwt;

    // check for local vars
    pwt = (pt-nd.trans).head(3); // transform translations, use differential rotation

    // dx
    Eigen::Matrix<double,3,1> dp = nd.dRdx * pwt; // dR'/dq * [pw - t]
    jacc(0,3) = (pz*dp(0) - px*dp(2))*ipz2fxq;
    jacc(1,3) = (pz*dp(1) - py*dp(2))*ipz2fyq;
    jacc(2,3) = (pz*dp(0) - (px-b)*dp(2))*ipz2fxq; // right image px
    // dy
    dp = nd.dRdy * pwt; // dR'/dq * [pw - t]
    jacc(0,4) = (pz*dp(0) - px*dp(2))*ipz2fxq;
    jacc(1,4) = (pz*dp(1) - py*dp(2))*ipz2fyq;
    jacc(2,4) = (pz*dp(0) - (px-b)*dp(2))*ipz2fxq; // right image px
    // dz
    dp = nd.dRdz * pwt; // dR'/dq * [pw - t]
    jacc(0,5) = (pz*dp(0) - px*dp(2))*ipz2fxq;
    jacc(1,5) = (pz*dp(1) - py*dp(2))*ipz2fyq;
    jacc(2,5) = (pz*dp(0) - (px-b)*dp(2))*ipz2fxq; // right image px

    // set d(t) values [ pz*dpx/dx - px*dpz/dx ] / pz^2
    dp = -nd.w2n.col(0);        // dpc / dx
    jacc(0,0) = (pz*dp(0) - px*dp(2))*ipz2fxq;
    jacc(1,0) = (pz*dp(1) - py*dp(2))*ipz2fy;
    jacc(2,0) = (pz*dp(0) - (px-b)*dp(2))*ipz2fxq; // right image px
    dp = -nd.w2n.col(1);        // dpc / dy
    jacc(0,1) = (pz*dp(0) - px*dp(2))*ipz2fxq;
    jacc(1,1) = (pz*dp(1) - py*dp(2))*ipz2fy;
    jacc(2,1) = (pz*dp(0) - (px-b)*dp(2))*ipz2fxq; // right image px
    dp = -nd.w2n.col(2);        // dpc / dz
    jacc(0,2) = (pz*dp(0) - px*dp(2))*ipz2fxq;
    jacc(1,2) = (pz*dp(1) - py*dp(2))*ipz2fy;
    jacc(2,2) = (pz*dp(0) - (px-b)*dp(2))*ipz2fxq; // right image px

    // Jacobians wrt point parameters
    // set d(t) values [ pz*dpx/dx - px*dpz/dx ] / pz^2
    dp = nd.w2n.col(0); // dpc / dx
    jacp(0,0) = (pz*dp(0) - px*dp(2))*ipz2fxq;
    jacp(1,0) = (pz*dp(1) - py*dp(2))*ipz2fy;
    jacp(2,0) = (pz*dp(0) - (px-b)*dp(2))*ipz2fxq; // right image px
    dp = nd.w2n.col(1); // dpc / dy
    jacp(0,1) = (pz*dp(0) - px*dp(2))*ipz2fxq;
    jacp(1,1) = (pz*dp(1) - py*dp(2))*ipz2fy;
    jacp(2,1) = (pz*dp(0) - (px-b)*dp(2))*ipz2fxq; // right image px
    dp = nd.w2n.col(2); // dpc / dz
    jacp(0,2) = (pz*dp(0) - px*dp(2))*ipz2fxq;
    jacp(1,2) = (pz*dp(1) - py*dp(2))*ipz2fy;
    jacp(2,2) = (pz*dp(0) - (px-b)*dp(2))*ipz2fxq; // right image px

#ifdef DEBUG
    for (int i=0; i<2; i++)
      for (int j=0; j<6; j++)
        if (std::isnan(jacc(i,j)) ) { printf("[SetJac] NaN in jacc(%d,%d)\n", i, j);  *(int *)0x0 = 0; }
#endif
    if (useCovar)
    {
      jacc = covarmat * jacc;
      jacp = covarmat * jacp;
    }

    // Set Hessians + extras.
    jpp->Hpp = jacp.transpose() * jacp;
    jpp->Hcc = jacc.transpose() * jacc;
    jpp->Hpc = jacp.transpose() * jacc;
    jpp->JcTE = jacc.transpose() * err;
    jpp->Bp = jacp.transpose() * err;
    
    jp = jpp;
  }

  // calculate error of a projection
  // we should do something about negative Z
  double Proj::calcErrStereo_(const Node &nd, const Point &pt, double huber)
  { 
    Eigen::Vector3d p1 = nd.w2i * pt; 
    Eigen::Vector3d p2 = nd.w2n * pt; 
    Eigen::Vector3d pb(nd.baseline,0,0);
    
    // TODO: Clean this up a bit. 
    if (pointPlane)
    {
      // Project point onto plane.
      Eigen::Vector3d w = pt.head<3>()-plane_point;

      //printf("w: %f %f %f\n", w.x(), w.y(), w.z());
      //Eigen::Vector3d projpt = pt.head<3>()+(w.dot(plane_normal))*plane_normal;
      Eigen::Vector3d projpt = plane_point+(w.dot(plane_normal))*plane_normal;
      //      Eigen::Vector3d projpt = pt.head<3>()+(w.dot(plane_normal))*plane_normal;
      //printf("[Proj] Distance to plane: %f\n", w.dot(plane_normal));
      p1 = nd.w2i*Eigen::Vector4d(projpt.x(), projpt.y(), projpt.z(), 1.0);
      p2 = nd.w2n*Eigen::Vector4d(projpt.x(), projpt.y(), projpt.z(), 1.0);
    }
    
    double invp1 = 1.0/p1(2);
    
    err.head<2>() = p1.head<2>()*invp1;
    // right camera px
    p2 = nd.Kcam*(p2-pb);
 
    err(2) = p2(0)/p2(2);
    if (p1(2) <= 0.0) 
    {
#ifdef DEBUG
      printf("[CalcErr] negative Z! Node %d\n",ndi);
      if (std::isnan(err[0]) || std::isnan(err[1]) ) printf("[CalcErr] NaN!\n");
#endif
      err.setZero();
      
      return 0.0;
    }
    err -= kp;
    
    if (abs(err(0)) > 1e6 || abs(err(1)) > 1e6 || abs(err(2)) > 1e6)
    {
      printf("\n\n[CalcErr] Excessive error.\n");
      
      isValid = false;
    }
    
    if (useCovar)
      err = covarmat*err;
     
    // Huber kernel weighting
    if (huber > 0.0)
      {
        double b2 = huber*huber; // kernel width
        double e2 = err.squaredNorm();
        if (e2 > b2)
          {
            double c = 2.0*huber*sqrt(e2) - b2;
            double w = sqrt(c/e2);
            err *= w;
            //            std::cout << "Huber weight: " << w << "  Err sq: " << e2 << std::endl;
          }
      }

    return err.squaredNorm();
  }
  
  // Constructors for track.
  Track::Track() : point() { }
  Track::Track(Point p) : point(p) { }
      
} // sba

