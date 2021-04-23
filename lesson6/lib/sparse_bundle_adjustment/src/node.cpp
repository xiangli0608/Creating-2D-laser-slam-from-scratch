#include <sparse_bundle_adjustment/node.h>

using namespace std;
using namespace Eigen;

namespace sba
{
    void Node::setTransform()
    { transformW2F(w2n,trans,qrot); }

  //
  // sets incremental angle derivatives
  //
  void Node::setDri()
  {
    setDr(true);
  }

  // constant derivative matrices
  // these are the derivatives of the *inverse* rotation
  Eigen::Matrix3d Node::dRidx, Node::dRidy, Node::dRidz;

  void Node::initDr()
  {
    dRidx  << 0.0,0.0,0.0,  
              0.0,0.0,2.0,
              0.0,-2.0,0.0;
    dRidy  << 0.0,0.0,-2.0,
              0.0,0.0,0.0,
              2.0,0.0,0.0;
    dRidz  << 0.0,2.0,0.0,  
              -2.0,0.0,0.0,
              0.0,0.0,0.0;
  }


  void Node::normRot()
  { 
    //      std::cout << "[NormRot] qrot start = " << qrot.transpose() << std::endl;
    if (qrot.w() < 0) qrot.coeffs().head<3>() = -qrot.coeffs().head<3>();
    double sn = qrot.coeffs().head<3>().squaredNorm();
    if (sn >= 0.9999)            // too close to high derivatives
      qrot.coeffs().head<3>() *= -1.0/(sqrt(sn)*1.0001); // switch sides; 1e-4 seems to work well
    qrot.w() = sqrt(1.0 - qrot.coeffs().head<3>().squaredNorm());
    if (std::isnan(qrot.x()) || std::isnan(qrot.y()) || std::isnan(qrot.z()) || std::isnan(qrot.w()) )
      { 
        printf("[NormRot] Bad quaternion: %f %f %f %f\n", qrot.x(), qrot.y(), qrot.z(), qrot.w()); 
        exit(1); 
      }
    //      std::cout << "[NormRot] qrot end   = " << qrot.transpose() << std::endl;
  }

  //
  // sets angle derivatives
  //
  void Node::setDr(bool local)
  {
    // for dS'*R', with dS the incremental change
    if (local)
      {
#if 0
        dRdx = w2n.block<3,3>(0,0) * dRidx;
        dRdy = w2n.block<3,3>(0,0) * dRidy;
        dRdz = w2n.block<3,3>(0,0) * dRidz;
#endif
        dRdx = dRidx * w2n.block<3,3>(0,0);
        dRdy = dRidy * w2n.block<3,3>(0,0);
        dRdz = dRidz * w2n.block<3,3>(0,0);

      }
    else
      {
        double x2 = qrot.x() * 2.0;
        double y2 = qrot.y() * 2.0;
        double z2 = qrot.z() * 2.0;
        double w2 = qrot.w() * 2.0;
        double xw = qrot.x()/qrot.w(); // these are problematic for w near zero
        double yw = qrot.y()/qrot.w();
        double zw = qrot.z()/qrot.w();

        // dR/dx 
        dRdx(0,0) = 0.0;
        dRdx(0,1) = y2-zw*x2;
        dRdx(0,2) = z2+yw*x2;

        dRdx(1,0) = y2+zw*x2;
        dRdx(1,1) = -2.0*x2;
        dRdx(1,2) = w2-xw*x2;

        dRdx(2,0) = z2-yw*x2;
        dRdx(2,1) = -dRdx(1,2);
        dRdx(2,2) = dRdx(1,1);
      
        // dR/dy 
        dRdy(0,0) = -2.0*y2;
        dRdy(0,1) = x2-zw*y2;
        dRdy(0,2) = (-w2)+yw*y2;

        dRdy(1,0) = x2+zw*y2;
        dRdy(1,1) = 0.0;
        dRdy(1,2) = dRdx(2,0);

        dRdy(2,0) = -dRdy(0,2);
        dRdy(2,1) = dRdx(0,2);
        dRdy(2,2) = dRdy(0,0);

        // dR/dz
        dRdz(0,0) = -2.0*z2;
        dRdz(0,1) = w2-zw*z2;
        dRdz(0,2) = dRdy(1,0);

        dRdz(1,0) = -dRdz(0,1);
        dRdz(1,1) = dRdz(0,0);
        dRdz(1,2) = dRdx(0,1);

        dRdz(2,0) = dRdy(0,1);
        dRdz(2,1) = dRdx(1,0);
        dRdz(2,2) = 0.0;
      }
  }
  
  void Node::normRotLocal()
  {
      qrot.normalize();
      if (qrot.w() < 0) qrot.coeffs().head<3>() = -qrot.coeffs().head<3>();
      if (std::isnan(qrot.x()) || std::isnan(qrot.y()) || std::isnan(qrot.z()) || std::isnan(qrot.w()) )
        { 
          printf("[NormRot] Bad quaternion in normRotLocal(): %f %f %f %f\n", qrot.x(), qrot.y(), qrot.z(), qrot.w());
          exit(1); 
        }
      //      std::cout << "[NormRot] qrot end   = " << qrot.transpose() << std::endl;
  }
   
  void Node::projectMono(const Point& point, Eigen::Vector3d& proj)
  {
    Vector2d proj2d;
    project2im(proj2d, point);
    
    proj.head<2>() = proj2d;
  }
  
  void Node::projectStereo(const Point& point, Eigen::Vector3d& proj)
  {
    Vector2d proj2d;
    Vector3d pc, baseline_vect;
    project2im(proj2d, point);
    
    // Camera coords for right camera
    baseline_vect << baseline, 0, 0;
    pc = Kcam * (w2n*point - baseline_vect); 
    proj.head<2>() = proj2d;
    proj(2) = pc(0)/pc(2);
  }
  
  
  // transforms
  void transformW2F(Eigen::Matrix<double,3,4> &m, 
                    const Eigen::Matrix<double,4,1> &trans, 
                    const Eigen::Quaternion<double> &qrot)
  {
    m.block<3,3>(0,0) = qrot.toRotationMatrix().transpose();
    m.col(3).setZero();         // make sure there's no translation
    m.col(3) = -m*trans;
  };

  void transformF2W(Eigen::Matrix<double,3,4> &m, 
                    const Eigen::Matrix<double,4,1> &trans, 
                    const Eigen::Quaternion<double> &qrot)
  {
    m.block<3,3>(0,0) = qrot.toRotationMatrix();
    m.col(3) = trans.head(3);
  };


  // returns the local R,t in nd0 that produces nd1
  // NOTE: returns a postfix rotation; should return a prefix
  void transformN2N(Eigen::Matrix<double,4,1> &trans, 
                    Eigen::Quaternion<double> &qr,
                    Node &nd0, Node &nd1)
  {
    Matrix<double,3,4> tfm;
    Quaterniond q0,q1;
    q0 = nd0.qrot;
    transformW2F(tfm,nd0.trans,q0);
    trans.head(3) = tfm*nd1.trans;
    trans(3) = 1.0;
    q1 = nd1.qrot;
    qr = q0.inverse()*q1;
    qr.normalize();
    if (qr.w() < 0)
      qr.coeffs() = -qr.coeffs();
  }


} // namespace sba


