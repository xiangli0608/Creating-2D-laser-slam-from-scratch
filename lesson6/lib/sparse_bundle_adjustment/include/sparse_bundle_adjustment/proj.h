#ifndef _PROJ_H_
#define _PROJ_H_

#ifndef EIGEN_USE_NEW_STDVECTOR
#define EIGEN_USE_NEW_STDVECTOR
#endif // EIGEN_USE_NEW_STDVECTOR

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/StdVector>
#include <map>

#include <sparse_bundle_adjustment/node.h>

namespace sba
{
  class JacobProds
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // needed for 16B alignment

    JacobProds() 
    {
      Hpp.setZero();
      Hpc.setZero();
      Hcc.setZero();
      Bp.setZero();
      JcTE.setZero();
    }

    /// Point-to-point Hessian (JpT*Jp).
    Eigen::Matrix<double,3,3> Hpp;
      
    /// Point-to-camera Hessian (JpT*Jc)
    Eigen::Matrix<double,3,6> Hpc;
      
    /// Camera-to-camera Hessian (JcT*Jc)
    Eigen::Matrix<double,6,6> Hcc;
      
    /// The B matrix with respect to points (JpT*Err)
    Eigen::Matrix<double,3,1> Bp;
      
    /// Another matrix with respect to cameras (JcT*Err)
    Eigen::Matrix<double,6,1> JcTE;
  };

  class Proj; // Forward reference.
  
  /// Obnoxiously long type def for the map type that holds the point 
  /// projections in tracks.
  typedef std::map<const int, Proj, std::less<int>, Eigen::aligned_allocator<Proj> > ProjMap;

  /// \brief Proj holds a projection measurement of a point onto a
  /// frame. They are a repository for the link between the frame and
  /// the point, with auxillary info such as Jacobians.
  class Proj
  {
    public:

      EIGEN_MAKE_ALIGNED_OPERATOR_NEW // needed for 16B alignment

      /// \brief General & stereo constructor. To construct a monocular 
      /// projection, either use stereo = false or the other constructor.
      /// NOTE: sets the projection to be valid.
      Proj(int ci, Eigen::Vector3d &q, bool stereo = true);
      
      /// \brief Monocular constructor. To construct a stereo projection, 
      /// use other constructor.
      /// NOTE: sets the projection to be valid.
      Proj(int ci, Eigen::Vector2d &q);
      
      /// \brief Default constructor. Initializes to default values, 
      /// kp = <0 0 0> and ndi = <0>. Also sets the projection to be invalid.
      Proj();
      
      /// Node index, the camera node for this projection.
      int ndi;
      
      /// Keypoint, u,v,u-d vector
      Eigen::Vector3d kp;
      
      /// Reprojection error.
      Eigen::Vector3d err;
      
      /// Whether the projection is Stereo (True) or Monocular (False).
      bool stereo;
      
      /// Calculates re-projection error and stores it in #err.
     double calcErr(const Node &nd, const Point &pt, double huber = 0.0);
      
      /// \brief Get the correct squared norm of the error, depending on 
      /// whether the projection is monocular or stereo.
      double getErrSquaredNorm();
      
      /// \brief Get the correct norm of the error, depending on whether the 
      /// projection is monocular or stereo.
      double getErrNorm();
      
      /// Sets the jacobians and hessians for the projection to use for SBA.
      /** Monocular:
      
          dpc/dq = dR'/dq [pw-t], in homogeneous form, with q a quaternion param
          dpc/dx = -R' * [1 0 0]', in homogeneous form, with x a translation param
          d(px/pz)/du = [ pz dpx/du - px dpz/du ] / pz^2,
          works for all variables       
          
          Stereo:
          pc = R'[pw-t]            => left cam
          pc = R'[pw-t] + [b 0 0]' => right cam px only

          dpc/dq = dR'/dq [pw-t], in homogeneous form, with q a quaternion param
          dpc/dx = -R' * [1 0 0]', in homogeneous form, with x a translation param
          d(px/pz)/du = [ pz dpx/du - px dpz/du ] / pz^2,
          works for all variables
          only change for right cam is px += b */
      void setJacobians(const Node &nd, const Point &pt, JacobProds *jpp);
      
      /// Jacobian products
      JacobProds *jp;
      
      /// Point-to-camera matrix (HpcT*Hpp^-1)
      /// Need to save this
      Eigen::Matrix<double,6,3> Tpc;
      
      /// valid or not (could be out of bounds)
      bool isValid;
      
      /// scaling factor for quaternion derivatives relative to translational ones;
      /// not sure if this is needed, it's close to 1.0
      static constexpr double qScale = 1.0;
      
      /// Use a covariance matrix?
      bool useCovar;
      
      /// Covariance matrix for cost calculation.
      Eigen::Matrix<double,3,3> covarmat;
      
      /// Whether this is a point-plane match (true) or a point-point match (false).
      bool pointPlane;
      
      /// Normal for point-plane projections
      Eigen::Vector3d plane_normal;
      
      /// Point for point-plane projections
      Eigen::Vector3d plane_point;
      
      /// Point-plane match point index in SBA.
      int plane_point_index;
      
      /// Point-plane node index in SBA.
      int plane_node_index;
      
      /// Original normal in #plane_node_index coordinate's frame.
      Eigen::Vector3d plane_local_normal;

      /// \brief Set the covariance matrix to use for cost calculation.
      /// Without the covariance matrix, cost is calculated by:
      /// cost = ||err||
      /// With a covariance matrix, the cost is calculated by:
      /// cost = (err)T*covar*(err)
      void setCovariance(const Eigen::Matrix3d &covar);
      
      /// Clear the covariance matrix and no longer use it.
      void clearCovariance();
      
      
    protected:
      /// Set monocular jacobians/hessians.
      void setJacobiansMono_(const Node &nd, const Point &pt, JacobProds *jpp);
      
      /// Set stereo jacobians/hessians.
      void setJacobiansStereo_(const Node &nd, const Point &pt, JacobProds *jpp);
      
      /// Calculate error function for stereo.
      double calcErrMono_(const Node &nd, const Point &pt, double huber);
      
      /// Calculate error function for stereo.
      double calcErrStereo_(const Node &nd, const Point &pt, double huber);
  };
    
  class Track
  {
    public:
      /// Constructor for a Track at point p.
      Track(Point p);
      
      /// Default constructor for Track.
      Track();
      
      /// \brief A map of all the projections of the point with camera index 
      /// as key, based off an STL map.
      ProjMap projections;
      
      /// \brief An Eigen 4-vector containing the <x, y, z, w> coordinates of 
      /// the point associated with the track.
      Point point;
  };
  
  
} // sba

#endif // _PROJ_H
