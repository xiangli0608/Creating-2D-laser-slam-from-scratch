#include <sparse_bundle_adjustment/visualization.h>

namespace sba {

void drawGraph(const SysSBA &sba, const ros::Publisher &camera_pub,
               const ros::Publisher &point_pub, int decimation, int bicolor)
{
  int num_points = sba.tracks.size();
  int num_cameras = sba.nodes.size();
  if (num_points == 0 && num_cameras == 0) return;
  
  visualization_msgs::Marker camera_marker, point_marker;
  camera_marker.header.frame_id = "/pgraph";
  camera_marker.header.stamp = ros::Time::now();
  camera_marker.ns = "pgraph";
  camera_marker.id = 0;
  camera_marker.action = visualization_msgs::Marker::ADD;
  camera_marker.pose.position.x = 0;
  camera_marker.pose.position.y = 0;
  camera_marker.pose.position.z = 0;
  camera_marker.pose.orientation.x = 0.0;
  camera_marker.pose.orientation.y = 0.0;
  camera_marker.pose.orientation.z = 0.0;
  camera_marker.pose.orientation.w = 1.0;
  camera_marker.scale.x = 0.02;
  camera_marker.scale.y = 0.02;
  camera_marker.scale.z = 0.02;
  camera_marker.color.r = 0.0f;
  camera_marker.color.g = 1.0f;
  camera_marker.color.b = 1.0f;
  camera_marker.color.a = 1.0f;
  camera_marker.lifetime = ros::Duration();
  camera_marker.type = visualization_msgs::Marker::LINE_LIST;

  point_marker = camera_marker;
  point_marker.color.r = 1.0f;
  point_marker.color.g = 0.0f;
  point_marker.color.b = 0.0f;
  point_marker.color.a = 0.5f;
  point_marker.scale.x = 0.02;
  point_marker.scale.y = 0.02;
  point_marker.scale.z = 0.02;
  point_marker.type = visualization_msgs::Marker::POINTS;

  // draw points, decimated
  point_marker.points.resize((int)(num_points/(double)decimation + 0.5));
  point_marker.colors.resize((int)(num_points/(double)decimation + 0.5));
  for (int i=0, ii=0; i < num_points; i += decimation, ii++)
    {
      const Vector4d &pt = sba.tracks[i].point;
      point_marker.colors[ii].r = 1.0f;
      if (bicolor > 0 && i >= bicolor)
	point_marker.colors[ii].g = 1.0f;
      else
	point_marker.colors[ii].g = 0.0f;
      point_marker.colors[ii].b = 0.0f;
      point_marker.points[ii].x = pt(2);
      point_marker.points[ii].y = -pt(0);
      point_marker.points[ii].z = -pt(1);
    }

  // draw cameras
  camera_marker.points.resize(num_cameras*6);
  for (int i=0, ii=0; i < num_cameras; i++)
    {
      const Node &nd = sba.nodes[i];
      Vector3d opt;
      Matrix<double,3,4> tr;
      transformF2W(tr,nd.trans,nd.qrot);

      camera_marker.points[ii].x = nd.trans.z();
      camera_marker.points[ii].y = -nd.trans.x();
      camera_marker.points[ii++].z = -nd.trans.y();
      opt = tr*Vector4d(0,0,0.3,1);
      camera_marker.points[ii].x = opt.z();
      camera_marker.points[ii].y = -opt.x();
      camera_marker.points[ii++].z = -opt.y();

      camera_marker.points[ii].x = nd.trans.z();
      camera_marker.points[ii].y = -nd.trans.x();
      camera_marker.points[ii++].z = -nd.trans.y();
      opt = tr*Vector4d(0.2,0,0,1);
      camera_marker.points[ii].x = opt.z();
      camera_marker.points[ii].y = -opt.x();
      camera_marker.points[ii++].z = -opt.y();

      camera_marker.points[ii].x = nd.trans.z();
      camera_marker.points[ii].y = -nd.trans.x();
      camera_marker.points[ii++].z = -nd.trans.y();
      opt = tr*Vector4d(0,0.1,0,1);
      camera_marker.points[ii].x = opt.z();
      camera_marker.points[ii].y = -opt.x();
      camera_marker.points[ii++].z = -opt.y();
    }

  // draw point-plane projections
  int num_tracks = sba.tracks.size();
  int ii = camera_marker.points.size();

  for (int i=0; i < num_tracks; i++)
    {
      const ProjMap &prjs = sba.tracks[i].projections;
      for (ProjMap::const_iterator itr = prjs.begin(); itr != prjs.end(); itr++)
        {
          const Proj &prj = (*itr).second;
          if (prj.pointPlane)	// have a ptp projection
            {
              camera_marker.points.resize(ii+2);
              Point pt0 = sba.tracks[i].point;
              Vector3d plane_point = prj.plane_point;
              Vector3d plane_normal = prj.plane_normal;
              Eigen::Vector3d w = pt0.head<3>()-plane_point;
              //              Eigen::Vector3d projpt = plane_point+(w.dot(plane_normal))*plane_normal;
              Eigen::Vector3d projpt = pt0.head<3>() - (w.dot(plane_normal))*plane_normal;
              //              Vector3d pt1 = pt0.head<3>()+0.1*plane_normal;
              Vector3d pt1 = projpt;
	          
              camera_marker.points[ii].x = pt0.z();
              camera_marker.points[ii].y = -pt0.x();
              camera_marker.points[ii++].z = -pt0.y();
              camera_marker.points[ii].x = pt1.z();
              camera_marker.points[ii].y = -pt1.x();
              camera_marker.points[ii++].z = -pt1.y();
            }
        } 
    }

  camera_pub.publish(camera_marker);
  point_pub.publish(point_marker);
}

} // namespace sba
