#include <ros/ros.h>
#include <ros/time.h>

// Messages
#include <sparse_bundle_adjustment/Frame.h>
#include <visualization_msgs/Marker.h>
#include <sparse_bundle_adjustment/sba_file_io.h>

#include <sparse_bundle_adjustment/sba.h>
#include <sparse_bundle_adjustment/visualization.h>

#include <map>

#include <ros/callback_queue.h>


using namespace sba;

class SBANode
{
  public:
    SysSBA sba;
    ros::NodeHandle n;
    ros::Subscriber frame_sub;
    ros::Publisher cam_marker_pub;
    ros::Publisher point_marker_pub;
    
    ros::Timer timer_sba;
    ros::Timer timer_vis;
    
    // Mapping from external point index to internal (sba) point index
    std::map<unsigned int, unsigned int> point_indices;
    
    // Mapping from external node index to internal (sba) node index
    std::map<unsigned int, unsigned int> node_indices;
    
    void addFrame(const sba::Frame::ConstPtr& msg);
    void addNode(const sba::CameraNode& msg);
    void addPoint(const sba::WorldPoint& msg);
    void addProj(const sba::Projection& msg);
    void doSBA(/*const ros::TimerEvent& event*/);
    void publishTopics(/*const ros::TimerEvent& event*/);
    SBANode();
};
    
void SBANode::addFrame(const sba::Frame::ConstPtr& msg)
{
  unsigned int i = 0;
  
  ros::Time beginning = ros::Time::now();
  
  // Add all nodes
  for (i=0; i < msg->nodes.size(); i++)
  {
    addNode(msg->nodes[i]);
  }
  
  // Add all points
  for (i=0; i < msg->points.size(); i++)
  {
    addPoint(msg->points[i]);
  }
  
  // Add all projections
  for (i=0; i < msg->projections.size(); i++)
  { 
    addProj(msg->projections[i]);
  }
  
  publishTopics();
  
  // Do SBA Every 10 frames.
  if (sba.nodes.size() % 5 == 0)
  {
    doSBA();
  }
  
  ros::Time end = ros::Time::now();
  
  printf("[SBA] Added frame with %u nodes, %u points, and %u projections (%f s)\n", 
          (unsigned int)msg->nodes.size(), (unsigned int)msg->points.size(), 
          (unsigned int)msg->projections.size(), (end-beginning).toSec());
}

void SBANode::addNode(const sba::CameraNode& msg)
{
  Vector4d trans(msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z, 1.0);
  Quaterniond qrot(msg.transform.rotation.w, msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z);
  
  frame_common::CamParams cam_params;
  cam_params.fx = msg.fx;
  cam_params.fy = msg.fy;
  cam_params.cx = msg.cx;
  cam_params.cy = msg.cy;
  cam_params.tx = msg.baseline;
  
  bool fixed = msg.fixed;
  
  unsigned int newindex = sba.addNode(trans, qrot, cam_params, fixed);
  
  node_indices[msg.index] = newindex;  
}

void SBANode::addPoint(const sba::WorldPoint& msg)
{
  Vector4d point(msg.x, msg.y, msg.z, msg.w);
  unsigned int newindex = sba.addPoint(point);
  
  point_indices[msg.index] = newindex;
}

void SBANode::addProj(const sba::Projection& msg)
{
  int camindex = node_indices[msg.camindex];
  int pointindex = point_indices[msg.pointindex];
  Vector3d keypoint(msg.u, msg.v, msg.d);
  bool stereo = msg.stereo;
  bool usecovariance = msg.usecovariance;
  Eigen::Matrix3d covariance;
  covariance << msg.covariance[0], msg.covariance[1], msg.covariance[2],
                msg.covariance[3], msg.covariance[4], msg.covariance[5],
                msg.covariance[6], msg.covariance[7], msg.covariance[8];
  
  // Make sure it's valid before adding it.
  if (pointindex < (int)sba.tracks.size() && camindex < (int)sba.nodes.size())
  {
    sba.addProj(camindex, pointindex, keypoint, stereo);
    if (usecovariance)
      sba.setProjCovariance(camindex, pointindex, covariance);
  }
  else
  {
    ROS_INFO("Failed to add projection: C: %d, P: %d, Csize: %d, Psize: %d", 
            camindex, pointindex,(int)sba.nodes.size(),(int)sba.tracks.size());       
  }
}

void SBANode::doSBA(/*const ros::TimerEvent& event*/)
{
  unsigned int projs = 0;
  // For debugging.
  for (int i = 0; i < (int)sba.tracks.size(); i++)
  {
    projs += sba.tracks[i].projections.size();
  }
  ROS_INFO("SBA Nodes: %d, Points: %d, Projections: %d", (int)sba.nodes.size(),
    (int)sba.tracks.size(), projs);
  
  if (sba.nodes.size() > 0)
  {
    // Copied from vslam.cpp: refine()
    sba.doSBA(10, 1.0e-4, SBA_SPARSE_CHOLESKY);
    
    double cost = sba.calcRMSCost();
    
    if (isnan(cost) || isinf(cost)) // is NaN?
    {
      ROS_INFO("NaN cost!");  
    }
    else
    { 
      if (sba.calcRMSCost() > 4.0)
        sba.doSBA(10, 1.0e-4, SBA_SPARSE_CHOLESKY);  // do more
      if (sba.calcRMSCost() > 4.0)
        sba.doSBA(10, 1.0e-4, SBA_SPARSE_CHOLESKY);  // do more
    }
  }
}

void SBANode::publishTopics(/*const ros::TimerEvent& event*/)
{
  // Visualization
  if (cam_marker_pub.getNumSubscribers() > 0 || point_marker_pub.getNumSubscribers() > 0)
  { 
     drawGraph(sba, cam_marker_pub, point_marker_pub);
  }
}

SBANode::SBANode()
{
  // Advertise visualization topics.
  cam_marker_pub = n.advertise<visualization_msgs::Marker>("/sba/cameras", 1);
  point_marker_pub = n.advertise<visualization_msgs::Marker>("/sba/points", 1);

  //timer_sba = n.createTimer(ros::Duration(5.0), &SBANode::doSBA, this);
  //timer_vis = n.createTimer(ros::Duration(1.0), &SBANode::publishTopics, this);
  
  // Subscribe to topics.
  frame_sub = n.subscribe<sba::Frame>("/sba/frames", 5000, &SBANode::addFrame, this);
  
  sba.useCholmod(true);
  
  printf("[SBA] Initialization complete.\n");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sba_node");
  SBANode sbanode;

  ros::spin();
}

