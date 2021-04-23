#ifndef VSLAM_SYSTEM_VISUALIZATION_H
#define VSLAM_SYSTEM_VISUALIZATION_H

#include <ros/ros.h>
#include <sparse_bundle_adjustment/sba.h>
#include <visualization_msgs/Marker.h>

namespace sba {

// draw the graph on rviz
void drawGraph(const SysSBA &sba, const ros::Publisher &camera_pub,
               const ros::Publisher &point_pub, int decimation = 1, int bicolor = 0);

} // namespace sba

#endif
