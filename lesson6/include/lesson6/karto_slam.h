/*
 * slam_karto
 * Copyright (c) 2008, Willow Garage, Inc.
 *
 * THE WORK (AS DEFINED BELOW) IS PROVIDED UNDER THE TERMS OF THIS CREATIVE
 * COMMONS PUBLIC LICENSE ("CCPL" OR "LICENSE"). THE WORK IS PROTECTED BY
 * COPYRIGHT AND/OR OTHER APPLICABLE LAW. ANY USE OF THE WORK OTHER THAN AS
 * AUTHORIZED UNDER THIS LICENSE OR COPYRIGHT LAW IS PROHIBITED.
 *
 * BY EXERCISING ANY RIGHTS TO THE WORK PROVIDED HERE, YOU ACCEPT AND AGREE TO
 * BE BOUND BY THE TERMS OF THIS LICENSE. THE LICENSOR GRANTS YOU THE RIGHTS
 * CONTAINED HERE IN CONSIDERATION OF YOUR ACCEPTANCE OF SUCH TERMS AND
 * CONDITIONS.
 *
 */

/* Author: Brian Gerkey */

#ifndef LESSON6_KARTO_SLAM_H_
#define LESSON6_KARTO_SLAM_H_

#include "ros/ros.h"
#include "ros/console.h"
#include "message_filters/subscriber.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "visualization_msgs/MarkerArray.h"

#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/GetMap.h"

#include "open_karto/Mapper.h"

#include <boost/thread.hpp>

#include <string>
#include <map>
#include <vector>

// back end
#include "spa_solver/spa_solver.h"
#include "g2o_solver/g2o_solver.h"
#include "ceres_solver/ceres_solver.h"
#include "gtsam_solver/gtsam_solver.h"

// 从smap的二位数组存储格式，转到一维数组，数组序号也需要转换到一维数组
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

class SlamKarto
{
public:
    SlamKarto();
    ~SlamKarto();
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr &scan);

private:
    void InitParams();
    bool getOdomPose(karto::Pose2 &karto_pose, const ros::Time &t);
    karto::LaserRangeFinder *getLaser(const sensor_msgs::LaserScan::ConstPtr &scan);
    bool addScan(karto::LaserRangeFinder *laser,
                 const sensor_msgs::LaserScan::ConstPtr &scan,
                 karto::Pose2 &karto_pose);
    bool updateMap();

    void publishLoop(double transform_publish_period);
    void publishTransform();
    void publishGraphVisualization();
    karto::ScanSolver* CreateSolver(std::string solver_type);

    // time for tolerance on the published transform,
    // basically defines how long a map->odom transform is good for
    ros::Duration transform_tolerance_;

    // ROS handles
    ros::NodeHandle node_;
    tf::TransformListener tf_;
    tf::TransformBroadcaster *tfB_;

    message_filters::Subscriber<sensor_msgs::LaserScan> *scan_filter_sub_;
    tf::MessageFilter<sensor_msgs::LaserScan> *scan_filter_;

    ros::Publisher sst_;
    ros::Publisher sstm_;
    ros::Publisher marker_publisher_;

    // The map that will be published / send to service callers
    nav_msgs::GetMap::Response map_;

    // Storage for ROS parameters
    std::string odom_frame_;
    std::string map_frame_;
    std::string base_frame_;
    int throttle_scans_;
    ros::Duration map_update_interval_;
    double resolution_;
    double transform_publish_period_;
    double use_scan_range_;
    boost::mutex map_mutex_;
    boost::mutex map_to_odom_mutex_;

    // Karto bookkeeping
    karto::Mapper *mapper_;
    karto::Dataset *dataset_;

    std::map<std::string, karto::LaserRangeFinder *> lasers_;
    std::map<std::string, bool> lasers_inverted_;

    // Internal state
    bool got_map_;
    int laser_count_;
    boost::thread *transform_thread_;
    tf::Transform map_to_odom_;
    uint marker_count_;

    bool use_back_end_;
    std::string solver_type_;
    karto::ScanSolver *solver_;
};

#endif
