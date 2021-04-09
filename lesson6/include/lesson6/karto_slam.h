/*
 * Copyright 2021 The Project Author: lixiang
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

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

// #include "spa_solver.h"

#include <boost/thread.hpp>

#include <string>
#include <map>
#include <vector>

//从smap的二位数组存储格式，转到一维数组，数组序号也需要转换到一维数组
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
    unsigned marker_count_;
    bool inverted_laser_;

    // SpaSolver *solver_;

};

#endif
