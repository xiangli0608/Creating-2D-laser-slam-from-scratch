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

#ifndef LESSON4_HECTOR_SLAM_H_
#define LESSON4_HECTOR_SLAM_H_

#include "ros/ros.h"

#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"

#include "sensor_msgs/LaserScan.h"
#include "std_msgs/String.h"
#include "nav_msgs/GetMap.h"
#include "laser_geometry/laser_geometry.h"

#include "map/GridMap.h"
#include "slam_main/HectorSlamProcessor.h"
#include "scan/DataPointContainer.h"
#include "util/MapLockerInterface.h"
#include "util/HectorMapMutex.h"
#include "util/PoseInfoContainer.h"

#include <boost/thread.hpp>
#include <chrono>

class MapPublisherContainer
{
public:
    ros::Publisher mapPublisher_;
    ros::Publisher mapMetadataPublisher_;
    nav_msgs::GetMap::Response map_;
    ros::ServiceServer dynamicMapServiceServer_;
};

class HectorMappingRos
{
public:
    HectorMappingRos();
    ~HectorMappingRos();
    void scanCallback(const sensor_msgs::LaserScan &scan);
    void publishMapLoop(double p_map_pub_period_);

private:
    void InitParams();
    void setMapInfo(nav_msgs::GetMap::Response &map_, const hectorslam::GridMap &gridMap);
    void publishMap(MapPublisherContainer &map_, const hectorslam::GridMap &gridMap, ros::Time timestamp, MapLockerInterface *mapMutex = 0);
    bool rosPointCloudToDataContainer(const sensor_msgs::PointCloud &pointCloud, const tf::StampedTransform &laserTransform, hectorslam::DataContainer &dataContainer, float scaleToMap);

protected:
    ros::NodeHandle node_handle_;  // ros中的句柄
    ros::NodeHandle private_node_; // ros中的私有句柄
    ros::Subscriber laser_scan_subscriber_;
    ros::Publisher odometryPublisher_;

    std::vector<MapPublisherContainer> mapPubContainer;

    tf::TransformListener tf_;
    tf::TransformBroadcaster *tfB_;
    tf::Transform map_to_odom_;
    tf::StampedTransform laserTransform_;
    laser_geometry::LaserProjection projector_;
    sensor_msgs::PointCloud laser_point_cloud_; // 点云格式的雷达数据

    boost::thread *map_publish_thread_;

    hectorslam::HectorSlamProcessor *slamProcessor;
    hectorslam::DataContainer laserScanContainer;
    PoseInfoContainer poseInfoContainer_; // 用于记录轨迹

    int lastGetMapUpdateIndex;

    // Hector中的各种参数

    std::string p_base_frame_; // base_frame
    std::string p_map_frame_;  // map的frame
    std::string p_odom_frame_; // 里程计odom的frame
    std::string p_scan_frame_; // 激光scan的frame
    std::string p_scan_topic_; // 激光scan的topic

    // Parameters related to publishing the scanmatcher pose directly via tf
    std::string p_tf_map_scanmatch_transform_frame_name_;
    bool pub_map_to_baselink_tf_;
    bool p_pub_map_odom_transform_;
    bool p_pub_odometry_;
    int p_scan_subscriber_queue_size_;
    double p_use_max_scan_range_;

    // 地图更新参数
    double p_update_factor_free_;
    double p_update_factor_occupied_;
    double p_map_update_distance_threshold_;
    double p_map_update_angle_threshold_;

    // map parameters --- resolution / size / init pose / map levels 
    double p_map_resolution_;
    int p_map_size_;
    double p_map_start_x_;
    double p_map_start_y_;
    int p_map_multi_res_levels_;
    double p_map_pub_period_;

    bool p_timing_output_;

    // laser data filter
    float p_sqr_laser_min_dist_;
    float p_sqr_laser_max_dist_;
    float p_laser_z_min_value_;
    float p_laser_z_max_value_;

    std::chrono::steady_clock::time_point start_time_;
    std::chrono::steady_clock::time_point end_time_;
    std::chrono::duration<double> time_used_;
};

#endif
