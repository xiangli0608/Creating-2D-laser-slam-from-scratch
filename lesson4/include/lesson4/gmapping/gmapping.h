
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

#ifndef LESSON4_GMAPPING_GMAPPING_H_
#define LESSON4_GMAPPING_GMAPPING_H_

#include <iostream>
#include <chrono>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>

#include "lesson4/gmapping/grid/map.h"
#include "lesson4/gmapping/grid/gridlinetraversal.h"

namespace gmapping
{

#define GMAPPING_UNKNOWN (-1)
#define GMAPPING_FREE (0)
#define GMAPPING_OCC (100)

//从smap的二位数组存储格式，转到一维数组，数组序号也需要转换到一维数组
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

class GMapping
{
private:
    ros::NodeHandle node_handle_;           // ros中的句柄
    ros::NodeHandle private_node_;          // ros中的私有句柄
    ros::Subscriber laser_scan_subscriber_; // 声明一个Subscriber
    ros::Publisher map_publisher_;          // 声明一个Publisher
    ros::Publisher map_publisher_metadata_; // 声明一个Publisher
    nav_msgs::OccupancyGrid map_;           //用来发布map的实体对象

    double max_range_;      // 激光雷达最大的量程
    double max_use_range_;  // 激光雷达最大使用距离

    double xmin_;           // 地图的边界
    double ymin_;
    double xmax_;
    double ymax_;
    double resolution_;     // 地图的分辨率
    double occ_thresh_;     // 大于这个阈值的格子才认为是占用栅格

    bool got_first_scan_;       // 是否获得第一帧scan标志
    std::vector<double> a_cos_; // 保存下来雷达各个角度的cos值
    std::vector<double> a_sin_; // 保存下来雷达各个角度的sin值

    std::vector<GridLineTraversalLine> line_lists_;
    std::vector<Point> hit_lists_;

    std::chrono::steady_clock::time_point start_time_;
    std::chrono::steady_clock::time_point end_time_;
    std::chrono::duration<double> time_used_;

    void InitParams();
    void CreateCache(const sensor_msgs::LaserScan::ConstPtr &scan_msg);
    void PublishMap(const sensor_msgs::LaserScan::ConstPtr &scan_msg);
    void ComputeMap(ScanMatcherMap &map, const sensor_msgs::LaserScan::ConstPtr &scan_msg);

public:
    GMapping();
    ~GMapping();
    void ScanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg);
};

} // end namespace

#endif // LESSON4_GMAPPING_H_