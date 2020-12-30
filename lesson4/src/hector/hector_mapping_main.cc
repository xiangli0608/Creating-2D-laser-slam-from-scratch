/*
 * Copyright 2020 The Project Author: lixiang
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

#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/GetMap.h>

#include <boost/thread.hpp>

#include "lesson4/hector_mapping/hector_mapping.h"

namespace hector_mapping
{

class HectorMapping
{
private:
    ros::NodeHandle node_handle_;           // ros中的句柄
    ros::NodeHandle private_node_;          // ros中的私有句柄
    ros::Subscriber laser_scan_subscriber_; // 声明一个Subscriber
    ros::Publisher map_publisher_;         // 声明一个Publisher
    ros::Publisher mapMetadata_publisher_;         // 声明一个Publisher

    void InitParams();

public:
    HectorMapping(/* args */);
    ~HectorMapping();
    void ScanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg);
};

HectorMapping::HectorMapping(/* args */) : private_node_("~")
{
    // \033[1;32m，\033[0m 终端显示成绿色
    ROS_INFO_STREAM("\033[1;32m----> lesson4 hector mapping started.\033[0m");

    laser_scan_subscriber_ = node_handle_.subscribe(
        "laser_scan", 1, &HectorMapping::ScanCallback, this);

    std::string mapTopic_ = "map";
    std::string mapMetaTopicStr(mapTopic_);
    mapMetaTopicStr.append("_metadata");

    std::cout << "mapTopic_: " << mapTopic_ << std::endl;
    std::cout << "mapMetaTopicStr: " << mapMetaTopicStr << std::endl;

    map_publisher_ = node_handle_.advertise<nav_msgs::OccupancyGrid>(mapTopic_, 1, true);
    mapMetadata_publisher_ = node_handle_.advertise<nav_msgs::MapMetaData>(mapMetaTopicStr, 1, true);

    // 参数初始化
    InitParams();

}

HectorMapping::~HectorMapping()
{
}

/*
 * ros与csm的参数初始化
 */
void HectorMapping::InitParams()
{

}

void HectorMapping::ScanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{

}

} // end namespace


int main(int argc, char **argv)
{
    ros::init(argc, argv, "lesson4_hector_mapping");

    hector_mapping::HectorMapping hector_mapping;

    ros::spin();

    return (0);
}
