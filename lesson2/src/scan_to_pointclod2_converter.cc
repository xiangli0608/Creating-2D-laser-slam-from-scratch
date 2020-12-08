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

#include "lesson2/scan_to_pointclod2_converter.h"
#include <limits>

ScanToPointCloud2Converter::ScanToPointCloud2Converter() : private_node_("~")
{
    // \033[1;32m，\033[0m 终端显示成绿色
    ROS_INFO_STREAM("\033[1;32m----> Scan to PointCloud2 Converter.\033[0m");

    laser_scan_subscriber_ = node_handle_.subscribe(
        "laser_scan", 1, &ScanToPointCloud2Converter::ScanCallback, this);

    // 注意，这里的发布器，发布的数据类型为 pcl::PointCloud<PointT>
    // ros中自动做了 pcl::PointCloud<PointT> 到 sensor_msgs/PointCloud2 的数据类型的转换
    pointcloud2_publisher_ = node_handle_.advertise<PointCloudT>(
        "pointcloud2_converted", 1, this);

    // 无效点的值设置为nan
    invalid_point_.x = std::numeric_limits<float>::quiet_NaN();
    invalid_point_.y = std::numeric_limits<float>::quiet_NaN();
    invalid_point_.z = std::numeric_limits<float>::quiet_NaN();
}

ScanToPointCloud2Converter::~ScanToPointCloud2Converter()
{
    ROS_INFO("Destroying ScanToPointCloud2Converter");
}

void ScanToPointCloud2Converter::ScanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
    // 声明一个 pcl::PointCloud<PointT> 类型的指针
    PointCloudT::Ptr cloud_msg = boost::shared_ptr<PointCloudT>(new PointCloudT());

    // 对容器进行初始化
    cloud_msg->points.resize(scan_msg->ranges.size());

    for (unsigned int i = 0; i < scan_msg->ranges.size(); ++i)
    {
        // 首先声明一个 cloud_msg第i个点的 引用
        PointT &point_tmp = cloud_msg->points[i];
        // 获取scan的第i个点的距离值
        float range = scan_msg->ranges[i];

        // 将 inf 与 nan 点 设置为无效点
        if (!std::isfinite(range))
        {
            // std::cout << " " << i << " " << scan_msg->ranges[i];
            point_tmp = invalid_point_;
            continue;
        }

        // 有些雷达驱动会将无效点设置成 range_max+1
        // 所以要根据雷达的range_min与range_max进行有效值的判断
        if (range > scan_msg->range_min && range < scan_msg->range_max)
        {
            // 获取第i个点对应的角度
            float angle = scan_msg->angle_min + i * scan_msg->angle_increment;
            // 获取第i个点在笛卡尔坐标系下的坐标
            point_tmp.x = range * cos(angle);
            point_tmp.y = range * sin(angle);
            point_tmp.z = 0.0;
        }
        else
            // 无效点
            point_tmp = invalid_point_;
    }

    cloud_msg->width = scan_msg->ranges.size();
    cloud_msg->height = 1;
    cloud_msg->is_dense = false; // contains nans
    // 将scan_msg的消息头 赋值到 PointCloudT的消息头
    pcl_conversions::toPCL(scan_msg->header, cloud_msg->header);

    // 由于ros中自动做了 pcl::PointCloud<PointT> 到 sensor_msgs/PointCloud2 的数据类型的转换
    // 所以这里直接发布 pcl::PointCloud<PointT> 即可
    pointcloud2_publisher_.publish(cloud_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lesson2_scan_to_cloud_converter_node"); // 节点的名字
    ScanToPointCloud2Converter scan_to_cloud_converter;

    ros::spin(); // 程序执行到此处时开始进行等待，每次订阅的消息到来都会执行一次ScanCallback()
    return 0;
}