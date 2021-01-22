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

#include "lesson4/hector_mapping/hector_mapping.h"

namespace hector_mapping
{

HectorMapping::HectorMapping() : private_node_("~")
{
    ROS_INFO_STREAM("\033[1;32m----> Make Hector Map by no move started.\033[0m");

    laser_scan_subscriber_ = node_handle_.subscribe(
        "laser_scan", 1, &HectorMapping::ScanCallback, this);

    map_publisher_ = node_handle_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
    map_publisher_metadata_ = node_handle_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);

    InitParams();
    
    // 地图始终存在
    // hector_map_ = new hectorslam::GridMap(resolution_, map_size_, offset_);
}

HectorMapping::~HectorMapping()
{
    // delete hector_map_;
}

// ros的参数初始化
void HectorMapping::InitParams()
{
    if (!private_node_.getParam("xmin", xmin_))
        xmin_ = -40.0;
    if (!private_node_.getParam("ymin", ymin_))
        ymin_ = -40.0;
    if (!private_node_.getParam("xmax", xmax_))
        xmax_ = 40.0;
    if (!private_node_.getParam("ymax", ymax_))
        ymax_ = 40.0;
    if (!private_node_.getParam("delta", resolution_))
        resolution_ = 0.05;

    map_size_[0] = (xmax_ - xmin_) / resolution_;
    map_size_[1] = (ymax_ - ymin_) / resolution_;

    // hector定义的offset做地图左上角的物理坐标
    offset_[0] = xmin_;
    offset_[1] = ymax_;

    /***************** 对map_进行初始化 *****************/

    // 地图的分辨率为0.05m,代表一个格子的距离是0.05m
    map_.info.resolution = resolution_;

    // 地图图片像素的大小, width为地图的宽度是多少个像素
    map_.info.width = map_size_[0];
    map_.info.height = map_size_[1];

    // 地图左下角的点对应的物理坐标
    map_.info.origin.position.x = xmin_;
    map_.info.origin.position.y = ymin_;

    // 对数组进行初始化, 数组的大小为实际像素的个数
    map_.data.resize(map_.info.width * map_.info.height);
}

// 回调函数
void HectorMapping::ScanCallback(const sensor_msgs::LaserScan &scan)
{
    if (!got_first_scan_)       // 如果是第一次接收scan
    {
        got_first_scan_ = true; // 改变第一帧的标志位
        map_frame_ = scan.header.frame_id;
    }

    // 0 初始化与申请内存空间
    start_time_ = std::chrono::steady_clock::now();

    // 当雷达数据到来时新建地图
    hector_map_ = new hectorslam::GridMap(resolution_, map_size_, offset_);

    end_time_ = std::chrono::steady_clock::now();
    time_used_ = std::chrono::duration_cast<std::chrono::duration<double>>(end_time_ - start_time_);
    std::cout << "\n初始化与申请内存空间用时: " << time_used_.count() << " 秒。" << std::endl;

    // 1 将雷达的数据类型转成hector需要的格式
    start_time_ = std::chrono::steady_clock::now();

    ROSLaserScanToDataContainer(scan, laserScan_container_, resolution_);
    
    end_time_ = std::chrono::steady_clock::now();
    time_used_ = std::chrono::duration_cast<std::chrono::duration<double>>(end_time_ - start_time_);
    std::cout << "转换数据用时: " << time_used_.count() << " 秒。" << std::endl;

    // 2 将雷达的数据类型写成hector的地图
    start_time_ = std::chrono::steady_clock::now();

    ComputeMap();

    end_time_ = std::chrono::steady_clock::now();
    time_used_ = std::chrono::duration_cast<std::chrono::duration<double>>(end_time_ - start_time_);
    std::cout << "计算hector地图用时: " << time_used_.count() << " 秒。" << std::endl;

    // 3 将hector的地图转换成ros中的地图格式并发布出去
    start_time_ = std::chrono::steady_clock::now();

    PublishMap();

    end_time_ = std::chrono::steady_clock::now();
    time_used_ = std::chrono::duration_cast<std::chrono::duration<double>>(end_time_ - start_time_);
    std::cout << "转换成ros地图用时: " << time_used_.count() << " 秒。" << std::endl;

    // 4 析构与释放内存空间
    start_time_ = std::chrono::steady_clock::now();

    delete hector_map_;

    end_time_ = std::chrono::steady_clock::now();
    time_used_ = std::chrono::duration_cast<std::chrono::duration<double>>(end_time_ - start_time_);
    std::cout << "析构与释放内存空间用时: " << time_used_.count() << " 秒。" << std::endl;
}

// 将ROS中的雷达数据格式转成hector中定义的格式
void HectorMapping::ROSLaserScanToDataContainer(const sensor_msgs::LaserScan &scan,
                                                hectorslam::DataContainer &dataContainer,
                                                float resolution)
{
    size_t size = scan.ranges.size();

    float angle = scan.angle_min;

    dataContainer.clear();

    dataContainer.setOrigo(Eigen::Vector2f::Zero());

    float maxRangeForContainer = scan.range_max - 0.1f;

    for (size_t i = 0; i < size; ++i)
    {
        float dist = scan.ranges[i];

        if ((dist > scan.range_min) && (dist < maxRangeForContainer))
        {
            // dist *= resolution; ///! 将实际物理尺度转换到地图尺度）
            dataContainer.add(Eigen::Vector2f(cos(angle) * dist, sin(angle) * dist));
            // std::cout << "x: " << cos(angle) * dist << " y: " << sin(angle) * dist << std::endl;
        }

        angle += scan.angle_increment;
    }
}

// 将雷达数据写成hector的地图
void HectorMapping::ComputeMap()
{
    Eigen::Vector3f robotPoseWorld(0, 0, 0);
    hector_map_mutex_.lock();
    hector_map_->updateByScanJustOnce(laserScan_container_, robotPoseWorld);
    hector_map_mutex_.unlock();
}

// 将hector的地图转成ROS格式的地图并发布出去
void HectorMapping::PublishMap()
{
    int sizeX = hector_map_->getSizeX();
    int sizeY = hector_map_->getSizeY();
    int size = sizeX * sizeY;

    std::vector<int8_t> &data = map_.data;

    //std::vector contents are guaranteed to be contiguous, use memset to set all to unknown to save time in loop
    memset(&data[0], -1, sizeof(int8_t) * size);

    ros_map_mutex_.lock();

    for (int i = 0; i < size; ++i)
    {
        if (hector_map_->isFree(i))
        {
            data[i] = 0;
        }
        else if (hector_map_->isOccupied(i))
        {
            data[i] = 100;
        }
    }

    ros_map_mutex_.unlock();

    // 添加当前的时间戳
    map_.header.stamp = ros::Time::now();
    map_.header.frame_id = map_frame_;

    // 发布map和map_metadata
    map_publisher_.publish(map_);
    map_publisher_metadata_.publish(map_.info);
}

} // namespace hector_mapping

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lesson4_make_hector_map");

    hector_mapping::HectorMapping hector_mapping;

    ros::spin();

    return (0);
}