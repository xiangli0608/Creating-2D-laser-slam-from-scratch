
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

#include "lesson4/gmapping/gmapping.h"

namespace gmapping
{

// 构造函数
GMapping::GMapping() : private_node_("~")
{
    // \033[1;32m，\033[0m 终端显示成绿色
    ROS_INFO_STREAM("\033[1;32m----> Make Gmapping Map by no move started.\033[0m");

    laser_scan_subscriber_ = node_handle_.subscribe(
        "laser_scan", 1, &GMapping::ScanCallback, this);

    map_publisher_ = node_handle_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
    map_publisher_metadata_ = node_handle_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);

    // 参数初始化
    InitParams();
}

GMapping::~GMapping()
{
}

// ros的参数初始化
void GMapping::InitParams()
{
    if (!private_node_.getParam("maxRange", max_range_))
        max_range_ = 30 - 0.01;
    if (!private_node_.getParam("maxUrange", max_use_range_))
        max_use_range_ = 25;

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
    if (!private_node_.getParam("occ_thresh", occ_thresh_))
        occ_thresh_ = 0.25;

    /********************************************/
    // 对map_进行初始化
    // map_.header.frame_id = "map";

    // 地图的分辨率为0.05m,代表一个格子的距离是0.05m
    map_.info.resolution = resolution_;

    // 地图图片像素的大小, width为地图的宽度是多少个像素
    map_.info.width = (xmax_ - xmin_) / map_.info.resolution;
    map_.info.height = (ymax_ - ymin_) / map_.info.resolution;

    // 地图左下角的点对应的物理坐标
    map_.info.origin.position.x = xmin_;
    map_.info.origin.position.y = ymin_;

    // 对数组进行初始化, 数组的大小为实际像素的个数
    map_.data.resize(map_.info.width * map_.info.height);
    /********************************************/

    got_first_scan_ = false;
}

// 回调函数 进行数据处理
void GMapping::ScanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{

    static ros::Time last_map_update(0, 0); //存储上一次地图更新的时间

    if (!got_first_scan_) //如果是第一次接收scan
    {
        // 将雷达各个角度的sin与cos值保存下来，以节约计算量
        CreateCache(scan_msg);
        got_first_scan_ = true; //改变第一帧的标志位
    }

    start_time_ = std::chrono::steady_clock::now();

    // 计算当前雷达数据对应的栅格地图并发布出去
    PublishMap(scan_msg);

    end_time_ = std::chrono::steady_clock::now();
    time_used_ = std::chrono::duration_cast<std::chrono::duration<double>>(end_time_ - start_time_);
    std::cout << "\n转换一次地图用时: " << time_used_.count() << " 秒。" << std::endl;

    last_map_update = scan_msg->header.stamp;
}

// 雷达数据间的角度是固定的，因此可以将对应角度的cos与sin值缓存下来，不用每次都计算
void GMapping::CreateCache(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
    a_cos_.clear();
    a_sin_.clear();
    double angle;

    for (unsigned int i = 0; i < scan_msg->ranges.size(); i++)
    {
        angle = scan_msg->angle_min + i * scan_msg->angle_increment;
        a_cos_.push_back(cos(angle));
        a_sin_.push_back(sin(angle));
    }
}

// 计算当前雷达数据对应的栅格地图并发布出去
void GMapping::PublishMap(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
    // 地图的中点
    Point center;
    center.x = (xmin_ + xmax_) / 2.0;
    center.y = (ymin_ + ymax_) / 2.0;

    // ScanMatcherMap为GMapping中存储地图的数据类型
    ScanMatcherMap gmapping_map_(center, xmin_, ymin_, xmax_, ymax_, resolution_);
    
    // 使用当前雷达数据更新GMapping地图中栅格的值
    ComputeMap(gmapping_map_, scan_msg);

    // 将gmapping_map_中的存储的栅格值 赋值到 ros的map中
    for (int x = 0; x < gmapping_map_.getMapSizeX(); x++)
    {
        for (int y = 0; y < gmapping_map_.getMapSizeY(); y++)
        {
            IntPoint p(x, y);
            // 获取这点栅格的值，只有大于occ_thresh_时才认为是占用
            double occ = gmapping_map_.cell(p); 

            // 未知
            if (occ < 0)
                map_.data[MAP_IDX(map_.info.width, x, y)] = GMAPPING_UNKNOWN;
            // 占用
            else if (occ > occ_thresh_) // 默认0.25
                map_.data[MAP_IDX(map_.info.width, x, y)] = GMAPPING_OCC;
            // 空闲
            else
                map_.data[MAP_IDX(map_.info.width, x, y)] = GMAPPING_FREE;
        }
    }

    // 添加当前的时间戳
    map_.header.stamp = ros::Time::now();
    map_.header.frame_id = scan_msg->header.frame_id;

    // 发布map和map_metadata
    map_publisher_.publish(map_);
    map_publisher_metadata_.publish(map_.info);
}

// 使用当前雷达数据更新GMapping地图中栅格的值
void GMapping::ComputeMap(ScanMatcherMap &map, const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
    line_lists_.clear();
    hit_lists_.clear();

    // lp为地图坐标系下的激光雷达坐标系的位姿
    OrientedPoint lp(0, 0, 0.0);

    // 将位姿lp转换成地图坐标系下的位置
    IntPoint p0 = map.world2map(lp);

    // 地图的有效区域(地图坐标系)
    HierarchicalArray2D<PointAccumulator>::PointSet activeArea;

    // 通过激光雷达的数据，找出地图的有效区域
    for (unsigned int i = 0; i < scan_msg->ranges.size(); i++)
    {
        // 排除错误的激光点
        double d = scan_msg->ranges[i];
        if (d > max_range_ || d == 0.0 || !std::isfinite(d))
            continue;
        if (d > max_use_range_)
            d = max_use_range_;

        // p1为激光雷达的数据点在地图坐标系下的坐标
        Point phit = lp;
        phit.x += d * a_cos_[i];
        phit.y += d * a_sin_[i];
        IntPoint p1 = map.world2map(phit);

        // 使用bresenham算法来计算 从激光位置到激光点 要经过的栅格的坐标
        GridLineTraversalLine line;
        GridLineTraversal::gridLine(p0, p1, &line);
        // 将line保存起来以备后用
        line_lists_.push_back(line);

        // 计算活动区域的大小
        for (int i = 0; i < line.num_points - 1; i++)
        {
            activeArea.insert(map.storage().patchIndexes(line.points[i]));
        }
        // 如果d<m_usableRange则需要把击中点也算进去 说明这个值是好的。
        // 同时如果d==max_use_range_ 那么说明这个值只用来进行标记空闲区域　不用来进行标记障碍物
        if (d < max_use_range_)
        {
            IntPoint cp = map.storage().patchIndexes(p1);
            activeArea.insert(cp);
            hit_lists_.push_back(phit);
        }
    }

    // 为activeArea分配内存
    map.storage().setActiveArea(activeArea, true);
    map.storage().allocActiveArea();

    // 在map上更新空闲点
    for (auto line : line_lists_)
    {
        // 更新空闲位置
        for (int k = 0; k < line.num_points - 1; k++)
        {
            // 未击中，就不记录击中的位置了，所以传入参数Point(0,0)
            map.cell(line.points[k]).update(false, Point(0, 0));
        }
    }
    // 在map上添加hit点
    for (auto hit : hit_lists_)
    {
        IntPoint p1 = map.world2map(hit);
        map.cell(p1).update(true, hit);
    }
}

} // namespace gmapping

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lesson4_make_gmapping_map");

    gmapping::GMapping gmapping;

    ros::spin();

    return (0);
}