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

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <map>
#include <vector>

#define max_scan_count 1500

struct smoothness_t
{
    float value;
    size_t index;
};

struct by_value
{
    bool operator()(smoothness_t const &left, smoothness_t const &right)
    {
        return left.value < right.value;
    }
};

// 声明一个类
class LaserScan
{
private:
    ros::NodeHandle node_handle_;           // ros中的句柄
    ros::NodeHandle private_node_;          // ros中的私有句柄
    ros::Subscriber laser_scan_subscriber_; // 声明一个Subscriber
    ros::Publisher feature_scan_publisher_; // 声明一个Publisher

    std::vector<smoothness_t> scan_smoothness_; // 存储每个点的曲率与索引
    float *scan_curvature_;                     // 存储每个点的曲率

    float edge_threshold_;

public:
    LaserScan();
    ~LaserScan();
    void ScanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg);
};

// 构造函数
LaserScan::LaserScan() : private_node_("~")
{
    ROS_INFO_STREAM("LaserScan initial.");
    // 将雷达的回调函数与订阅的topic进行绑定
    laser_scan_subscriber_ = node_handle_.subscribe("laser_scan", 1, &LaserScan::ScanCallback, this);
    feature_scan_publisher_ = node_handle_.advertise<sensor_msgs::LaserScan>("feature_scan", 1, this);

    scan_smoothness_.resize(max_scan_count);
    scan_curvature_ = new float[max_scan_count];

    edge_threshold_ = 0.1;
}

LaserScan::~LaserScan()
{
}

// 回调函数
void LaserScan::ScanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
    // 通过ranges中数据的个数进行雷达数据的遍历
    int scan_count = scan_msg->ranges.size();
    ROS_INFO_STREAM("scan_count: " << scan_count);

    std::map<int, int> map_index;
    int count = 0;
    float new_scan[max_scan_count];

    // for (int i = 5; i < scan_count - 5; i++)
    for (int i = 0; i < scan_count; i++)
    {
        if (!std::isfinite(scan_msg->ranges[i]))
        {
            // std::cout << " " << i << " " << scan_msg->ranges[i];
            continue;
        }

        map_index[count] = i;
        new_scan[count] = scan_msg->ranges[i];
        count++;
    }

    std::cout << "count: " << count << std::endl;

    for (int i = 5; i < count - 5; i++)
    {
        float diff_range = new_scan[i - 5] + new_scan[i - 4] +
                           new_scan[i - 3] + new_scan[i - 2] +
                           new_scan[i - 1] + new_scan[i] * 10 +
                           new_scan[i + 1] + new_scan[i + 2] +
                           new_scan[i + 3] + new_scan[i + 4] +
                           new_scan[i + 5];
        // diffX * diffX + diffY * diffY + diffZ * diffZ;
        scan_curvature_[i] = diff_range * diff_range;
        scan_smoothness_[i].value = scan_curvature_[i];
        scan_smoothness_[i].index = i;
    }
    std::cout << map_index.size();

    sensor_msgs::LaserScan corner_scan;
    corner_scan.header = scan_msg->header;
    corner_scan.angle_min = scan_msg->angle_min;
    corner_scan.angle_max = scan_msg->angle_max;
    corner_scan.angle_increment = scan_msg->angle_increment;
    corner_scan.range_min = scan_msg->range_min;
    corner_scan.range_max = scan_msg->range_max;

    corner_scan.ranges.resize(max_scan_count);

    // 每根线分成6部分
    for (int j = 0; j < 6; j++)
    {
        // 从startRingIndex到endRingIndex,分成6分

        int start_index = (0 * (6 - j) + count * j) / 6;
        int end_index = (0 * (5 - j) + count * (j + 1)) / 6 - 1;
        // std::cout << "start_index: " << start_index << " end_index: " << end_index << std::endl;

        if (start_index >= end_index)
            continue;

        // 将这段点云按照曲率从小到大进行排序
        std::sort(scan_smoothness_.begin() + start_index,
                  scan_smoothness_.begin() + end_index, by_value());

        for (int k = end_index; k >= start_index; k--)
        {
            // 最后的点 的曲率最大，如果满足条件，就是角点
            // edgeThreshold为0.1，正圆的曲率为0
            int index = scan_smoothness_[k].index;
            if (scan_smoothness_[k].value > edge_threshold_)
            {
                corner_scan.ranges[map_index[index]] = scan_msg->ranges[map_index[index]];
            }
        }
        
    }
    feature_scan_publisher_.publish(corner_scan);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lesson1_feature_detection_node"); // 节点的名字
    LaserScan laser_scan;

    ros::spin(); // 程序执行到此处时开始进行等待，每次订阅的消息到来都会执行一次ScanCallback()
    return 0;
}