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

// 声明一个类
class LaserScan
{
private:
    ros::NodeHandle node_handle_;           // ros中的句柄
    ros::NodeHandle private_node_;          // ros中的私有句柄
    ros::Subscriber laser_scan_subscriber_; // 声明一个Subscriber
    ros::Publisher feature_scan_publisher_; // 声明一个Publisher

    float * cloudCurvature_;                // 存储每个点的曲率

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
}

LaserScan::~LaserScan()
{
}

// 回调函数
void LaserScan::ScanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
    // 通过ranges中数据的个数进行雷达数据的遍历
    int scan_count = scan_msg->ranges.size();
    ROS_INFO_STREAM("scan_count: " << scan_count );

    float new_scan[scan_count];
    float cloud_curvature[scan_count];
    std::map<int, int> map_index;
    int count = 0;

    // for (int i = 5; i < scan_count - 5; i++)
    for (int i = 0; i < scan_count; i++)
    {
        if(!std::isfinite(scan_msg->ranges[i]) )
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
        float diff_range = new_scan[i-5] + new_scan[i-4] + 
                           new_scan[i-3] + new_scan[i-2] + 
                           new_scan[i-1] + new_scan[i] * 10 + 
                           new_scan[i+1] + new_scan[i+2] + 
                           new_scan[i+3] + new_scan[i+4] + 
                           new_scan[i+5];
        // diffX * diffX + diffY * diffY + diffZ * diffZ;
        cloud_curvature[i] = diff_range * diff_range;
    }

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lesson1_feature_detection_node"); // 节点的名字
    LaserScan laser_scan;

    ros::spin();    // 程序执行到此处时开始进行等待，每次订阅的消息到来都会执行一次ScanCallback()
    return 0;
}