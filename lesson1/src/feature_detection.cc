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
#include <chrono>

#define max_scan_count 1500 // 雷达数据个数的最大值

struct smoothness_t
{
    float value;
    size_t index;
};

// 排序的规则,从小到大进行排序
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

    float edge_threshold_; // 提取角点的阈值

public:
    LaserScan();
    ~LaserScan();
    void ScanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg);
};

// 构造函数
LaserScan::LaserScan() : private_node_("~")
{
    // \033[1;32m，\033[0m 终端显示成绿色
    ROS_INFO_STREAM("\033[1;32m----> Feature Extraction Started.\033[0m");

    // 将雷达的回调函数与订阅的topic进行绑定
    laser_scan_subscriber_ = node_handle_.subscribe("laser_scan", 1, &LaserScan::ScanCallback, this);
    // 将提取后的点发布到 feature_scan 这个topic
    feature_scan_publisher_ = node_handle_.advertise<sensor_msgs::LaserScan>("feature_scan", 1, this);

    // 将提取角点的阈值设置为1.0
    edge_threshold_ = 1.0;
}

LaserScan::~LaserScan()
{
}

// 回调函数
void LaserScan::ScanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
    std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();

    std::vector<smoothness_t> scan_smoothness_(max_scan_count); // 存储每个点的曲率与索引
    float *scan_curvature_ = new float[max_scan_count];         // 存储每个点的曲率

    std::map<int, int> map_index;   // 有效点的索引 对应的 scan实际的索引
    int count = 0;                  // 有效点的索引
    float new_scan[max_scan_count]; // 存储scan数据的距离值

    // 通过ranges中数据的个数进行雷达数据的遍历
    int scan_count = scan_msg->ranges.size();
    // ROS_INFO_STREAM("scan_count: " << scan_count);

    // 去处inf或者nan点,保存有效点
    for (int i = 0; i < scan_count; i++)
    {
        if (!std::isfinite(scan_msg->ranges[i]))
        {
            // std::cout << " " << i << " " << scan_msg->ranges[i];
            continue;
        }

        // 这点在原始数据中的索引为i，在new_scan中的索引为count
        map_index[count] = i;
        // new_scan中保存了有效点的距离值
        new_scan[count] = scan_msg->ranges[i];
        count++;
    }

    // std::cout << "count: " << count << std::endl;

    // 计算曲率值, 通过当前点前后5个点距离值的偏差程度来代表曲率
    // 如果是球面, 则当前点周围的10个点的距离之和 减去 当前点距离的10倍 应该等于0
    for (int i = 5; i < count - 5; i++)
    {
        float diff_range = new_scan[i - 5] + new_scan[i - 4] +
                           new_scan[i - 3] + new_scan[i - 2] +
                           new_scan[i - 1] - new_scan[i] * 10 +
                           new_scan[i + 1] + new_scan[i + 2] +
                           new_scan[i + 3] + new_scan[i + 4] +
                           new_scan[i + 5];
        // diffX * diffX + diffY * diffY
        scan_curvature_[i] = diff_range * diff_range;
        scan_smoothness_[i].value = scan_curvature_[i];
        scan_smoothness_[i].index = i;
    }

    // 声明一个临时的sensor_msgs::LaserScan变量,用于存储特征提取后的scan数据,并发布出去,在rviz中进行显示
    sensor_msgs::LaserScan corner_scan;
    corner_scan.header = scan_msg->header;
    corner_scan.angle_min = scan_msg->angle_min;
    corner_scan.angle_max = scan_msg->angle_max;
    corner_scan.angle_increment = scan_msg->angle_increment;
    corner_scan.range_min = scan_msg->range_min;
    corner_scan.range_max = scan_msg->range_max;

    // 对float[] 进行初始化
    corner_scan.ranges.resize(max_scan_count);

    // 进行角点的提取,将完整的scan分成6部分,每部分提取20个角点
    for (int j = 0; j < 6; j++)
    {
        int start_index = (0 * (6 - j) + count * j) / 6;
        int end_index = (0 * (5 - j) + count * (j + 1)) / 6 - 1;
        // std::cout << "start_index: " << start_index << " end_index: " << end_index << std::endl;

        if (start_index >= end_index)
            continue;

        // 将这段点云按照曲率从小到大进行排序
        std::sort(scan_smoothness_.begin() + start_index,
                  scan_smoothness_.begin() + end_index, by_value());

        int largestPickedNum = 0;
        // 最后的点 的曲率最大，如果满足条件，就是角点
        for (int k = end_index; k >= start_index; k--)
        {
            int index = scan_smoothness_[k].index;
            if (scan_smoothness_[k].value > edge_threshold_)
            {
                // 每一段最多只取20个角点
                largestPickedNum++;
                if (largestPickedNum <= 20)
                {
                    corner_scan.ranges[map_index[index]] = scan_msg->ranges[map_index[index]];
                }
                else
                {
                    break;
                }
            }
        }
    }

    // 将提取后的scan数据发布出去
    feature_scan_publisher_.publish(corner_scan);

    std::chrono::steady_clock::time_point end_time = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(end_time - start_time);
    // std::cout<<"处理一次数据用时: "<< time_used.count() << " 秒。" << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lesson1_feature_detection_node"); // 节点的名字
    LaserScan laser_scan;

    ros::spin(); // 程序执行到此处时开始进行等待，每次订阅的消息到来都会执行一次ScanCallback()
    return 0;
}