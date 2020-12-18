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

#ifndef LESSON2_SCAN_MATCH_PLICP
#define LESSON2_SCAN_MATCH_PLICP

#include <cmath>
#include <vector>
#include <chrono>

// ros
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/TwistStamped.h>

// tf2
#include <tf2/utils.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/transform_broadcaster.h"

// csm
#include <csm/csm_all.h>
#undef min
#undef max

class ScanMatchPLICP
{
private:
    ros::NodeHandle node_handle_;           // ros中的句柄
    ros::NodeHandle private_node_;          // ros中的私有句柄
    ros::Subscriber laser_scan_subscriber_; // 声明一个Subscriber

    ros::Time last_icp_time_;

    bool initialized_;
    std::vector<double> a_cos_;
    std::vector<double> a_sin_;

    std::chrono::steady_clock::time_point start_time_;
    std::chrono::steady_clock::time_point end_time_;
    std::chrono::duration<double> time_used_;

    // csm
    sm_params input_;
    sm_result output_;
    LDP prev_ldp_scan_;
    LDP curr_ldp_scan_;

    // tf2_ros::Buffer tfBuffer_;
    // geometry_msgs::TransformStamped transformStamped_;
    // double kf_dist_linear_;
    // double kf_dist_linear_sq_;
    // double kf_dist_angular_;
    // bool is_inverted_; // 雷达是否倒着安装, false 为正着安装
    // geometry_msgs::Twist latest_velocity_;

    bool CheckInverted();
    void CreateCache(const sensor_msgs::LaserScan::ConstPtr &scan_msg);
    void LaserScanToLDP(const sensor_msgs::LaserScan::ConstPtr &scan_msg, LDP &ldp);
    void ScanMatchWithPLICP(LDP &curr_ldp_scan, const ros::Time &time);
    void GetPrediction(double &pr_ch_x, double &pr_ch_y, double &pr_ch_a, double dt);
    bool NewKeyframeNeeded(const tf2::Transform &d);

public:
    ScanMatchPLICP();
    ~ScanMatchPLICP();

    void ScanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg);
};

#endif // LESSON2_SCAN_MATCH_PLICP