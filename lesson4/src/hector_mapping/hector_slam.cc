
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

#include "lesson4/hector_mapping/hector_slam.h"

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include "sensor_msgs/PointCloud2.h"

#ifndef TF_SCALAR_H
typedef btScalar tfScalar;
#endif

HectorMappingRos::HectorMappingRos()
    : lastGetMapUpdateIndex(-100), tfB_(0), map__publish_thread_(0), initial_pose_set_(false)
{
    ros::NodeHandle private_nh_("~");

    std::string mapTopic_ = "map";

    private_nh_.param("pub_map_odom_transform", p_pub_map_odom_transform_, true);
    private_nh_.param("pub_odometry", p_pub_odometry_, false);
    private_nh_.param("scan_subscriber_queue_size", p_scan_subscriber_queue_size_, 5);

    private_nh_.param("map_resolution", p_map_resolution_, 0.05);
    private_nh_.param("map_size", p_map_size_, 1024);
    private_nh_.param("map_start_x", p_map_start_x_, 0.5);
    private_nh_.param("map_start_y", p_map_start_y_, 0.5);
    private_nh_.param("map_multi_res_levels", p_map_multi_res_levels_, 3);

    private_nh_.param("update_factor_free", p_update_factor_free_, 0.4);
    private_nh_.param("update_factor_occupied", p_update_factor_occupied_, 0.9);

    private_nh_.param("map_update_distance_thresh", p_map_update_distance_threshold_, 0.4);
    private_nh_.param("map_update_angle_thresh", p_map_update_angle_threshold_, 0.9);

    private_nh_.param("scan_topic", p_scan_topic_, std::string("laser_scan"));

    private_nh_.param("use_tf_scan_transformation", p_use_tf_scan_transformation_, true);
    private_nh_.param("use_tf_pose_start_estimate", p_use_tf_pose_start_estimate_, false);
    private_nh_.param("map_with_known_poses", p_map_with_known_poses_, false);

    private_nh_.param("base_frame", p_base_frame_, std::string("base_link"));
    private_nh_.param("map_frame", p_map_frame_, std::string("map"));
    private_nh_.param("odom_frame", p_odom_frame_, std::string("odom"));

    private_nh_.param("pub_map_scanmatch_transform", p_pub_map_scanmatch_transform_, true);
    private_nh_.param("tf_map_scanmatch_transform_frame_name", p_tf_map_scanmatch_transform_frame_name_, std::string("scanmatcher_frame"));

    private_nh_.param("output_timing", p_timing_output_, false);

    private_nh_.param("map_pub_period", p_map_pub_period_, 2.0);

    double tmp = 0.0;
    private_nh_.param("laser_min_dist", tmp, 0.4);
    p_sqr_laser_min_dist_ = static_cast<float>(tmp * tmp);

    private_nh_.param("laser_max_dist", tmp, 30.0);
    p_sqr_laser_max_dist_ = static_cast<float>(tmp * tmp);

    private_nh_.param("laser_z_min_value", tmp, -1.0);
    p_laser_z_min_value_ = static_cast<float>(tmp);

    private_nh_.param("laser_z_max_value", tmp, 1.0);
    p_laser_z_max_value_ = static_cast<float>(tmp);

    if (p_pub_odometry_)
    {
        odometryPublisher_ = node_.advertise<nav_msgs::Odometry>("scanmatch_odom", 50);
    }

    slamProcessor = new hectorslam::HectorSlamProcessor(static_cast<float>(p_map_resolution_), p_map_size_, p_map_size_, Eigen::Vector2f(p_map_start_x_, p_map_start_y_),
                                                        p_map_multi_res_levels_);
    slamProcessor->setUpdateFactorFree(p_update_factor_free_);                // 0.4
    slamProcessor->setUpdateFactorOccupied(p_update_factor_occupied_);        // 0.9
    slamProcessor->setMapUpdateMinDistDiff(p_map_update_distance_threshold_); // 0.4
    slamProcessor->setMapUpdateMinAngleDiff(p_map_update_angle_threshold_);   // 0.9

    int mapLevels = slamProcessor->getMapLevels();
    mapLevels = 1; /// 这里设置成只发布最高精度的地图，如果有其他需求，如进行路径规划等等需要多层地图时，注释本行。

    for (int i = 0; i < mapLevels; ++i)
    {
        mapPubContainer.push_back(MapPublisherContainer());
        slamProcessor->addMapMutex(i, new HectorMapMutex());

        std::string mapTopicStr(mapTopic_);

        if (i != 0)
        {
            mapTopicStr.append("_" + boost::lexical_cast<std::string>(i));
        }

        std::string mapMetaTopicStr(mapTopicStr);
        mapMetaTopicStr.append("_metadata");

        MapPublisherContainer &tmp = mapPubContainer[i];
        tmp.mapPublisher_ = node_.advertise<nav_msgs::OccupancyGrid>(mapTopicStr, 1, true);
        tmp.mapMetadataPublisher_ = node_.advertise<nav_msgs::MapMetaData>(mapMetaTopicStr, 1, true);

        // setServiceGetMapData(tmp.map_, slamProcessor->getGridMap(i)); // 设置地图服务

        if (i == 0)
        {
            mapPubContainer[i].mapMetadataPublisher_.publish(mapPubContainer[i].map_.map.info);
        }
    }

    ROS_INFO("HectorSM p_base_frame_: %s", p_base_frame_.c_str());
    ROS_INFO("HectorSM p_map_frame_: %s", p_map_frame_.c_str());
    ROS_INFO("HectorSM p_odom_frame_: %s", p_odom_frame_.c_str());
    ROS_INFO("HectorSM p_scan_topic_: %s", p_scan_topic_.c_str());
    ROS_INFO("HectorSM p_use_tf_scan_transformation_: %s", p_use_tf_scan_transformation_ ? ("true") : ("false"));
    ROS_INFO("HectorSM p_pub_map_odom_transform_: %s", p_pub_map_odom_transform_ ? ("true") : ("false"));
    ROS_INFO("HectorSM p_scan_subscriber_queue_size_: %d", p_scan_subscriber_queue_size_);
    ROS_INFO("HectorSM p_map_pub_period_: %f", p_map_pub_period_);
    ROS_INFO("HectorSM p_update_factor_free_: %f", p_update_factor_free_);
    ROS_INFO("HectorSM p_update_factor_occupied_: %f", p_update_factor_occupied_);
    ROS_INFO("HectorSM p_map_update_distance_threshold_: %f ", p_map_update_distance_threshold_);
    ROS_INFO("HectorSM p_map_update_angle_threshold_: %f", p_map_update_angle_threshold_);
    ROS_INFO("HectorSM p_laser_z_min_value_: %f", p_laser_z_min_value_);
    ROS_INFO("HectorSM p_laser_z_max_value_: %f", p_laser_z_max_value_);

    /** 设置subscriber 和 publisher ,其中scanCallback是主要处理函数 **/
    scanSubscriber_ = node_.subscribe(p_scan_topic_, p_scan_subscriber_queue_size_, &HectorMappingRos::scanCallback, this); // 雷达数据处理

    // poseUpdatePublisher_ = node_.advertise<geometry_msgs::PoseWithCovarianceStamped>(p_pose_update_topic_, 1, false);
    // posePublisher_ = node_.advertise<geometry_msgs::PoseStamped>("slam_out_pose", 1, false);
    // scan_point_cloud_publisher_ = node_.advertise<sensor_msgs::PointCloud>("slam_cloud", 1, false);

    tfB_ = new tf::TransformBroadcaster();
    // ROS_ASSERT(tfB_);

    map__publish_thread_ = new boost::thread(boost::bind(&HectorMappingRos::publishMapLoop, this, p_map_pub_period_));

    map_to_odom_.setIdentity();

    lastMapPublishTime = ros::Time(0, 0);
}

HectorMappingRos::~HectorMappingRos()
{
    delete slamProcessor;

    if (tfB_)
        delete tfB_;

    if (map__publish_thread_)
        delete map__publish_thread_;
}

/**
 * 激光数据处理回调函数，将ros数据格式转换为算法中的格式，并转换成地图尺度，交由slamProcessor处理。
 * 算法中所有的计算都是在地图尺度下进行。  ROS部分的内容根据实际需求添加删减
 * @param scan  激光的数据ROS消息包
 */
void HectorMappingRos::scanCallback(const sensor_msgs::LaserScan &scan)
{
    ros::WallTime startTime = ros::WallTime::now();

    // 不使用tf，默认雷达的坐标系就是base_link
    if (!p_use_tf_scan_transformation_) // 未使用tf
    {

        if (rosLaserScanToDataContainer(scan, laserScanContainer, slamProcessor->getScaleToMap()))
        {
            slamProcessor->update(laserScanContainer, slamProcessor->getLastScanMatchPose());
        }
    }
    else
    {
        ros::Duration dur(0.5);

        if (tf_.waitForTransform(p_base_frame_, scan.header.frame_id, scan.header.stamp, dur))
        {
            tf::StampedTransform laserTransform;
            tf_.lookupTransform(p_base_frame_, scan.header.frame_id, scan.header.stamp, laserTransform);

            //projector_.transformLaserScanToPointCloud(p_base_frame_ ,scan, pointCloud,tf_);
            /** 将laser scan 转换成 laser frame下的点云消息 **/
            // projector_.projectLaser(scan, laser_point_cloud_,30.0);

            if (scan_point_cloud_publisher_.getNumSubscribers() > 0)
            {
                scan_point_cloud_publisher_.publish(laser_point_cloud_);
            }

            Eigen::Vector3f startEstimate(Eigen::Vector3f::Zero());

            if (rosPointCloudToDataContainer(laser_point_cloud_, laserTransform, laserScanContainer, slamProcessor->getScaleToMap()))
            {
                if (initial_pose_set_)
                {
                    initial_pose_set_ = false;
                    startEstimate = initial_pose_;
                }
                else if (p_use_tf_pose_start_estimate_)
                {

                    try
                    {
                        tf::StampedTransform stamped_pose;

                        tf_.waitForTransform(p_map_frame_, p_base_frame_, scan.header.stamp, ros::Duration(0.5));
                        tf_.lookupTransform(p_map_frame_, p_base_frame_, scan.header.stamp, stamped_pose);

                        tfScalar yaw, pitch, roll;
                        stamped_pose.getBasis().getEulerYPR(yaw, pitch, roll);

                        startEstimate = Eigen::Vector3f(stamped_pose.getOrigin().getX(), stamped_pose.getOrigin().getY(), yaw);
                    }
                    catch (tf::TransformException e)
                    {
                        ROS_ERROR("Transform from %s to %s failed\n", p_map_frame_.c_str(), p_base_frame_.c_str());
                        startEstimate = slamProcessor->getLastScanMatchPose();
                    }
                }
                else
                {
                    startEstimate = slamProcessor->getLastScanMatchPose();
                }

                if (p_map_with_known_poses_)
                {
                    slamProcessor->update(laserScanContainer, startEstimate, true);
                }
                else
                {
                    // 进入扫描匹配与地图更新
                    slamProcessor->update(laserScanContainer, startEstimate);
                }
            }
        }
        else
        {
            ROS_INFO("lookupTransform %s to %s timed out. Could not transform laser scan into base_frame.", p_base_frame_.c_str(), scan.header.frame_id.c_str());
            return;
        }
    }

    if (p_timing_output_)
    {
        ros::WallDuration duration = ros::WallTime::now() - startTime;
        ROS_INFO("HectorSLAM Iter took: %f milliseconds", duration.toSec() * 1000.0f);
    }

    //If we're just building a map with known poses, we're finished now. Code below this point publishes the localization results.
    /// 以下内容ROS相关，发布定位结果消息等等
    if (p_map_with_known_poses_)
    {
        return;
    }

    poseInfoContainer_.update(slamProcessor->getLastScanMatchPose(), slamProcessor->getLastScanMatchCovariance(), scan.header.stamp, p_map_frame_);

    poseUpdatePublisher_.publish(poseInfoContainer_.getPoseWithCovarianceStamped());
    posePublisher_.publish(poseInfoContainer_.getPoseStamped());

    if (p_pub_odometry_)
    {
        nav_msgs::Odometry tmp;
        tmp.pose = poseInfoContainer_.getPoseWithCovarianceStamped().pose;

        tmp.header = poseInfoContainer_.getPoseWithCovarianceStamped().header;
        tmp.child_frame_id = p_base_frame_;
        odometryPublisher_.publish(tmp);
    }

    if (p_pub_map_odom_transform_)
    {
        tf::StampedTransform odom_to_base;

        try
        {
            tf_.waitForTransform(p_odom_frame_, p_base_frame_, scan.header.stamp, ros::Duration(0.5));
            tf_.lookupTransform(p_odom_frame_, p_base_frame_, scan.header.stamp, odom_to_base);
        }
        catch (tf::TransformException e)
        {
            ROS_ERROR("Transform failed during publishing of map_odom transform: %s", e.what());
            odom_to_base.setIdentity();
        }
        map_to_odom_ = tf::Transform(poseInfoContainer_.getTfTransform() * odom_to_base.inverse());
        tfB_->sendTransform(tf::StampedTransform(map_to_odom_, scan.header.stamp, p_map_frame_, p_odom_frame_));
    }

    if (p_pub_map_scanmatch_transform_)
    {
        tfB_->sendTransform(tf::StampedTransform(poseInfoContainer_.getTfTransform(), scan.header.stamp, p_map_frame_, p_tf_map_scanmatch_transform_frame_name_));
    }
}

void HectorMappingRos::publishMapLoop(double map_pub_period)
{
    ros::Rate r(1.0 / map_pub_period);
    while (ros::ok())
    {
        //ros::WallTime t1 = ros::WallTime::now();
        ros::Time mapTime(ros::Time::now());
        //publishMap(mapPubContainer[2],slamProcessor->getGridMap(2), mapTime);
        //publishMap(mapPubContainer[1],slamProcessor->getGridMap(1), mapTime);
        publishMap(mapPubContainer[0], slamProcessor->getGridMap(0), mapTime, slamProcessor->getMapMutex(0));

        //ros::WallDuration t2 = ros::WallTime::now() - t1;

        //std::cout << "time s: " << t2.toSec();
        //ROS_INFO("HectorSM ms: %4.2f", t2.toSec()*1000.0f);

        r.sleep();
    }
}

void HectorMappingRos::publishMap(MapPublisherContainer &mapPublisher,
                                  const hectorslam::GridMap &gridMap, ros::Time timestamp, MapLockerInterface *mapMutex)
{
    nav_msgs::GetMap::Response &map_(mapPublisher.map_);

    //only update map if it changed
    if (lastGetMapUpdateIndex != gridMap.getUpdateIndex())
    {

        int sizeX = gridMap.getSizeX();
        int sizeY = gridMap.getSizeY();

        int size = sizeX * sizeY;

        std::vector<int8_t> &data = map_.map.data;

        //std::vector contents are guaranteed to be contiguous, use memset to set all to unknown to save time in loop
        memset(&data[0], -1, sizeof(int8_t) * size);

        if (mapMutex)
        {
            mapMutex->lockMap();
        }

        for (int i = 0; i < size; ++i)
        {
            if (gridMap.isFree(i))
            {
                data[i] = 0;
            }
            else if (gridMap.isOccupied(i))
            {
                data[i] = 100;
            }
        }

        lastGetMapUpdateIndex = gridMap.getUpdateIndex();

        if (mapMutex)
        {
            mapMutex->unlockMap();
        }
    }

    map_.map.header.stamp = timestamp;

    mapPublisher.mapPublisher_.publish(map_.map);
}

/**
 *  数据转换，转换到算法使用的数据格式，并将尺度转换到地图尺度。
 * @param scan  ros数据格式输入
 * @param dataContainer  算法使用数据格式输出
 * @param scaleToMap    地图尺度系数
 * @return
 */
bool HectorMappingRos::rosLaserScanToDataContainer(const sensor_msgs::LaserScan &scan, hectorslam::DataContainer &dataContainer, float scaleToMap)
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
            dist *= scaleToMap; ///! 将实际物理尺度转换到地图尺度）
            dataContainer.add(Eigen::Vector2f(cos(angle) * dist, sin(angle) * dist));
        }

        angle += scan.angle_increment;
    }

    return true;
}

bool HectorMappingRos::rosPointCloudToDataContainer(const sensor_msgs::PointCloud &pointCloud, const tf::StampedTransform &laserTransform, hectorslam::DataContainer &dataContainer, float scaleToMap)
{
    size_t size = pointCloud.points.size();
    //ROS_INFO("size: %d", size);

    dataContainer.clear();

    tf::Vector3 laserPos(laserTransform.getOrigin());
    dataContainer.setOrigo(Eigen::Vector2f(laserPos.x(), laserPos.y()) * scaleToMap);

    for (size_t i = 0; i < size; ++i)
    {

        const geometry_msgs::Point32 &currPoint(pointCloud.points[i]);

        float dist_sqr = currPoint.x * currPoint.x + currPoint.y * currPoint.y;

        if ((dist_sqr > p_sqr_laser_min_dist_) && (dist_sqr < p_sqr_laser_max_dist_))
        {

            if ((currPoint.x < 0.0f) && (dist_sqr < 0.50f))
            {
                continue;
            }

            tf::Vector3 pointPosBaseFrame(laserTransform * tf::Vector3(currPoint.x, currPoint.y, currPoint.z));

            float pointPosLaserFrameZ = pointPosBaseFrame.z() - laserPos.z();

            if (pointPosLaserFrameZ > p_laser_z_min_value_ && pointPosLaserFrameZ < p_laser_z_max_value_)
            {
                dataContainer.add(Eigen::Vector2f(pointPosBaseFrame.x(), pointPosBaseFrame.y()) * scaleToMap);
            }
        }
    }

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lesson4_hector_slam");

    HectorMappingRos hector_slam;

    ros::spin();

    return (0);
}