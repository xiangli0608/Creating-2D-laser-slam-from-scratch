
#include "lesson2/scan_match_icp.h"
#include <chrono>

ScanMatchICP::ScanMatchICP(/* args */)
{
    // \033[1;32m，\033[0m 终端显示成绿色
    ROS_INFO_STREAM("\033[1;32m----> Scan to PointCloud2 Converter.\033[0m");

    laser_scan_subscriber_ = node_handle_.subscribe(
        "laser_scan", 1, &ScanMatchICP::ScanCallback, this);

    is_first_scan_ = true;

    current_pointcloud_ = boost::shared_ptr<PointCloudT>(new PointCloudT());
    last_pointcloud_ = boost::shared_ptr<PointCloudT>(new PointCloudT());
}

ScanMatchICP::~ScanMatchICP()
{
    // delete current_pointcloud_;
    // delete last_pointcloud_;

    ROS_INFO("Destroying ScanMatchICP");
}

void ScanMatchICP::ScanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
    ScanMatchWithICP(scan_msg);
}

void ScanMatchICP::ScanMatchWithICP(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
    std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
    
    if (is_first_scan_ == true)
    {
        ConvertScan2PointCloud(scan_msg);
        is_first_scan_ = false;
        return;
    }
    else
        *last_pointcloud_ = *current_pointcloud_;

    ConvertScan2PointCloud(scan_msg);

    // ICP Settings
    icp_.setInputSource(last_pointcloud_);
    icp_.setInputTarget(current_pointcloud_);

    // 
    pcl::PointCloud<pcl::PointXYZ> unused_result;
    icp_.align(unused_result);

    // std::cout << "has converged:" << icp_.hasConverged() << " score: " << icp_.getFitnessScore() << std::endl;

    if (icp_.hasConverged() == false)
    {
        std::cout << "not Converged" << std::endl;
        return;
    }
    else
    {
        Eigen::Affine3f transfrom;
        transfrom = icp_.getFinalTransformation();

        // Get pose transformation
        float x, y, z, roll, pitch, yaw;
        pcl::getTranslationAndEulerAngles(transfrom, x, y, z, roll, pitch, yaw);
        std::cout << "tran: (" << x << ", " << y << ", " << yaw * 180 / M_PI << ")" << std::endl;
    }

    std::chrono::steady_clock::time_point end_time = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(end_time - start_time);
    std::cout<<"处理一次数据用时: "<< time_used.count() << " 秒。" << std::endl;

}

void ScanMatchICP::ConvertScan2PointCloud(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
    PointCloudT::Ptr cloud_msg = boost::shared_ptr<PointCloudT>(new PointCloudT());
    // 对容器进行初始化
    cloud_msg->points.resize(scan_msg->ranges.size());

    for (unsigned int i = 0; i < scan_msg->ranges.size(); ++i)
    {
        // 首先声明一个 cloud_msg第i个点的 引用
        pcl::PointXYZ &point_tmp = cloud_msg->points[i];
        // 获取scan的第i个点的距离值
        float range = scan_msg->ranges[i];

        // 将 inf 与 nan 点 设置为无效点
        if (!std::isfinite(range))
        {
            // std::cout << " " << i << " " << scan_msg->ranges[i];
            // point_tmp = invalid_point_;
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
        // else
        // 无效点
        // point_tmp = invalid_point_;
    }

    cloud_msg->width = scan_msg->ranges.size();
    cloud_msg->height = 1;
    cloud_msg->is_dense = false; // contains nans

    // 将scan_msg的消息头 赋值到 pcl::PointCloud<pcl::PointXYZ>的消息头
    pcl_conversions::toPCL(scan_msg->header, cloud_msg->header);
    *current_pointcloud_ = *cloud_msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lesson2_scan_to_cloud_converter_node"); // 节点的名字
    ScanMatchICP scan_match_icp;

    ros::spin(); // 程序执行到此处时开始进行等待，每次订阅的消息到来都会执行一次ScanCallback()
    return 0;
}