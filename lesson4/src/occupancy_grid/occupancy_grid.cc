
#include <iostream>
#include <chrono>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

class OccupancyGrid
{
private:
    ros::NodeHandle node_handle_;           // ros中的句柄
    ros::Publisher map_publisher_;          // 声明一个Publisher
    ros::Publisher map_publisher_metadata_; // 声明一个Publisher
    nav_msgs::OccupancyGrid map_;           //用来发布map的实体对象

    std::chrono::steady_clock::time_point start_time_;
    std::chrono::steady_clock::time_point end_time_;
    std::chrono::duration<double> time_used_;

public:
    OccupancyGrid();
    void PublishMap();
};

// 构造函数
OccupancyGrid::OccupancyGrid()
{
    // \033[1;32m，\033[0m 终端显示成绿色
    ROS_INFO_STREAM("\033[1;32m----> Make Occupancy Grid Map by no move started.\033[0m");

    map_publisher_ = node_handle_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
    map_publisher_metadata_ = node_handle_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);

    // 对map_进行初始化
    map_.header.frame_id = "map";

    // 地图的分辨率为0.05m,代表一个格子的距离是0.05m
    map_.info.resolution = 0.05;

    // 地图图片像素的大小, width为地图的宽度是多少个像素
    map_.info.width = 30;
    map_.info.height = 30;

    // 如果要表示地图图片为多少米的话,就需要用实际长度除以分辨率,得到像素值
    // map_.info.width = 100 / map_.info.resolution;
    // map_.info.height = 100 / map_.info.resolution;

    // 地图左下角的点对应的物理坐标
    map_.info.origin.position.x = 0.0;
    map_.info.origin.position.y = 0.0;

    // 对数组进行初始化, 数组的大小为实际像素的个数
    map_.data.resize(map_.info.width * map_.info.height);
}

// 构造map并进行发布
void OccupancyGrid::PublishMap()
{
    start_time_ = std::chrono::steady_clock::now();

    // 通过二维索引算出来的一维索引
    int index = 0;

    // 10种情况
    int count = 10;

    // 固定列, 优先对行进行遍历
    for (int j = 0; j < map_.info.height; j++)
    {
        for (int i = 0; i < map_.info.width; i++)
        {
            // 二维坐标转成一维坐标
            index = i + j * map_.info.width;
            // std::cout << " index: " << index ;

            // 0代表空闲, 100代表占用, -1代表未知, 默认值为0

            // 为map赋予不同的值来体验效果, 从-1 到 254
            if (index % count == 0)
                map_.data[index] = -1;
            else if (index % count == 1)
                map_.data[index] = 0;
            else if (index % count == 2)
                map_.data[index] = 30;
            else if (index % count == 3)
                map_.data[index] = 60;
            else if (index % count == 4)
                map_.data[index] = 100;
            else if (index % count == 5)
                map_.data[index] = 140;
            else if (index % count == 6)
                map_.data[index] = 180;
            else if (index % count == 7)
                map_.data[index] = 220;
            else if (index % count == 8)
                map_.data[index] = 240;
            else if (index % count == 9)
                map_.data[index] = 254;
        }
    }
    
    // 设置这一帧地图数据的时间戳
    map_.header.stamp = ros::Time::now();

    // 发布map和map_metadata话题
    map_publisher_.publish(map_);
    map_publisher_metadata_.publish(map_.info);

    end_time_ = std::chrono::steady_clock::now();
    time_used_ = std::chrono::duration_cast<std::chrono::duration<double>>(end_time_ - start_time_);
    std::cout << "发布一次地图用时: " << time_used_.count() << " 秒。\n" << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lesson4_make_occupancy_grid_map");
    OccupancyGrid occupancy_grid;
    ros::Rate rate(1);

    while (ros::ok())
    {
        ROS_INFO("publish occupancy map");
        occupancy_grid.PublishMap();
        rate.sleep();
    }

    return (0);
}
