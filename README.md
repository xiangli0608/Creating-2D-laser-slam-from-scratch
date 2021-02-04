# Creating-2D-laser-slam-from-scratch

## 1 lesson1: 

### 1.1 如何遍历雷达数据
该节点展示了Laser_scan的数据类型，以及如何对其进行遍历

通过如下命令运行该节点
`roslaunch lesson1 demo.launch`

### 1.2 对雷达数据进行简单的特征提取
该节点展示了如何对Laser_scan进行简单的特征点提取, 特征点提取的算法取自于LIO-SAM

通过如下命令运行该节点
`roslaunch lesson1 feature_detection.launch`

## 2 lesson2: 

### 2.1 使用PCL进行雷达数据的格式转换
该节点展示了如果对将 sensor_msgs/LaserScan 的数据类型 转换成 sensor_msgs/PointCloud2。
实际上是将 sensor_msgs/LaserScan 转成了 pcl::PointCloud< PointT>, 再由ros将　pcl::PointCloud< PointT> 转换成 sensor_msgs/PointCloud2。

通过如下命令生成包

```
cd ~/catkin_ws/src/Creating-2D-laser-slam-from-scratch
catkin_create_pkg lesson2 pcl_conversions pcl_ros roscpp sensor_msgs 
```

通过如下命令运行该节点
`roslaunch lesson2 scan_to_pointcloud2_converter.launch`

### 2.2 使用PCL的ICP算法计算雷达的帧间坐标变换
该节点展示了如何使用PCL的ICP算法进行雷达的帧间坐标变换, 感受ICP算法的不足

通过如下命令运行该节点
`roslaunch lesson2 scan_match_icp.launch`

## 3 lesson3

### 3.1 使用PL-ICP算法计算雷达的帧间坐标变换
该节点展示了如何使用PLICP算法进行雷达的帧间坐标变换

通过如下命令生成包
`
cd ~/catkin_ws/src/Creating-2D-laser-slam-from-scratch
catkin_create_pkg lesson3 roscpp sensor_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs nav_msgs
`
编译前需要安装依赖，命令为
`sudo apt-get install ros-kinetic-csm`

通过如下命令运行该节点
`roslaunch lesson3 scan_match_plicp.launch`

### 3.2 基于PL-ICP的激光里程计
该节点使用 基于PLICP算法计算出的帧间坐标变换，累加成一个激光雷达里程计，并发布tf.
本激光里程计在长走廊环境下**匹配失败**．

通过如下命令运行该节点
`roslaunch lesson3 plicp_odometry.launch`

## 4 lesson4

### 4.1 简单的栅格地图的构建
该节点展示了如何发布栅格地图，以及向栅格地图中存储不同值时的效果

通过如下命令运行该节点
`roslaunch lesson4 make_occupancy_grid_map.launch`

### 4.2 基于GMapping的栅格地图的构建
该节点展示了如何使用GMapping中的建图算法，将激光雷达数据转换成栅格地图

通过如下命令运行该节点
`roslaunch lesson4 make_gmapping_map.launch`

### 4.3 基于Hector的栅格地图的构建
该节点展示了如何使用Hector中的建图算法，将激光雷达数据转换成栅格地图

通过如下命令运行该节点
`roslaunch lesson4 make_hector_map.launch`

### 4.4 hector slam的简单重写
该节点对hector的代码进行了整理，并发布了map->odom->base_link的TF树，并进行了注释

通过如下命令运行该节点
`roslaunch lesson4 hector_slam.launch`

hector中依赖了laser_geometry，如果编译不过请手动安装下这个包