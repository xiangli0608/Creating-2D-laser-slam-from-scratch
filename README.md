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
cd ~/catkin_ws/src
catkin_create_pkg lesson2 pcl_conversions pcl_ros roscpp sensor_msgs 
```

通过如下命令运行该节点
`roslaunch lesson2 scan_to_pointcloud2_converter.launch`

### 2.2 使用PCL的ICP算法计算雷达的帧间坐标变换
该节点展示了如何使用PCL的ICP算法进行雷达的帧间坐标变换, 感受ICP算法的不足

通过如下命令运行该节点
`roslaunch lesson2 scan_match_icp.launch`

### 2.3 使用PL-ICP算法计算雷达的帧间坐标变换