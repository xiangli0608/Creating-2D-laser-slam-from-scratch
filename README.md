# Creating-2D-laser-slam-from-scratch

## 这个项目的目的

本人看了几套开源激光SLAM的代码, 依然感觉自己不是特别会, 对SLAM只能说懂个大概, 不敢说理解的特别深入. 同时, 加了一些社群, 发现很多初学激光SLAM的同学苦于没有激光SLAM方面的教程.

所以, 我想着自己来手动实践一下, 从零开始搭一套SLAM出来, 通过代码的角度一点一点地深入学习SLAM.

由于我现在的水平有限, 只能先对现有的开源激光SLAM框架入手, 先对这些框架进行解读与改进, 在这个过程中慢慢形成属于自己的一套激光SLAM的框架与架构. 


由于我现在是边学边写文章, 同时还要工作, 所以大概会以每三周左右发布一篇新文章. 文章将在 **公众号**, **CSDN**, **知乎** 三个途径进行同步更新.

 - **公众号名称**: 从零开始搭SLAM
 - **CSDN地址**: [https://blog.csdn.net/tiancailx?spm=1011.2124.3001.5113](https://blog.csdn.net/tiancailx?spm=1011.2124.3001.5113)
 - **知乎专栏地址**: [https://www.zhihu.com/column/c_1314297528322764800](https://www.zhihu.com/column/c_1314297528322764800)

## 这些代码怎么用
每篇博客中会详细说明本篇博客对应的代码的路径，与如何运行代码.

## 依赖库
代码是处于更新状态的，所有需要安装的依赖库都会写在 install_dependence.sh 脚本中，如果发现编译时由于依赖库报错，按照如下命令安装下依赖库即可.

```bash
cd /path_to_workspace/src/Creating-2D-laser-slam-from-scratch
chmod +x install_dependence.sh
./install_dependence.sh
```

## 运行环境
- ubuntu 16.04
- ros kinectic
- pcl 1.7
- ros-kinetic-libg2o
- ceres-slover 1.13.0
- gtsam 4.0.2

## 测试数据
目前, 所用的数据集是我自己录制的，我将所有的数据集汇总到一个在线表格中，地址如下
[https://docs.qq.com/sheet/DVElRQVNlY0tHU01I?tab=BB08J2](https://docs.qq.com/sheet/DVElRQVNlY0tHU01I?tab=BB08J2)

---

以下为每篇文章对应的节点如何运行，以及对应节点的功能简介

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

### 4.4 hector slam 的简单重写
该节点对hector的代码进行了整理，并发布了map->odom->base_link的TF树，并进行了注释

通过如下命令运行该节点
`roslaunch lesson4 hector_slam.launch`

hector中依赖了laser_geometry，如果编译不过请手动安装下这个包

## 5 lesson5
### 5.1 使用imu以及轮速计进行二维激光雷达数据的运动畸变校正
该节点使用imu以及轮速计进行二维激光雷达数据的运动畸变校正，将校正畸变后的数据以点云的形式发布出来

通过如下命令运行该节点
`roslaunch lesson5 lidar_undistortion.launch`

## 6 lesson6
### 6.1 基于Karto的前端实现
该节点使用了slam_karto的代码，进行了open_karto 的调用．并将回环检测与后端优化注释掉了，以体验前端．

通过如下命令运行该节点
`roslaunch lesson6 karto_slam.launch`

### 6.2 基于sparse-bundle-adjustment的后端优化与回环检测的实现
该节点使用了slam_karto的代码，进行了karto的后端优化模块的实现，并进行了参数化配置，并在室外数据集上验证回环检测与后端优化的效果

通过如下命令运行该节点
`roslaunch lesson6 karto_slam_outdoor.launch`

### 6.3 基于G2O的后端优化的实现
使用g2o代替sba来进行karto的后端优化的计算.

通过如下命令运行该节点
`roslaunch lesson6 karto_slam_outdoor.launch solver_type:=g2o_solver`

### 6.4 基于Ceres的后端优化的实现
使用ceres代替sba来进行karto的后端优化的计算.

通过如下命令运行该节点
`roslaunch lesson6 karto_slam_outdoor.launch solver_type:=ceres_solver`

### 6.5 基于GTSAM的后端优化的实现
使用gtsam代替sba来进行karto的后端优化的计算.

通过如下命令运行该节点
`roslaunch lesson6 karto_slam_outdoor.launch solver_type:=gtsam_solver`

## 完结撒花
从零开始搭二维激光SLAM系列的代码及文章就到此为止了.

二维激光SLAM领域想写的, 想实验的基本都写上了. 涵盖了二维激光SLAM的前端, 建图, 后端的大部分知识了, 当然也有很多没有提到的知识点.

下个系列应该是写从零开始搭视觉SLAM, 敬请期待.

历史文章请大家去个人公众号: **从零开始搭SLAM** 中查看.

十分感谢大家的关注!

完结于2021-11-22

李 想