/*
 * slam_karto
 * Copyright (c) 2008, Willow Garage, Inc.
 *
 * THE WORK (AS DEFINED BELOW) IS PROVIDED UNDER THE TERMS OF THIS CREATIVE
 * COMMONS PUBLIC LICENSE ("CCPL" OR "LICENSE"). THE WORK IS PROTECTED BY
 * COPYRIGHT AND/OR OTHER APPLICABLE LAW. ANY USE OF THE WORK OTHER THAN AS
 * AUTHORIZED UNDER THIS LICENSE OR COPYRIGHT LAW IS PROHIBITED.
 *
 * BY EXERCISING ANY RIGHTS TO THE WORK PROVIDED HERE, YOU ACCEPT AND AGREE TO
 * BE BOUND BY THE TERMS OF THIS LICENSE. THE LICENSOR GRANTS YOU THE RIGHTS
 * CONTAINED HERE IN CONSIDERATION OF YOUR ACCEPTANCE OF SUCH TERMS AND
 * CONDITIONS.
 *
 */

/* Author: Brian Gerkey */

#include "lesson6/karto_slam.h"

SlamKarto::SlamKarto() : got_map_(false),
                         laser_count_(0),
                         transform_thread_(NULL),
                         marker_count_(0)
{
  ROS_INFO_STREAM("\033[1;32m----> Karto SLAM started.\033[0m");

  // Initialize Karto structures
  mapper_ = new karto::Mapper();
  dataset_ = new karto::Dataset();

  // 参数初始化
  InitParams();

  map_to_odom_.setIdentity();

  // Set up advertisements and subscriptions
  tfB_ = new tf::TransformBroadcaster();
  sst_ = node_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
  sstm_ = node_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
  scan_filter_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(node_, "scan", 5);
  scan_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(*scan_filter_sub_, tf_, odom_frame_, 5);
  scan_filter_->registerCallback(boost::bind(&SlamKarto::laserCallback, this, _1));
  marker_publisher_ = node_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);

  // Create a thread to periodically publish the latest map->odom
  // transform; it needs to go out regularly, uninterrupted by potentially
  // long periods of computation in our main loop.
  transform_thread_ = new boost::thread(boost::bind(&SlamKarto::publishLoop, this, transform_publish_period_));

  // Set solver to be used in loop closure
  if (use_back_end_)
  {
    ROS_INFO("Use back end.");
    solver_ = CreateSolver(solver_type_);
    mapper_->SetScanSolver(solver_);
  }
}

SlamKarto::~SlamKarto()
{
  if (transform_thread_)
  {
    transform_thread_->join();
    delete transform_thread_;
  }
  if (scan_filter_)
    delete scan_filter_;
  if (scan_filter_sub_)
    delete scan_filter_sub_;

  if (mapper_)
    delete mapper_;
  if (dataset_)
    delete dataset_;
  if (solver_)
    delete solver_;
}

// ros的参数初始化
void SlamKarto::InitParams()
{
  // Retrieve parameters
  ros::NodeHandle private_nh_("~");
  if (!private_nh_.getParam("odom_frame", odom_frame_))
    odom_frame_ = "odom";
  if (!private_nh_.getParam("map_frame", map_frame_))
    map_frame_ = "map";
  if (!private_nh_.getParam("base_frame", base_frame_))
    base_frame_ = "base_link";
  if (!private_nh_.getParam("throttle_scans", throttle_scans_))
    throttle_scans_ = 1;
  double tmp;
  if (!private_nh_.getParam("map_update_interval", tmp))
    tmp = 5.0;
  map_update_interval_.fromSec(tmp);

  private_nh_.param("use_scan_range", use_scan_range_, 12.0);

  double tmp_tol;
  private_nh_.param("transform_tolerance", tmp_tol, 0.0);
  transform_tolerance_.fromSec(tmp_tol);

  if (!private_nh_.getParam("resolution", resolution_))
  {
    resolution_ = 0.05;
  }

  private_nh_.param("transform_publish_period", transform_publish_period_, 0.05);

  // Setting General Parameters from the Parameter Server
  bool use_scan_matching;
  if (private_nh_.getParam("use_scan_matching", use_scan_matching))
    mapper_->setParamUseScanMatching(use_scan_matching);

  // 使用每个scan的质心来查看两个scan的距离
  bool use_scan_barycenter;
  if (private_nh_.getParam("use_scan_barycenter", use_scan_barycenter))
    mapper_->setParamUseScanBarycenter(use_scan_barycenter);

  // 设置最小距离，里程计移动长度超过此距离，则建立一个节点
  double minimum_travel_distance;
  if (private_nh_.getParam("minimum_travel_distance", minimum_travel_distance))
    mapper_->setParamMinimumTravelDistance(minimum_travel_distance);

  // 设置最小偏转角，如果里程计转向超过此值，则建立一个节点
  double minimum_travel_heading;
  if (private_nh_.getParam("minimum_travel_heading", minimum_travel_heading))
    mapper_->setParamMinimumTravelHeading(minimum_travel_heading);

  // 设置scanbuffer的长度
  int scan_buffer_size;
  if (private_nh_.getParam("scan_buffer_size", scan_buffer_size))
    mapper_->setParamScanBufferSize(scan_buffer_size);

  double scan_buffer_maximum_scan_distance;
  if (private_nh_.getParam("scan_buffer_maximum_scan_distance", scan_buffer_maximum_scan_distance))
    mapper_->setParamScanBufferMaximumScanDistance(scan_buffer_maximum_scan_distance);

  // 设置回环匹配最小响应阈值，大于此值才开始进行高精度匹配
  double link_match_minimum_response_fine;
  if (private_nh_.getParam("link_match_minimum_response_fine", link_match_minimum_response_fine))
    mapper_->setParamLinkMatchMinimumResponseFine(link_match_minimum_response_fine);

  // 设置两个连接的scans最大距离，大于此值则不考虑两者的响应阈值
  double link_scan_maximum_distance;
  if (private_nh_.getParam("link_scan_maximum_distance", link_scan_maximum_distance))
    mapper_->setParamLinkScanMaximumDistance(link_scan_maximum_distance);

  // 搜寻回环匹配的最大距离
  double loop_search_maximum_distance;
  if (private_nh_.getParam("loop_search_maximum_distance", loop_search_maximum_distance))
    mapper_->setParamLoopSearchMaximumDistance(loop_search_maximum_distance);

  // 做回环匹配
  bool do_loop_closing;
  if (private_nh_.getParam("do_loop_closing", do_loop_closing))
    mapper_->setParamDoLoopClosing(do_loop_closing);

  // 找到回环匹配的node，必须位于大于此值的scanbuffer上
  int loop_match_minimum_chain_size;
  if (private_nh_.getParam("loop_match_minimum_chain_size", loop_match_minimum_chain_size))
    mapper_->setParamLoopMatchMinimumChainSize(loop_match_minimum_chain_size);

  double loop_match_maximum_variance_coarse;
  if (private_nh_.getParam("loop_match_maximum_variance_coarse", loop_match_maximum_variance_coarse))
    mapper_->setParamLoopMatchMaximumVarianceCoarse(loop_match_maximum_variance_coarse);

  double loop_match_minimum_response_coarse;
  if (private_nh_.getParam("loop_match_minimum_response_coarse", loop_match_minimum_response_coarse))
    mapper_->setParamLoopMatchMinimumResponseCoarse(loop_match_minimum_response_coarse);

  double loop_match_minimum_response_fine;
  if (private_nh_.getParam("loop_match_minimum_response_fine", loop_match_minimum_response_fine))
    mapper_->setParamLoopMatchMinimumResponseFine(loop_match_minimum_response_fine);

  // Setting Correlation Parameters from the Parameter Server

  // 纠正位姿时使用的匹配器的大小
  double correlation_search_space_dimension;
  if (private_nh_.getParam("correlation_search_space_dimension", correlation_search_space_dimension))
    mapper_->setParamCorrelationSearchSpaceDimension(correlation_search_space_dimension);

  // 纠正位姿时使用的解析度
  double correlation_search_space_resolution;
  if (private_nh_.getParam("correlation_search_space_resolution", correlation_search_space_resolution))
    mapper_->setParamCorrelationSearchSpaceResolution(correlation_search_space_resolution);

  // 纠正位姿时将会被此值平滑
  double correlation_search_space_smear_deviation;
  if (private_nh_.getParam("correlation_search_space_smear_deviation", correlation_search_space_smear_deviation))
    mapper_->setParamCorrelationSearchSpaceSmearDeviation(correlation_search_space_smear_deviation);

  // Setting Correlation Parameters, Loop Closure Parameters from the Parameter Server

  // 回环检测时匹配器的大小
  double loop_search_space_dimension;
  if (private_nh_.getParam("loop_search_space_dimension", loop_search_space_dimension))
    mapper_->setParamLoopSearchSpaceDimension(loop_search_space_dimension);

  // 回环检测时匹配器的分辨率
  double loop_search_space_resolution;
  if (private_nh_.getParam("loop_search_space_resolution", loop_search_space_resolution))
    mapper_->setParamLoopSearchSpaceResolution(loop_search_space_resolution);

  // 回环检测时的平滑系数
  double loop_search_space_smear_deviation;
  if (private_nh_.getParam("loop_search_space_smear_deviation", loop_search_space_smear_deviation))
    mapper_->setParamLoopSearchSpaceSmearDeviation(loop_search_space_smear_deviation);

  // Setting Scan Matcher Parameters from the Parameter Server
  // scan-matching 时对里程计的补偿系数
  double distance_variance_penalty;
  if (private_nh_.getParam("distance_variance_penalty", distance_variance_penalty))
    mapper_->setParamDistanceVariancePenalty(distance_variance_penalty);
  // scan-matching时对角度的补偿系数
  double angle_variance_penalty;
  if (private_nh_.getParam("angle_variance_penalty", angle_variance_penalty))
    mapper_->setParamAngleVariancePenalty(angle_variance_penalty);
  // 精细匹配时搜索的角度范围
  double fine_search_angle_offset;
  if (private_nh_.getParam("fine_search_angle_offset", fine_search_angle_offset))
    mapper_->setParamFineSearchAngleOffset(fine_search_angle_offset);

  double coarse_search_angle_offset;
  if (private_nh_.getParam("coarse_search_angle_offset", coarse_search_angle_offset))
    mapper_->setParamCoarseSearchAngleOffset(coarse_search_angle_offset);

  double coarse_angle_resolution;
  if (private_nh_.getParam("coarse_angle_resolution", coarse_angle_resolution))
    mapper_->setParamCoarseAngleResolution(coarse_angle_resolution);

  // 最小角度补偿，防止评分过小
  double minimum_angle_penalty;
  if (private_nh_.getParam("minimum_angle_penalty", minimum_angle_penalty))
    mapper_->setParamMinimumAnglePenalty(minimum_angle_penalty);

  // 最小距离补偿，防止评分过小
  double minimum_distance_penalty;
  if (private_nh_.getParam("minimum_distance_penalty", minimum_distance_penalty))
    mapper_->setParamMinimumDistancePenalty(minimum_distance_penalty);

  // 在没有发现好的匹配的情况下，是否增加搜索范围
  bool use_response_expansion;
  if (private_nh_.getParam("use_response_expansion", use_response_expansion))
    mapper_->setParamUseResponseExpansion(use_response_expansion);

  // 是否启用后端
  private_nh_.param("use_back_end", use_back_end_, false);
  if (!private_nh_.getParam("solver_type", solver_type_))
    solver_type_ = "spa_solver";
}

karto::ScanSolver *SlamKarto::CreateSolver(std::string solver_type)
{
  karto::ScanSolver *solver_ptr;
  if (solver_type == "spa_solver")
  {
    ROS_INFO("solver type is SpaSolver.");
    solver_ptr = new SpaSolver();
  }
  else if (solver_type == "g2o_solver")
  {
    ROS_INFO("solver type is G2OSolver.");
    solver_ptr = new G2oSolver();
  }
  else if (solver_type == "ceres_solver")
  {
    ROS_INFO("solver type is CeresSolver.");
    solver_ptr = new CeresSolver();
  }
  else if (solver_type == "gtsam_solver")
  {
    ROS_INFO("solver type is GtsamSolver.");
    solver_ptr = new GTSAMSolver();
  }
  else
  {
    ROS_WARN("solver type is unkown.");
    solver_ptr = nullptr;
  }

  return solver_ptr;
}

void SlamKarto::laserCallback(const sensor_msgs::LaserScan::ConstPtr &scan)
{
  laser_count_++;
  if ((laser_count_ % throttle_scans_) != 0)
    return;

  static ros::Time last_map_update(0, 0);

  // Check whether we know about this laser yet
  karto::LaserRangeFinder *laser = getLaser(scan);

  if (!laser)
  {
    ROS_WARN("Failed to create laser device for %s; discarding scan",
             scan->header.frame_id.c_str());
    return;
  }

  karto::Pose2 odom_pose;
  if (addScan(laser, scan, odom_pose))
  {
    ROS_DEBUG("added scan at pose: %.3f %.3f %.3f",
              odom_pose.GetX(),
              odom_pose.GetY(),
              odom_pose.GetHeading());

    publishGraphVisualization();

    if (!got_map_ ||
        (scan->header.stamp - last_map_update) > map_update_interval_)
    {
      if (updateMap())
      {
        last_map_update = scan->header.stamp;
        got_map_ = true;
        ROS_DEBUG("Updated the map");
      }
    }
  }
}

karto::LaserRangeFinder *SlamKarto::getLaser(const sensor_msgs::LaserScan::ConstPtr &scan)
{
  // Check whether we know about this laser yet
  if (lasers_.find(scan->header.frame_id) == lasers_.end())
  {
    // New laser; need to create a Karto device for it.

    // Get the laser's pose, relative to base.
    tf::Stamped<tf::Pose> ident;
    tf::Stamped<tf::Transform> laser_pose;
    ident.setIdentity();
    ident.frame_id_ = scan->header.frame_id;
    ident.stamp_ = scan->header.stamp;
    try
    {
      tf_.transformPose(base_frame_, ident, laser_pose);
    }
    catch (tf::TransformException e)
    {
      ROS_WARN("Failed to compute laser pose, aborting initialization (%s)",
               e.what());
      return NULL;
    }

    double yaw = tf::getYaw(laser_pose.getRotation());

    ROS_INFO("laser %s's pose wrt base: %.3f %.3f %.3f",
             scan->header.frame_id.c_str(),
             laser_pose.getOrigin().x(),
             laser_pose.getOrigin().y(),
             yaw);
    // To account for lasers that are mounted upside-down,
    // we create a point 1m above the laser and transform it into the laser frame
    // if the point's z-value is <=0, it is upside-down

    tf::Vector3 v;
    v.setValue(0, 0, 1 + laser_pose.getOrigin().z());
    tf::Stamped<tf::Vector3> up(v, scan->header.stamp, base_frame_);

    try
    {
      tf_.transformPoint(scan->header.frame_id, up, up);
      ROS_DEBUG("Z-Axis in sensor frame: %.3f", up.z());
    }
    catch (tf::TransformException &e)
    {
      ROS_WARN("Unable to determine orientation of laser: %s", e.what());
      return NULL;
    }

    bool inverse = lasers_inverted_[scan->header.frame_id] = up.z() <= 0;
    if (inverse)
      ROS_INFO("laser is mounted upside-down");

    // Create a laser range finder device and copy in data from the first
    // scan
    std::string name = scan->header.frame_id;
    karto::LaserRangeFinder *laser =
        karto::LaserRangeFinder::CreateLaserRangeFinder(karto::LaserRangeFinder_Custom, karto::Name(name));
    laser->SetOffsetPose(karto::Pose2(laser_pose.getOrigin().x(),
                                      laser_pose.getOrigin().y(),
                                      yaw));
    laser->SetMinimumRange(scan->range_min);
    laser->SetMaximumRange(scan->range_max);
    laser->SetMinimumAngle(scan->angle_min);
    laser->SetMaximumAngle(scan->angle_max);
    laser->SetAngularResolution(scan->angle_increment);
    // TODO: expose this, and many other parameters
    laser->SetRangeThreshold(use_scan_range_);

    // Store this laser device for later
    lasers_[scan->header.frame_id] = laser;

    // Add it to the dataset, which seems to be necessary
    dataset_->Add(laser);
  }

  return lasers_[scan->header.frame_id];
}

bool SlamKarto::addScan(karto::LaserRangeFinder *laser,
                        const sensor_msgs::LaserScan::ConstPtr &scan,
                        karto::Pose2 &karto_pose)
{
  if (!getOdomPose(karto_pose, scan->header.stamp))
    return false;

  // Create a vector of doubles for karto
  std::vector<kt_double> readings;

  if (lasers_inverted_[scan->header.frame_id])
  {
    for (std::vector<float>::const_reverse_iterator it = scan->ranges.rbegin();
         it != scan->ranges.rend();
         ++it)
    {
      readings.push_back(*it);
    }
  }
  else
  {
    for (std::vector<float>::const_iterator it = scan->ranges.begin();
         it != scan->ranges.end();
         ++it)
    {
      readings.push_back(*it);
    }
  }

  // create localized range scan
  karto::LocalizedRangeScan *range_scan =
      new karto::LocalizedRangeScan(laser->GetName(), readings);
  range_scan->SetOdometricPose(karto_pose);
  range_scan->SetCorrectedPose(karto_pose);

  // Add the localized range scan to the mapper
  bool processed;
  if ((processed = mapper_->Process(range_scan)))
  {
    //std::cout << "Pose: " << range_scan->GetOdometricPose() << " Corrected Pose: " << range_scan->GetCorrectedPose() << std::endl;

    karto::Pose2 corrected_pose = range_scan->GetCorrectedPose();

    // Compute the map->odom transform
    tf::Stamped<tf::Pose> odom_to_map;
    try
    {
      tf_.transformPose(odom_frame_,
                        tf::Stamped<tf::Pose>(
                            tf::Transform(tf::createQuaternionFromRPY(0, 0, corrected_pose.GetHeading()),
                                          tf::Vector3(corrected_pose.GetX(), corrected_pose.GetY(), 0.0))
                                .inverse(),
                            scan->header.stamp, base_frame_),
                        odom_to_map);
    }
    catch (tf::TransformException e)
    {
      ROS_ERROR("Transform from base_link to odom failed\n");
      odom_to_map.setIdentity();
    }

    map_to_odom_mutex_.lock();
    map_to_odom_ = tf::Transform(tf::Quaternion(odom_to_map.getRotation()),
                                 tf::Point(odom_to_map.getOrigin()))
                       .inverse();
    map_to_odom_mutex_.unlock();

    // Add the localized range scan to the dataset (for memory management)
    dataset_->Add(range_scan);
  }
  else
    delete range_scan;

  return processed;
}

bool SlamKarto::getOdomPose(karto::Pose2 &karto_pose, const ros::Time &t)
{
  // Get the robot's pose
  tf::Stamped<tf::Pose> ident(tf::Transform(tf::createQuaternionFromRPY(0, 0, 0),
                                            tf::Vector3(0, 0, 0)),
                              t, base_frame_);
  tf::Stamped<tf::Transform> odom_pose;
  try
  {
    tf_.transformPose(odom_frame_, ident, odom_pose);
  }
  catch (tf::TransformException e)
  {
    ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
    return false;
  }
  double yaw = tf::getYaw(odom_pose.getRotation());

  karto_pose = karto::Pose2(odom_pose.getOrigin().x(),
                            odom_pose.getOrigin().y(),
                            yaw);
  return true;
}

bool SlamKarto::updateMap()
{
  boost::mutex::scoped_lock lock(map_mutex_);

  karto::OccupancyGrid *occ_grid =
      karto::OccupancyGrid::CreateFromScans(mapper_->GetAllProcessedScans(), resolution_);

  if (!occ_grid)
    return false;

  if (!got_map_)
  {
    map_.map.info.resolution = resolution_;
    map_.map.info.origin.position.x = 0.0;
    map_.map.info.origin.position.y = 0.0;
    map_.map.info.origin.position.z = 0.0;
    map_.map.info.origin.orientation.x = 0.0;
    map_.map.info.origin.orientation.y = 0.0;
    map_.map.info.origin.orientation.z = 0.0;
    map_.map.info.origin.orientation.w = 1.0;
  }

  // Translate to ROS format
  kt_int32s width = occ_grid->GetWidth();
  kt_int32s height = occ_grid->GetHeight();
  karto::Vector2<kt_double> offset = occ_grid->GetCoordinateConverter()->GetOffset();

  if (map_.map.info.width != (unsigned int)width ||
      map_.map.info.height != (unsigned int)height ||
      map_.map.info.origin.position.x != offset.GetX() ||
      map_.map.info.origin.position.y != offset.GetY())
  {
    map_.map.info.origin.position.x = offset.GetX();
    map_.map.info.origin.position.y = offset.GetY();
    map_.map.info.width = width;
    map_.map.info.height = height;
    map_.map.data.resize(map_.map.info.width * map_.map.info.height);
  }

  for (kt_int32s y = 0; y < height; y++)
  {
    for (kt_int32s x = 0; x < width; x++)
    {
      // Getting the value at position x,y
      kt_int8u value = occ_grid->GetValue(karto::Vector2<kt_int32s>(x, y));

      switch (value)
      {
      case karto::GridStates_Unknown:
        map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = -1;
        break;
      case karto::GridStates_Occupied:
        map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = 100;
        break;
      case karto::GridStates_Free:
        map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = 0;
        break;
      default:
        ROS_WARN("Encountered unknown cell value at %d, %d", x, y);
        break;
      }
    }
  }

  // Set the header information on the map
  map_.map.header.stamp = ros::Time::now();
  map_.map.header.frame_id = map_frame_;

  sst_.publish(map_.map);
  sstm_.publish(map_.map.info);

  delete occ_grid;

  return true;
}

void SlamKarto::publishLoop(double transform_publish_period)
{
  if (transform_publish_period == 0)
    return;

  ros::Rate r(1.0 / transform_publish_period);
  while (ros::ok())
  {
    publishTransform();
    r.sleep();
  }
}

void SlamKarto::publishTransform()
{
  boost::mutex::scoped_lock lock(map_to_odom_mutex_);
  ros::Time tf_expiration = ros::Time::now() + transform_tolerance_;
  tfB_->sendTransform(tf::StampedTransform(map_to_odom_, tf_expiration, map_frame_, odom_frame_));
}

void SlamKarto::publishGraphVisualization()
{
  if (!use_back_end_ || solver_type_ != "spa_solver")
    return;

  std::vector<float> graph;
  static_cast<SpaSolver*>(solver_)->getGraph(graph);

  visualization_msgs::MarkerArray marray;
  visualization_msgs::Marker m;
  m.header.frame_id = map_frame_;
  m.header.stamp = ros::Time::now();
  m.id = 0;
  m.ns = "karto";
  m.type = visualization_msgs::Marker::SPHERE;
  m.pose.position.x = 0.0;
  m.pose.position.y = 0.0;
  m.pose.position.z = 0.0;
  m.scale.x = 0.2;
  m.scale.y = 0.2;
  m.scale.z = 0.2;
  m.color.r = 1.0;
  m.color.g = 0;
  m.color.b = 0.0;
  m.color.a = 1.0;
  m.lifetime = ros::Duration(0);

  visualization_msgs::Marker edge;
  edge.header.frame_id = map_frame_;
  edge.header.stamp = ros::Time::now();
  edge.action = visualization_msgs::Marker::ADD;
  edge.ns = "karto";
  edge.id = 0;
  edge.type = visualization_msgs::Marker::LINE_STRIP;
  edge.scale.x = 0.1;
  edge.scale.y = 0.1;
  edge.scale.z = 0.1;
  edge.color.a = 1.0;
  edge.color.r = 0.0;
  edge.color.g = 0.0;
  edge.color.b = 1.0;

  m.action = visualization_msgs::Marker::ADD;
  uint id = 0;
  for (uint i = 0; i < graph.size() / 2; i++)
  {
    m.id = id;
    m.pose.position.x = graph[2 * i];
    m.pose.position.y = graph[2 * i + 1];
    marray.markers.push_back(visualization_msgs::Marker(m));
    id++;

    if (i > 0)
    {
      edge.points.clear();
      geometry_msgs::Point p;
      p.x = graph[2 * (i - 1)];
      p.y = graph[2 * (i - 1) + 1];
      edge.points.push_back(p);
      p.x = graph[2 * i];
      p.y = graph[2 * i + 1];
      edge.points.push_back(p);
      edge.id = id;

      marray.markers.push_back(visualization_msgs::Marker(edge));
      id++;
    }
  }

  m.action = visualization_msgs::Marker::DELETE;
  for (; id < marker_count_; id++)
  {
    m.id = id;
    marray.markers.push_back(visualization_msgs::Marker(m));
  }

  marker_count_ = marray.markers.size();

  marker_publisher_.publish(marray);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "slam_karto");

  SlamKarto kn;

  ros::spin();

  return 0;
}
