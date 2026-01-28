#include <pcl_localization/pcl_localization_component.hpp>
PCLLocalization::PCLLocalization(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("pcl_localization", options),
  clock_(RCL_ROS_TIME),
  tfbuffer_(std::make_shared<rclcpp::Clock>(clock_)),
  tflistener_(tfbuffer_),
  broadcaster_(this)
{
  declare_parameter("global_frame_id", "map");
  declare_parameter("odom_frame_id", "odom");
  declare_parameter("base_frame_id", "base_link");
  declare_parameter("registration_method", "NDT");
  declare_parameter("score_threshold", 2.0);
  declare_parameter("ndt_resolution", 1.0);
  declare_parameter("ndt_step_size", 0.1);
  declare_parameter("transform_epsilon", 0.01);
  declare_parameter("voxel_leaf_size", 0.2);
  declare_parameter("scan_max_range", 100.0);
  declare_parameter("scan_min_range", 1.0);
  declare_parameter("scan_period", 0.1);
  declare_parameter("use_pcd_map", false);
  declare_parameter("map_path", "/map/map.pcd");
  declare_parameter("use_odom", false);
  declare_parameter("use_imu", false);
  declare_parameter("enable_debug", false);
  declare_parameter("lidar_topic", "/pointcloud");
}

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturn PCLLocalization::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  initializeParameters();
  initializeService(); //创建服务
  initializePubSub();
  initializeRegistration();

  // path_ptr_ = std::make_shared<nav_msgs::msg::Path>();
  // path_ptr_->header.frame_id = global_frame_id_;

  RCLCPP_INFO(get_logger(), "Configuring end");
  return CallbackReturn::SUCCESS;
}

CallbackReturn PCLLocalization::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Activating");

  path_pub_->on_activate();
  initial_map_pub_->on_activate();


  if (use_pcd_map_) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::io::loadPCDFile(map_path_, *map_cloud_ptr);
    RCLCPP_INFO(get_logger(), "Map Size %ld", map_cloud_ptr->size());

    sensor_msgs::msg::PointCloud2::SharedPtr map_msg_ptr(new sensor_msgs::msg::PointCloud2);
    pcl::toROSMsg(*map_cloud_ptr, *map_msg_ptr);
    map_msg_ptr->header.frame_id = global_frame_id_;
    initial_map_pub_->publish(*map_msg_ptr);
    RCLCPP_INFO(get_logger(), "Initial Map Published");

    if (registration_method_ == "GICP" || registration_method_ == "GICP_OMP") {
      pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
      voxel_grid_filter_.setInputCloud(map_cloud_ptr);
      voxel_grid_filter_.filter(*filtered_cloud_ptr);
      registration_->setInputTarget(filtered_cloud_ptr);
    } else {
      registration_->setInputTarget(map_cloud_ptr);
    }

    map_recieved_ = true;
  }

  RCLCPP_INFO(get_logger(), "Activating end");
  return CallbackReturn::SUCCESS;
}

CallbackReturn PCLLocalization::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  path_pub_->on_deactivate();
  initial_map_pub_->on_deactivate();

  RCLCPP_INFO(get_logger(), "Deactivating end");
  return CallbackReturn::SUCCESS;
}

CallbackReturn PCLLocalization::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Cleaning Up");
  initial_pose_sub_.reset();
  initial_map_pub_.reset();
  path_pub_.reset();
  odom_sub_.reset();
  cloud_sub_.reset();
  imu_sub_.reset();

  RCLCPP_INFO(get_logger(), "Cleaning Up end");
  return CallbackReturn::SUCCESS;
}

CallbackReturn PCLLocalization::on_shutdown(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Shutting Down from %s", state.label().c_str());

  return CallbackReturn::SUCCESS;
}

CallbackReturn PCLLocalization::on_error(const rclcpp_lifecycle::State & state)
{
  RCLCPP_FATAL(get_logger(), "Error Processing from %s", state.label().c_str());

  return CallbackReturn::SUCCESS;
}

void PCLLocalization::initializeParameters()
{
  RCLCPP_INFO(get_logger(), "initializeParameters");
  get_parameter("global_frame_id", global_frame_id_);
  get_parameter("odom_frame_id", odom_frame_id_);
  get_parameter("base_frame_id", base_frame_id_);
  get_parameter("registration_method", registration_method_);
  get_parameter("score_threshold", score_threshold_);
  get_parameter("ndt_resolution", ndt_resolution_);
  get_parameter("ndt_step_size", ndt_step_size_);
  get_parameter("ndt_num_threads", ndt_num_threads_);
  get_parameter("transform_epsilon", transform_epsilon_);
  get_parameter("voxel_leaf_size", voxel_leaf_size_);
  get_parameter("scan_max_range", scan_max_range_);
  get_parameter("scan_min_range", scan_min_range_);
  get_parameter("scan_period", scan_period_);
  get_parameter("use_pcd_map", use_pcd_map_);
  get_parameter("map_path", map_path_);

  // 如果命令行传入的map_path为空，则使用yaml配置文件中的路径
  if (map_path_.empty()) {
    RCLCPP_INFO(get_logger(), "No map_path provided via command line, using path from yaml config");
  } else {
    RCLCPP_INFO(get_logger(), "Using map_path from command line: %s", map_path_.c_str());
  }

  get_parameter("set_initial_pose", set_initial_pose_);
  get_parameter("initial_pose_x", initial_pose_x_);
  get_parameter("initial_pose_y", initial_pose_y_);
  get_parameter("initial_pose_z", initial_pose_z_);
  get_parameter("initial_pose_qx", initial_pose_qx_);
  get_parameter("initial_pose_qy", initial_pose_qy_);
  get_parameter("initial_pose_qz", initial_pose_qz_);
  get_parameter("initial_pose_qw", initial_pose_qw_);
  get_parameter("use_odom", use_odom_);
  get_parameter("use_imu", use_imu_);
  get_parameter("enable_debug", enable_debug_);
  get_parameter("lidar_topic", lidar_topic_);

  RCLCPP_INFO(get_logger(),"global_frame_id: %s", global_frame_id_.c_str());
  RCLCPP_INFO(get_logger(),"odom_frame_id: %s", odom_frame_id_.c_str());
  RCLCPP_INFO(get_logger(),"base_frame_id: %s", base_frame_id_.c_str());
  RCLCPP_INFO(get_logger(),"registration_method: %s", registration_method_.c_str());
  RCLCPP_INFO(get_logger(),"ndt_resolution: %lf", ndt_resolution_);
  RCLCPP_INFO(get_logger(),"ndt_step_size: %lf", ndt_step_size_);
  RCLCPP_INFO(get_logger(),"ndt_num_threads: %d", ndt_num_threads_);
  RCLCPP_INFO(get_logger(),"transform_epsilon: %lf", transform_epsilon_);
  RCLCPP_INFO(get_logger(),"voxel_leaf_size: %lf", voxel_leaf_size_);
  RCLCPP_INFO(get_logger(),"scan_max_range: %lf", scan_max_range_);
  RCLCPP_INFO(get_logger(),"scan_min_range: %lf", scan_min_range_);
  RCLCPP_INFO(get_logger(),"scan_period: %lf", scan_period_);
  RCLCPP_INFO(get_logger(),"use_pcd_map: %d", use_pcd_map_);
  RCLCPP_INFO(get_logger(),"map_path: %s", map_path_.c_str());
  RCLCPP_INFO(get_logger(),"set_initial_pose: %d", set_initial_pose_);
  RCLCPP_INFO(get_logger(),"use_odom: %d", use_odom_);
  RCLCPP_INFO(get_logger(),"use_imu: %d", use_imu_);
  RCLCPP_INFO(get_logger(),"enable_debug: %d", enable_debug_);
  RCLCPP_INFO(get_logger(),"lidar_topic: %s", lidar_topic_.c_str());
}

// 初始化服务
void PCLLocalization::initializeService()
{
  localization_service_ = this->create_service<re_localization::srv::ReLocalization>(
    "re_localization",
    std::bind(
      &PCLLocalization::handleLocalizationService,
      this,
      std::placeholders::_1,
      std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(), "✅ 重定位服务初始化成功！");
}

// 服务回调函数
void PCLLocalization::handleLocalizationService(
  const std::shared_ptr<re_localization::srv::ReLocalization::Request> request/*request*/,
  std::shared_ptr<re_localization::srv::ReLocalization::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "✅ 服务调用成功！");

  // 获取初始化位姿
    RCLCPP_INFO(get_logger(), "initialPoseReceived");
  if (request->initial_pose.header.frame_id != global_frame_id_) {
    RCLCPP_WARN(this->get_logger(), "initialpose_frame_id does not match global_frame_id");
    return;
  }
  
  initialpose_recieved_ = true;

  corrent_pose_with_cov_stamped_ptr_ = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>(request->initial_pose); // 强制类型转换为共享指针
  
  RCLCPP_INFO(get_logger(), "initialPoseReceived end");

  // 获取点云
  sensor_msgs::msg::PointCloud2 cloud_msg;
  auto temp_node = std::make_shared<rclcpp::Node>("temp_wait_node");
  bool got_msg = rclcpp::wait_for_message<sensor_msgs::msg::PointCloud2>(
  cloud_msg,
  temp_node,
  lidar_topic_,
  std::chrono::seconds(3));

  if (!got_msg) {
    RCLCPP_WARN(this->get_logger(), "⚠️ 3秒内未接收到点云消息");
    response->success = false;
    return;
  }

  RCLCPP_INFO(this->get_logger(), "✅ 收到点云，开始匹配...");
  
  // 开始匹配
  std::cout << "map_recieved : " << map_recieved_ <<  "initialpose_recieved : " << initialpose_recieved_ << std::endl;
  if (!map_recieved_ || !initialpose_recieved_) {return;}
  RCLCPP_INFO(get_logger(), "cloudReceived");
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(cloud_msg, *cloud_ptr);

  if (use_imu_) {
    double received_time = cloud_msg.header.stamp.sec +
      cloud_msg.header.stamp.nanosec * 1e-9;
    lidar_undistortion_.adjustDistortion(cloud_ptr, received_time);
  }
 
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
  voxel_grid_filter_.setInputCloud(cloud_ptr);
  voxel_grid_filter_.filter(*filtered_cloud_ptr);

  double r;
  pcl::PointCloud<pcl::PointXYZI> tmp;
  for (const auto & p : filtered_cloud_ptr->points) {
    r = sqrt(pow(p.x, 2.0) + pow(p.y, 2.0));
    if (scan_min_range_ < r && r < scan_max_range_) {
      tmp.push_back(p);
    }
  }
  pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_ptr(new pcl::PointCloud<pcl::PointXYZI>(tmp));
  registration_->setInputSource(tmp_ptr);

  Eigen::Affine3d affine;
  tf2::fromMsg(corrent_pose_with_cov_stamped_ptr_->pose.pose, affine);

  Eigen::Matrix4f init_guess = affine.matrix().cast<float>();

  pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  rclcpp::Clock system_clock;
  rclcpp::Time time_align_start = system_clock.now();
  registration_->align(*output_cloud, init_guess);
  rclcpp::Time time_align_end = system_clock.now();

  bool has_converged = registration_->hasConverged();
  double fitness_score = registration_->getFitnessScore();
  if (!has_converged) {
    RCLCPP_WARN(get_logger(), "The registration didn't converge.");
    return;
  }
  if (fitness_score > score_threshold_) {
    RCLCPP_WARN(get_logger(), "The fitness score is over %lf.", score_threshold_);
  }

  Eigen::Matrix4f final_transformation = registration_->getFinalTransformation();
  Eigen::Matrix3d rot_mat = final_transformation.block<3, 3>(0, 0).cast<double>();
  Eigen::Quaterniond quat_eig(rot_mat);
  geometry_msgs::msg::Quaternion quat_msg = tf2::toMsg(quat_eig);

  corrent_pose_with_cov_stamped_ptr_->header.stamp = cloud_msg.header.stamp;
  corrent_pose_with_cov_stamped_ptr_->header.frame_id = global_frame_id_;
  corrent_pose_with_cov_stamped_ptr_->pose.pose.position.x = static_cast<double>(final_transformation(0, 3));
  corrent_pose_with_cov_stamped_ptr_->pose.pose.position.y = static_cast<double>(final_transformation(1, 3));
  corrent_pose_with_cov_stamped_ptr_->pose.pose.position.z = static_cast<double>(final_transformation(2, 3));
  corrent_pose_with_cov_stamped_ptr_->pose.pose.orientation = quat_msg;

  if (enable_debug_) {
    std::cout << "number of filtered cloud points: " << filtered_cloud_ptr->size() << std::endl;
    std::cout << "align time:" << time_align_end.seconds() - time_align_start.seconds() <<
      "[sec]" << std::endl;
    std::cout << "has converged: " << has_converged << std::endl;
    std::cout << "fitness score: " << fitness_score << std::endl;
    std::cout << "final transformation:" << std::endl;
    std::cout << final_transformation << std::endl;
    /* delta_angle check
    * trace(RotationMatrix) = 2(cos(theta) + 1)
    */
    double init_cos_angle = 0.5 *
      (init_guess.coeff(0, 0) + init_guess.coeff(1, 1) + init_guess.coeff(2, 2) - 1);
    double cos_angle = 0.5 *
      (final_transformation.coeff(0,
      0) + final_transformation.coeff(1, 1) + final_transformation.coeff(2, 2) - 1);
    double init_angle = acos(init_cos_angle);
    double angle = acos(cos_angle);
    // Ref:https://twitter.com/Atsushi_twi/status/1185868416864808960
    double delta_angle = abs(atan2(sin(init_angle - angle), cos(init_angle - angle)));
    std::cout << "delta_angle:" << delta_angle * 180 / M_PI << "[deg]" << std::endl;
    std::cout << "-----------------------------------------------------" << std::endl;
  }
    response->success = true;
    response->fitness_score = fitness_score;
    response->pose = *corrent_pose_with_cov_stamped_ptr_;
}

void PCLLocalization::initializePubSub()
{

  RCLCPP_INFO(get_logger(), "initializePubSub");

  path_pub_ = create_publisher<nav_msgs::msg::Path>(
    "path",
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  initial_map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "initial_map",
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  map_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&PCLLocalization::mapReceived, this, std::placeholders::_1));

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "odom", rclcpp::SensorDataQoS(),
    std::bind(&PCLLocalization::odomReceived, this, std::placeholders::_1));

  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    "/imu_raw_simtime2realtime", rclcpp::SensorDataQoS(),
    std::bind(&PCLLocalization::imuReceived, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "initializePubSub end");
}

void PCLLocalization::initializeRegistration()
{
  RCLCPP_INFO(get_logger(), "initializeRegistration");

  if (registration_method_ == "GICP") {
    boost::shared_ptr<pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>> gicp(
      new pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>());
    gicp->setTransformationEpsilon(transform_epsilon_);
    registration_ = gicp;
  }
  else if (registration_method_ == "NDT") {
    boost::shared_ptr<pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>> ndt(
      new pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>());
    ndt->setStepSize(ndt_step_size_);
    ndt->setResolution(ndt_resolution_);
    ndt->setTransformationEpsilon(transform_epsilon_);
    registration_ = ndt;
  }
  else if (registration_method_ == "NDT_OMP") {
    pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>::Ptr ndt_omp(
      new pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>());
    ndt_omp->setStepSize(ndt_step_size_);
    ndt_omp->setResolution(ndt_resolution_);
    ndt_omp->setTransformationEpsilon(transform_epsilon_);
    if (ndt_num_threads_ > 0) {
      ndt_omp->setNumThreads(ndt_num_threads_);
    } else {
      ndt_omp->setNumThreads(omp_get_max_threads());
    }
    registration_ = ndt_omp;
  }
  else if (registration_method_ == "GICP_OMP") {
    pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>::Ptr gicp_omp(
      new pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>());
    gicp_omp->setTransformationEpsilon(transform_epsilon_);
    registration_ = gicp_omp;
  }
  else {
    RCLCPP_ERROR(get_logger(), "Invalid registration method.");
    exit(EXIT_FAILURE);
  }


  voxel_grid_filter_.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
  RCLCPP_INFO(get_logger(), "initializeRegistration end");
}



void PCLLocalization::mapReceived(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "mapReceived");
  pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);

  if (msg->header.frame_id != global_frame_id_) {
    RCLCPP_WARN(this->get_logger(), "map_frame_id does not match　global_frame_id");
    return;
  }

  pcl::fromROSMsg(*msg, *map_cloud_ptr);

  if (registration_method_ == "GICP" || registration_method_ == "GICP_OMP") {
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    voxel_grid_filter_.setInputCloud(map_cloud_ptr);
    voxel_grid_filter_.filter(*filtered_cloud_ptr);
    registration_->setInputTarget(filtered_cloud_ptr);

  } else {
    registration_->setInputTarget(map_cloud_ptr);
  }

  map_recieved_ = true;
  RCLCPP_INFO(get_logger(), "mapReceived end");
}


void PCLLocalization::odomReceived(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  if (!use_odom_) {return;}
  RCLCPP_INFO(get_logger(), "odomReceived");

  double current_odom_received_time = msg->header.stamp.sec +
    msg->header.stamp.nanosec * 1e-9;
  double dt_odom = current_odom_received_time - last_odom_received_time_;
  last_odom_received_time_ = current_odom_received_time;
  if (dt_odom > 1.0 /* [sec] */) {
    RCLCPP_WARN(this->get_logger(), "odom time interval is too large");
    return;
  }
  if (dt_odom < 0.0 /* [sec] */) {
    RCLCPP_WARN(this->get_logger(), "odom time interval is negative");
    return;
  }

  tf2::Quaternion previous_quat_tf;
  double roll, pitch, yaw;
  tf2::fromMsg(corrent_pose_with_cov_stamped_ptr_->pose.pose.orientation, previous_quat_tf);

  tf2::Matrix3x3(previous_quat_tf).getRPY(roll, pitch, yaw);

  roll += msg->twist.twist.angular.x * dt_odom;
  pitch += msg->twist.twist.angular.y * dt_odom;
  yaw += msg->twist.twist.angular.z * dt_odom;

  Eigen::Quaterniond quat_eig =
    Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
    Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());

  geometry_msgs::msg::Quaternion quat_msg = tf2::toMsg(quat_eig);

  Eigen::Vector3d odom{
    msg->twist.twist.linear.x,
    msg->twist.twist.linear.y,
    msg->twist.twist.linear.z};
  Eigen::Vector3d delta_position = quat_eig.matrix() * dt_odom * odom;

  corrent_pose_with_cov_stamped_ptr_->pose.pose.position.x += delta_position.x();
  corrent_pose_with_cov_stamped_ptr_->pose.pose.position.y += delta_position.y();
  corrent_pose_with_cov_stamped_ptr_->pose.pose.position.z += delta_position.z();
  corrent_pose_with_cov_stamped_ptr_->pose.pose.orientation = quat_msg;
}

void PCLLocalization::imuReceived(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
  if (!use_imu_) {return;}

  sensor_msgs::msg::Imu tf_converted_imu;

  try {
    const geometry_msgs::msg::TransformStamped transform = tfbuffer_.lookupTransform(
     base_frame_id_, msg->header.frame_id, tf2::TimePointZero);

    geometry_msgs::msg::Vector3Stamped angular_velocity, linear_acceleration, transformed_angular_velocity, transformed_linear_acceleration;
    geometry_msgs::msg::Quaternion  transformed_quaternion;

    angular_velocity.header = msg->header;
    angular_velocity.vector = msg->angular_velocity;
    linear_acceleration.header = msg->header;
    linear_acceleration.vector = msg->linear_acceleration;

    tf2::doTransform(angular_velocity, transformed_angular_velocity, transform);
    tf2::doTransform(linear_acceleration, transformed_linear_acceleration, transform);

    tf_converted_imu.angular_velocity = transformed_angular_velocity.vector;
    tf_converted_imu.linear_acceleration = transformed_linear_acceleration.vector;
    tf_converted_imu.orientation = transformed_quaternion;

  }
  catch (tf2::TransformException& ex)
  {
    std::cout << "Failed to lookup transform" << std::endl;
    RCLCPP_WARN(this->get_logger(), "Failed to lookup transform.");
    return;
  }

  Eigen::Vector3f angular_velo{tf_converted_imu.angular_velocity.x, tf_converted_imu.angular_velocity.y,
    tf_converted_imu.angular_velocity.z};
  Eigen::Vector3f acc{tf_converted_imu.linear_acceleration.x, tf_converted_imu.linear_acceleration.y, tf_converted_imu.linear_acceleration.z};
  Eigen::Quaternionf quat{msg->orientation.w, msg->orientation.x, msg->orientation.y,
    msg->orientation.z};
  double imu_time = msg->header.stamp.sec +
    msg->header.stamp.nanosec * 1e-9;

  lidar_undistortion_.getImu(angular_velo, acc, quat, imu_time);

}
