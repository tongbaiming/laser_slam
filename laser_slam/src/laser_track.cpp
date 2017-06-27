#include "laser_slam/laser_track.hpp"

#include <gtsam/nonlinear/Marginals.h>

using namespace gtsam;
using namespace curves;

namespace laser_slam {

LaserTrack::LaserTrack(const LaserTrackParams& parameters,
                       unsigned int laser_track_id) : params_(parameters),
                           laser_track_id_(laser_track_id) {
  // Load the ICP configurations.
  std::ifstream ifs_icp_configurations(params_.icp_configuration_file.c_str());
  if (ifs_icp_configurations.good()) {
    LOG(INFO) << "Loading ICP configurations from: " << params_.icp_configuration_file;
    icp_.loadFromYaml(ifs_icp_configurations);
  } else {
    LOG(WARNING) << "Could not open ICP configuration file. Using default configuration.";
    icp_.setDefault();
  }

  // Load the ICP input filters configurations.
  std::cout << "params_.icp_input_filters_file: " << params_.icp_input_filters_file << std::endl;
  std::ifstream ifs_input_filters(params_.icp_input_filters_file.c_str());
  if (ifs_input_filters.good()) {
    LOG(INFO) << "Loading ICP input filters from: " << params_.icp_input_filters_file;
    input_filters_ = PointMatcher::DataPointsFilters(ifs_input_filters);
  } else {
    LOG(FATAL) << "Could not open ICP input filters configuration file.";
  }

  // Create a rigid transformation.
  rigid_transformation_ = PointMatcher::get().REG(Transformation).create("RigidTransformation");
  CHECK_NOTNULL(rigid_transformation_);

  // Create the noise models.
  using namespace gtsam::noiseModel;
  if (params_.add_m_estimator_on_odom) {
    LOG(INFO) << "Creating odometry noise model with cauchy.";
    odometry_noise_model_  = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Cauchy::Create(1),
        gtsam::noiseModel::Diagonal::Sigmas(params_.odometry_noise_model));
  } else {
    odometry_noise_model_ = gtsam::noiseModel::Diagonal::Sigmas(params_.odometry_noise_model);
  }

  if (params_.add_m_estimator_on_icp) {
    LOG(INFO) << "Creating ICP noise model with cauchy.";
    icp_noise_model_  = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Cauchy::Create(1),
        gtsam::noiseModel::Diagonal::Sigmas(params_.icp_noise_model));
  } else {
    icp_noise_model_ = gtsam::noiseModel::Diagonal::Sigmas(params_.icp_noise_model);
  }

  Eigen::Matrix<double,6,1> noise;
  noise(0) = 0.0000001;
  noise(1) = 0.0000001;
  noise(2) = 0.0000001;
  noise(3) = 0.0000001;
  noise(4) = 0.0000001;
  noise(5) = 0.0000001;

  prior_noise_model_ = gtsam::noiseModel::Diagonal::Sigmas(noise);
}

void LaserTrack::processPose(const Pose& pose) {
  std::lock_guard<std::recursive_mutex> lock(full_laser_track_mutex_);
  if (pose_measurements_.empty() && pose.time_ns != 0) {
    LOG(WARNING) << "First pose had timestamp different than 0 (" << pose.time_ns << ".";
  }
  pose_measurements_.push_back(pose);
}

void LaserTrack::processLaserScan(const LaserScan& in_scan) {
  std::lock_guard<std::recursive_mutex> lock(full_laser_track_mutex_);
  LaserScan scan = in_scan;

  // Apply the input filters.
  Clock clock;
  //input_filters_.apply(scan.scan);
  clock.takeTime();
  LOG(INFO) << "Took " << clock.getRealTime() << " ms to filter the input scan.";

  // Compute the relative pose measurement, extend the trajectory and
  // compute the ICP transformations.
  if (trajectory_.isEmpty()) {
    scan.key = extendTrajectory(scan.time_ns, getPoseMeasurement(scan.time_ns));
  } else {
    // Evaluate the pose measurement at the last trajectory node.
    SE3 last_pose_measurement = getPoseMeasurement(trajectory_.getMaxTime());

    // Evaluate the pose measurement at the new trajectory node.
    SE3 new_pose_measurement = getPoseMeasurement(scan.time_ns);

    // Create the relative pose measurement.
    RelativePose relative_measurement;
    relative_measurement.T_a_b = last_pose_measurement.inverse()*new_pose_measurement;
    relative_measurement.time_a_ns = trajectory_.getMaxTime();
    relative_measurement.key_a = getPoseKey(trajectory_.getMaxTime());
    relative_measurement.time_b_ns = scan.time_ns;

    // Extend the trajectory with the new node position.
    scan.key =  extendTrajectory(scan.time_ns, trajectory_.evaluate(trajectory_.getMaxTime()) *
                                 relative_measurement.T_a_b);

    // Complete and save the relative_measurement.
    relative_measurement.key_b = scan.key;
    odometry_measurements_.push_back(relative_measurement);

    // Compute the ICP transformations.
    if (params_.use_icp_factors) {
      computeICPTransformations();
    }
  }

  // Update the pose key and save the scan.
  setPoseKey(scan.time_ns, scan.key);
  laser_scans_.push_back(scan);
}
/*函数processPoseAndLaserScan中传入的参数pose，是由tf_transform转化而来，调用此函数的是laser_slam_ros中的scanCallbak回调函数，
其中tf_transform是从sensor frame到odom frame的转换，tbm将sensor frame设置为/base_link_inertia，讲odom frame设置为/world
因此laser_track中的pose_measurement_中储存的都是这种相对由里程计坐标系的pose*/
void LaserTrack::processPoseAndLaserScan(const Pose& pose, const LaserScan& in_scan,
                                         gtsam::NonlinearFactorGraph* newFactors,
                                         gtsam::Values* newValues,
                                         bool* is_prior) {
  std::lock_guard<std::recursive_mutex> lock(full_laser_track_mutex_);

  Clock scan_matching_clock;

  if (pose.time_ns != in_scan.time_ns) {
    LOG(WARNING) << "The time of the pose to add (" << pose.time_ns << ") does not match the " <<
        "time of the scan to add (" << in_scan.time_ns << ").";
  }

  if (newFactors != NULL) {
    //todo clear without failure.
    CHECK(newFactors->empty());
  }
  if (newValues != NULL) {
    newValues->clear();
  }
  //pose是由laser_slam_worker中的tf_transform得到的,而tf_transform的时间戳与cloud_msg_in的时间戳是一致的
  //in_scan是由cloud_msg_in转化而来的，二者时间戳一致，因此本函数中的参数pose与局部变量scan的时间戳是一致的
  LaserScan scan = in_scan;

  // Apply the input filters.
  fprintf(stderr, "I am in LaserTrack::processPoseAndLaserScan(), flag 1\n" );
  Clock clock;
  //input_filters_.apply(scan.scan);
  clock.takeTime();
  fprintf(stderr, "I am in LaserTrack::processPoseAndLaserScan(), flag 2\n" );
  LOG(INFO) << "Took " << clock.getRealTime() << " ms to filter the input scan.";

  // Save the pose measurement.
  if (pose_measurements_.empty() && pose.time_ns != 0) {
    LOG(WARNING) << "First pose had timestamp different than 0 (" << pose.time_ns << ".";
  }
  pose_measurements_.push_back(pose);

  // Compute the relative pose measurement, extend the trajectory and
  // compute the ICP transformations.
  if (trajectory_.isEmpty()) {
    //由于pose和scan的时间戳是一致的，这里恰好可以得到pose_measurements_中刚刚存储的pose
    //extendTrajectory函数向trajectory_成员变量中加入刚刚得到的pose
    scan.key = extendTrajectory(scan.time_ns, getPoseMeasurement(scan.time_ns));

    // Update the pose key and save the scan.
    setPoseKey(scan.time_ns, scan.key);
    laser_scans_.push_back(scan);

    if (newFactors != NULL) {
      Pose prior_pose = pose;
      // Add a prior on the first key.
      //问题在这里，如果yaml文件中将参数force_priors设置为true那么这里将采用如下固定的值，所以发布的local map对不准
      //tbm要将yaml文件中的参数改成false
      if (params_.force_priors) {
        prior_pose.T_w = SE3(SE3::Rotation(1.0, 0.0, 0.0, 0.0),
                             SE3::Position(0.0,
                                           kDistanceBetweenPriorPoses_m * laser_track_id_, 0.0));
      }

      newFactors->push_back(makeMeasurementFactor(prior_pose, prior_noise_model_));
    }
    if (is_prior != NULL) {
      *is_prior = true;
    }
  } else {
    // Evaluate the pose measurement at the last trajectory node.
    SE3 last_pose_measurement = getPoseMeasurement(trajectory_.getMaxTime());

    // Evaluate the pose measurement at the new trajectory node.
    SE3 new_pose_measurement = getPoseMeasurement(scan.time_ns);

    // Create the relative pose measurement.
    RelativePose relative_measurement;
    relative_measurement.T_a_b = last_pose_measurement.inverse()*new_pose_measurement;
    relative_measurement.time_a_ns = trajectory_.getMaxTime();
    relative_measurement.key_a = getPoseKey(trajectory_.getMaxTime());
    relative_measurement.time_b_ns = scan.time_ns;

    // Extend the trajectory with the new node position.
    scan.key =  extendTrajectory(scan.time_ns, trajectory_.evaluate(trajectory_.getMaxTime()) *
                                 relative_measurement.T_a_b);

    // Update the pose key and save the scan.
    setPoseKey(scan.time_ns, scan.key);
    laser_scans_.push_back(scan);

    // Complete and save the relative_measurement.
    relative_measurement.key_b = scan.key;
    odometry_measurements_.push_back(relative_measurement);

    // Compute the ICP transformations.
    if (params_.use_icp_factors) {
      computeICPTransformations();
    }

    scan_matching_clock.takeTime();
    scan_matching_times_.emplace(scan.time_ns, scan_matching_clock.getRealTime());

    if (newFactors != NULL) {
      // Add the odometry and ICP factors.
      if (params_.use_odom_factors) {
        newFactors->push_back(makeRelativeMeasurementFactor(relative_measurement,
                                                            odometry_noise_model_));
      }

      if (params_.use_icp_factors) {
        newFactors->push_back(makeRelativeMeasurementFactor(
            icp_transformations_[icp_transformations_.size()-1u], icp_noise_model_));
      }
    }
    if (is_prior != NULL) {
      *is_prior = false;
    }
  }

  if (newValues != NULL) {
    newValues->insert(scan.key, pose.T_w);
  }
}

void LaserTrack::getLastPointCloud(DataPoints* out_point_cloud) const {
  std::lock_guard<std::recursive_mutex> lock(full_laser_track_mutex_);
  CHECK_NOTNULL(out_point_cloud);
  // todo
}

void LaserTrack::getPointCloudOfTimeInterval(const std::pair<Time, Time>& times_ns,
                                             DataPoints* out_point_cloud) const {
  std::lock_guard<std::recursive_mutex> lock(full_laser_track_mutex_);
  CHECK_NOTNULL(out_point_cloud);
  *out_point_cloud = DataPoints();
  // todo
}

void LaserTrack::getLocalCloudInWorldFrame(const Time& timestamp_ns,
                                           DataPoints* out_point_cloud) const {
  std::lock_guard<std::recursive_mutex> lock(full_laser_track_mutex_);
  CHECK_NOTNULL(out_point_cloud);

  // Find an iterator to the local scan.
  std::vector<LaserScan>::const_iterator it = laser_scans_.end();
  do {
    --it;
  } while (it != laser_scans_.begin() && it->time_ns != timestamp_ns);
  CHECK(it->time_ns == timestamp_ns) << "The requested local scan could not be found:";

  // Get the rigid transformation from the trajectory to transform the scan in world frame.
  PointMatcher::TransformationParameters transformation_matrix =
      trajectory_.evaluate(timestamp_ns).getTransformationMatrix().cast<float>();
  correctTransformationMatrix(&transformation_matrix);

  // Transform the scan in world frame.
  *out_point_cloud = rigid_transformation_->compute(it->scan,transformation_matrix);
}

void LaserTrack::getTrajectory(Trajectory* trajectory) const {
  std::lock_guard<std::recursive_mutex> lock(full_laser_track_mutex_);
  CHECK_NOTNULL(trajectory)->clear();

  std::vector<Time> trajectory_times_ns;
  trajectory_.getCurveTimes(&trajectory_times_ns);

  for (auto time_ns: trajectory_times_ns) {
    trajectory->emplace(time_ns, trajectory_.evaluate(time_ns));
  }
}

const std::vector<LaserScan>& LaserTrack::getLaserScans() const {
  std::lock_guard<std::recursive_mutex> lock(full_laser_track_mutex_);
  return laser_scans_;
}

void LaserTrack::getCovariances(std::vector<Covariance>* out_covariances) const {
  std::lock_guard<std::recursive_mutex> lock(full_laser_track_mutex_);
  CHECK_NOTNULL(out_covariances)->clear();
  *out_covariances = covariances_;
}

Pose LaserTrack::getCurrentPose() const {
  std::lock_guard<std::recursive_mutex> lock(full_laser_track_mutex_);
  Pose current_pose;
  if (!trajectory_.isEmpty()) {
    current_pose.time_ns = getMaxTime();
    current_pose.T_w = trajectory_.evaluate(current_pose.time_ns);
  }
  return current_pose;
}

Pose LaserTrack::getPreviousPose() const {
  std::lock_guard<std::recursive_mutex> lock(full_laser_track_mutex_);
  Pose previous_pose;
  if (trajectory_.size() > 1u) {
    std::vector<Time> trajectory_times;
    trajectory_.getCurveTimes(&trajectory_times);
    previous_pose.time_ns = *(++trajectory_times.rbegin());
    previous_pose.T_w = trajectory_.evaluate(previous_pose.time_ns);
  }
  return previous_pose;
}

void LaserTrack::getOdometryTrajectory(Trajectory* trajectory) const {
  std::lock_guard<std::recursive_mutex> lock(full_laser_track_mutex_);
  CHECK_NOTNULL(trajectory)->clear();
  for (const auto& pose: pose_measurements_) {
    trajectory->emplace(pose.time_ns, pose.T_w);
  }
}

Time LaserTrack::getMinTime() const {
  std::lock_guard<std::recursive_mutex> lock(full_laser_track_mutex_);
  return trajectory_.getMinTime();
}

Time LaserTrack::getMaxTime() const {
  std::lock_guard<std::recursive_mutex> lock(full_laser_track_mutex_);
  return trajectory_.getMaxTime();
}

void LaserTrack::getLaserScansTimes(std::vector<curves::Time>* out_times_ns) const {
  std::lock_guard<std::recursive_mutex> lock(full_laser_track_mutex_);
  CHECK_NOTNULL(out_times_ns)->clear();
  for (size_t i = 0u; i < laser_scans_.size(); ++i) {
    out_times_ns->push_back(laser_scans_[i].time_ns);
  }
}

void LaserTrack::appendPriorFactors(const Time& prior_time_ns, NonlinearFactorGraph* graph) const {
  std::lock_guard<std::recursive_mutex> lock(full_laser_track_mutex_);
  CHECK_NOTNULL(graph);

  trajectory_.addPriorFactors(graph, prior_time_ns);
}

//tbm:LaserTrack这个类能够根据自身的里程计信息,和扫描的激光点云的匹配构造gtsam中因子图的factor
void LaserTrack::appendOdometryFactors(const curves::Time& optimization_min_time_ns,
                                       const curves::Time& optimization_max_time_ns,
                                       noiseModel::Base::shared_ptr noise_model,
                                       NonlinearFactorGraph* graph) const {
  std::lock_guard<std::recursive_mutex> lock(full_laser_track_mutex_);
  CHECK_NOTNULL(graph);

  //TODO(Renaud): this can be optimized but let's see when we fix issue #27.
  for (const auto& odometry_measurement: odometry_measurements_) {
    if (odometry_measurement.time_a_ns >= optimization_min_time_ns &&
        odometry_measurement.time_b_ns <= optimization_max_time_ns) {
      graph->push_back(makeRelativeMeasurementFactor(odometry_measurement, noise_model));
    }
  }
}

void LaserTrack::appendICPFactors(const curves::Time& optimization_min_time_ns,
                                  const curves::Time& optimization_max_time_ns,
                                  noiseModel::Base::shared_ptr noise_model,
                                  NonlinearFactorGraph* graph) const {
  std::lock_guard<std::recursive_mutex> lock(full_laser_track_mutex_);
  CHECK_NOTNULL(graph);

  for (size_t i = 0u; i < icp_transformations_.size(); ++i) {
    // If the second node falls within the optimization window.
    if (icp_transformations_[i].time_b_ns >= optimization_min_time_ns &&
        icp_transformations_[i].time_b_ns <= optimization_max_time_ns) {

      // If the first node also falls within the optimization window.
      if (icp_transformations_[i].time_a_ns >= optimization_min_time_ns &&
          icp_transformations_[i].time_a_ns <= optimization_max_time_ns) {
        graph->push_back(makeRelativeMeasurementFactor(icp_transformations_[i], noise_model));
      } else {
        graph->push_back(makeRelativeMeasurementFactor(icp_transformations_[i],
                                                       noise_model, true));
      }
    }
  }
}

//TODO(renaud): That's basically the same as above, just with a different variable. Can we combine?
void LaserTrack::appendLoopClosureFactors(const curves::Time& optimization_min_time_ns,
                                          const curves::Time& optimization_max_time_ns,
                                          noiseModel::Base::shared_ptr noise_model,
                                          NonlinearFactorGraph* graph) const {
  std::lock_guard<std::recursive_mutex> lock(full_laser_track_mutex_);
  CHECK_NOTNULL(graph);

  for (size_t i = 0u; i < loop_closures_.size(); ++i) {
    // If the second node falls within the optimization window.
    if (loop_closures_[i].time_b_ns >= optimization_min_time_ns &&
        loop_closures_[i].time_b_ns <= optimization_max_time_ns) {

      // If the first node also falls within the optimization window.
      if (loop_closures_[i].time_a_ns >= optimization_min_time_ns &&
          loop_closures_[i].time_a_ns <= optimization_max_time_ns) {
        graph->push_back(makeRelativeMeasurementFactor(loop_closures_[i], noise_model));
      } else {
        graph->push_back(makeRelativeMeasurementFactor(loop_closures_[i],
                                                       noise_model, true));
      }
    }
  }
}

//tbm:gtsam所需要的全部数据都来源与这个LaserTrack类
void LaserTrack::initializeGTSAMValues(const KeySet& keys, Values* values) const {
  std::lock_guard<std::recursive_mutex> lock(full_laser_track_mutex_);
  trajectory_.initializeGTSAMValues(keys, values);
}

void LaserTrack::updateFromGTSAMValues(const Values& values) {
  std::lock_guard<std::recursive_mutex> lock(full_laser_track_mutex_);
  trajectory_.updateFromGTSAMValues(values);
}

//tbm:并且,LaserTrack根据gtsam平滑后的结果更新自身保存的轨迹，也就是trajectory ,同时也更新
void LaserTrack::updateCovariancesFromGTSAMValues(const gtsam::NonlinearFactorGraph& factor_graph,
                                                  const gtsam::Values& values) {
  std::lock_guard<std::recursive_mutex> lock(full_laser_track_mutex_);
  gtsam::KeySet keys = factor_graph.keys();
  gtsam::Marginals marginals(factor_graph, values);
  for (const auto& key: keys) {
    covariances_.push_back(marginals.marginalCovariance(key));
  }
}

//tbm:根据相对位移构造因子，这里作为相对位移的类型是LaserTrack类中自己定义的一个类型RelativePose
ExpressionFactor<SE3>
LaserTrack::makeRelativeMeasurementFactor(const RelativePose& relative_pose_measurement,
                                          noiseModel::Base::shared_ptr noise_model,
                                          const bool fix_first_node) const {
  Expression<SE3> T_w_b(trajectory_.getValueExpression(relative_pose_measurement.time_b_ns));

  // If fix_first_node is true, a constant SE3 expression is used for T_w_a. That is, this
  // node will be fixed and not be part of the factor.
  // TODO(Renaud): is there a cleaner way of doing this?
  if (fix_first_node) {
    Expression<SE3> T_w_a(trajectory_.evaluate(relative_pose_measurement.time_a_ns));
    Expression<SE3> T_a_w(kindr::minimal::inverse(T_w_a));
    Expression<SE3> relative(kindr::minimal::compose(T_a_w, T_w_b));
    return ExpressionFactor<SE3>(noise_model,relative_pose_measurement.T_a_b, relative);
  } else {
    Expression<SE3> T_w_a(trajectory_.getValueExpression(relative_pose_measurement.time_a_ns));
    Expression<SE3> T_a_w(kindr::minimal::inverse(T_w_a));
    Expression<SE3> relative(kindr::minimal::compose(T_a_w, T_w_b));
    return ExpressionFactor<SE3>(noise_model,relative_pose_measurement.T_a_b, relative);
  }
}

gtsam::ExpressionFactor<SE3>
LaserTrack::makeMeasurementFactor(const Pose& pose_measurement,
                                  gtsam::noiseModel::Base::shared_ptr noise_model) const {
  Expression<SE3> T_w(trajectory_.getValueExpression(pose_measurement.time_ns));
  return ExpressionFactor<SE3>(noise_model,pose_measurement.T_w, T_w);
}

void LaserTrack::computeICPTransformations() {
  if (getNumScans() > 1u) {
    Clock clock;
    localScanToSubMap();
    clock.takeTime();
    LOG(INFO) << "Took " << clock.getRealTime() << " ms to compute the ICP transformations.";
  }
}

void LaserTrack::localScanToSubMap() {
  //tbm:LaserScan这个结构体类型也是在laser_slam命名空间中定义的，在文件common.hpp中，表示一个点云。
  LaserScan last_scan = laser_scans_.back();
  //tbm:RelativePose这个类型是在laser_slam命名空间中定义的，在文件common.hpp中
  RelativePose icp_transformation;
  icp_transformation.time_b_ns = last_scan.time_ns;
  icp_transformation.time_a_ns = laser_scans_[getNumScans() - 2u].time_ns;

  // Transform the last (parameters_.nscan_in_sub_map - 1) scans
  // in the frame of the second last scan.
  Clock clock;
  const SE3 T_w_to_second_last_scan = trajectory_.evaluate(
      laser_scans_[getNumScans() - 2u].time_ns);
  DataPoints sub_map = laser_scans_[getNumScans() - 2u].scan;
  PointMatcher::TransformationParameters transformation_matrix;
  //tbm:计算若干(由参数params_.nscan_in_sub_map决定)之前的LaserScan与倒数第二个LaserScan之间的转移矩阵,
  //tbm:并将他们连接到倒数第二个LaserScan上以构成submap
  for (size_t i = 0u; i < std::min(getNumScans() - 2u, size_t(params_.nscan_in_sub_map - 1u)); ++i) {
    LaserScan previous_scan = laser_scans_[getNumScans() - 3u - i];
    transformation_matrix = (T_w_to_second_last_scan.inverse() *
        trajectory_.evaluate(previous_scan.time_ns)).getTransformationMatrix().cast<float>();

    correctTransformationMatrix(&transformation_matrix);

    sub_map.concatenate(rigid_transformation_->compute(previous_scan.scan,transformation_matrix));
  }
  clock.takeTime();
  LOG(INFO) << "Took " << clock.getRealTime() << " ms to build the submap.";
  //tbm:这个initial_guess就是最新的LaserScan与倒数第二个LaserScan之间的转移矩阵
  //tbm:需要注意的是,这里全部用的是trajectory_.evalue(icp_transformation.time_b_ns)来得到最新LaserScan的位姿。
  //tbm:这个icp_transformation.time_b_ns在上面定义为last_scan.time_ns
  // Obtain the initial guess from the trajectory.
  SE3 initial_guess = trajectory_.evaluate(icp_transformation.time_a_ns).inverse() *
      trajectory_.evaluate(icp_transformation.time_b_ns);
  transformation_matrix = initial_guess.getTransformationMatrix().cast<float>();

  PointMatcher::TransformationParameters icp_solution = transformation_matrix;
  //tbm:利用ethz的libpointmatcher包中包装后的icp算法计算，初始值是由initial_guess转化而来的transformation_matrix
  // Compute the ICP solution.
  try {
    icp_solution = icp_.compute(last_scan.scan, sub_map, transformation_matrix);
  }

  catch (PointMatcher::ConvergenceError error)
  {
    //LOG(INFO) << "ICP failed to converge: " << error.what();
  }
  //tbm:rigid_transformation的类型是PointMatcher::Transformation*,
  if (params_.save_icp_results) {
    last_scan.scan.save("/tmp/last_scan.vtk");
    sub_map.save("/tmp/sub_map.vtk");
    correctTransformationMatrix(&transformation_matrix);
    rigid_transformation_->compute(last_scan.scan,transformation_matrix).save(
        "/tmp/last_scan_alligned_by_initial_guess.vtk");
    correctTransformationMatrix(&icp_solution);
    rigid_transformation_->compute(last_scan.scan,icp_solution).save(
        "/tmp/last_scan_alligned_by_solution.vtk");
  }

  icp_transformation.T_a_b = convertTransformationMatrixToSE3(icp_solution);
  icp_transformation.key_a = getPoseKey(icp_transformation.time_a_ns);
  icp_transformation.key_b = getPoseKey(icp_transformation.time_b_ns);
  icp_transformations_.push_back(icp_transformation);
}

Pose* LaserTrack::findPose(const Time& timestamp_ns) {
  CHECK(!pose_measurements_.empty()) << "Cannot register the scan as no pose was registered.";
  CHECK_LE(timestamp_ns, pose_measurements_.back().time_ns) << "The requested time ("
      << timestamp_ns << ") does not exist in the pose measurements. Last pose time is "
      << pose_measurements_.back().time_ns << ".";

  // Find an iterator to the pose measurement at the requested time.
  PoseVector::iterator it = pose_measurements_.end();
  do {
    --it;
  } while (it != pose_measurements_.begin() && it->time_ns != timestamp_ns);

  CHECK_EQ(it->time_ns, timestamp_ns) 
  << "The requested time does not exist in the pose measurements.";

  return &(*it);
}

Pose LaserTrack::findPose(const Time& timestamp_ns) const {
  CHECK(!pose_measurements_.empty()) << "Cannot register the scan as no pose was registered.";
  CHECK_LE(timestamp_ns, pose_measurements_.back().time_ns) << "The requested time ("
      << timestamp_ns << ") does not exist in the pose measurements. Last pose time is "
      << pose_measurements_.back().time_ns << ".";

  // Find an iterator to the pose measurement at the requested time.
  PoseVector::const_iterator it = pose_measurements_.end();
  do {
    --it;
  } while (it != pose_measurements_.begin() && it->time_ns != timestamp_ns);

  CHECK_EQ(it->time_ns, timestamp_ns)
  << "The requested time does not exist in the pose measurements.";

  return *it;
}

Pose LaserTrack::findNearestPose(const Time& timestamp_ns) const {
  std::lock_guard<std::recursive_mutex> lock(full_laser_track_mutex_);
  CHECK(!pose_measurements_.empty()) << "Cannot find nearest pose as no pose was registered.";
  CHECK_LE(timestamp_ns, pose_measurements_.back().time_ns) << "The requested time ("
      << timestamp_ns << ") is later than the latest pose time. Latest pose time is "
      << pose_measurements_.back().time_ns << ".";

  Pose pose;
  pose.time_ns = timestamp_ns;
  pose.T_w = trajectory_.evaluate(timestamp_ns);
  // Not used.
  pose.key = Key();

  return pose;
}

Key LaserTrack::extendTrajectory(const Time& timestamp_ns, const SE3& value) {
  std::vector<Time> times_ns;
  std::vector<SE3> values;
  std::vector<Key> keys;
  times_ns.push_back(timestamp_ns);
  values.push_back(value);
  trajectory_.extend(times_ns, values, &keys);
  CHECK_EQ(keys.size(), 1u);
  return keys[0];
}

std::vector<LaserScan>::const_iterator LaserTrack::getIteratorToScanAtTime(
    const curves::Time& time_ns) const {
  bool found = false;
  std::vector<LaserScan>::const_iterator it = laser_scans_.begin();
  while (it != laser_scans_.end() && !found) {
    if (it->time_ns == time_ns) {
      found = true;
    } else {
      ++it;
    }
  }
  if (it == laser_scans_.end()) {
    CHECK(false) << "Could not find the scan.";
  }
  CHECK_EQ(time_ns, it->time_ns);
  return it;
}

void LaserTrack::buildSubMapAroundTime(const curves::Time& time_ns,
                                       const unsigned int sub_maps_radius,
                                       DataPoints* sub_map_out) const {
  std::lock_guard<std::recursive_mutex> lock(full_laser_track_mutex_);
  LOG(INFO) << "buildSubMapAroundTime " << time_ns << " for track " <<
      laser_track_id_;
  CHECK_NOTNULL(sub_map_out);
  const SE3 T_w_a = trajectory_.evaluate(time_ns);

  std::vector<LaserScan>::const_iterator it = getIteratorToScanAtTime(time_ns);
  DataPoints sub_map = it->scan;
  std::vector<LaserScan>::const_iterator it_before = it;
  std::vector<LaserScan>::const_iterator it_after = it;

  PointMatcher::TransformationParameters transformation_matrix;

  // Add the scans with decreasing time stamps.
  bool reached_begin = false;
  if (it_before != laser_scans_.begin()) {
    for (unsigned int i = 0u; i < sub_maps_radius; ++i) {
      if (!reached_begin) {
        --it_before;
        if (it_before == laser_scans_.begin()) {
          reached_begin = true;
        }
        transformation_matrix = (T_w_a.inverse() *
            trajectory_.evaluate(it_before->time_ns)).getTransformationMatrix().cast<float>();
        correctTransformationMatrix(&transformation_matrix);
        sub_map.concatenate(rigid_transformation_->compute(it_before->scan,transformation_matrix));
      }
    }
  }

  // Add the scans with increasing time stamps.
  bool reached_end = false;
  for (unsigned int i = 0u; i < sub_maps_radius; ++i) {
    ++it_after;
    if (it_after != laser_scans_.end() && !reached_end) {
      transformation_matrix = (T_w_a.inverse() *
          trajectory_.evaluate(it_after->time_ns)).getTransformationMatrix().cast<float>();
      correctTransformationMatrix(&transformation_matrix);
      sub_map.concatenate(rigid_transformation_->compute(it_after->scan,transformation_matrix));
    } else {
      reached_end = true;
    }
  }

  //TODO move to?
  *sub_map_out = sub_map;
}

} // namespace laser_slam
